/*
 * Fastmix.cpp
 * -----------
 * Purpose: Mixer core for rendering samples, mixing plugins, etc...
 * Notes  : If this is Fastmix.cpp, where is Slowmix.cpp? :)
 * Authors: Olivier Lapicque
 *          OpenMPT Devs
 * The OpenMPT source code is released under the BSD license. Read LICENSE for more details.
 */


// FIXME:
// - Playing samples backwards should reverse interpolation LUTs for interpolation modes
//   with more than two taps since they're not symmetric. We might need separate LUTs
//   because otherwise we will add tons of branches.
// - Loop wraparound works pretty well in general, but not at the start of bidi samples.
// - The loop lookahead stuff might still fail for samples with backward loops.

#include "stdafx.h"
#include "Sndfile.h"
#include "MixerLoops.h"
#include "MixFuncTable.h"
#ifdef MPT_INTMIXER
#include "IntMixer.h"
#else
#include "FloatMixer.h"
#endif
#include "plugins/PlugInterface.h"
#include <cfloat>  // For FLT_EPSILON
#include <algorithm>


OPENMPT_NAMESPACE_BEGIN


/////////////////////////////////////////////////////////////////////////

struct MixLoopState
{
	const int8 * samplePointer = nullptr;
	const int8 * lookaheadPointer = nullptr;
	SmpLength lookaheadStart = 0;
	uint32 maxSamples = 0;
	const uint8 ITPingPongDiff;
	const bool precisePingPongLoops;

	MixLoopState(const CSoundFile &sndFile, const ModChannel &chn)
		: ITPingPongDiff{sndFile.m_playBehaviour[kITPingPongMode] ? uint8(1) : uint8(0)}
		, precisePingPongLoops{!sndFile.m_playBehaviour[kImprecisePingPongLoops]}
	{
		if(chn.pCurrentSample == nullptr)
			return;

		UpdateLookaheadPointers(chn);

		// For platforms that have no fast 64-bit division, precompute this constant
		// as it won't change during the invocation of CreateStereoMix.
		SamplePosition increment = chn.increment;
		if(increment.IsNegative())
			increment.Negate();
		maxSamples = 16384u / (increment.GetUInt() + 1u);
		if(maxSamples < 2)
			maxSamples = 2;
	}

	// Calculate offset of loop wrap-around buffer for this sample.
	void UpdateLookaheadPointers(const ModChannel &chn)
	{
		samplePointer = static_cast<const int8 *>(chn.pCurrentSample);
		lookaheadPointer = nullptr;
		if(!samplePointer)
			return;
		if(chn.nLoopEnd < InterpolationLookaheadBufferSize)
			lookaheadStart = chn.nLoopStart;
		else
			lookaheadStart = std::max(chn.nLoopStart, chn.nLoopEnd - InterpolationLookaheadBufferSize);
		// We only need to apply the loop wrap-around logic if the sample is actually looping.
		// As we round rather than truncate with No Interpolation, we also need the wrap-around logic for samples that are not interpolated.
		if(chn.dwFlags[CHN_LOOP])
		{
			const bool inSustainLoop = chn.InSustainLoop() && chn.nLoopStart == chn.pModSample->nSustainStart && chn.nLoopEnd == chn.pModSample->nSustainEnd;

			// Do not enable wraparound magic if we're previewing a custom loop!
			if(inSustainLoop || (chn.nLoopStart == chn.pModSample->nLoopStart && chn.nLoopEnd == chn.pModSample->nLoopEnd))
			{
				SmpLength lookaheadOffset = 3 * InterpolationLookaheadBufferSize + chn.pModSample->nLength - chn.nLoopEnd;
				if(inSustainLoop)
				{
					lookaheadOffset += 4 * InterpolationLookaheadBufferSize;
				}
				lookaheadPointer = samplePointer + lookaheadOffset * chn.pModSample->GetBytesPerSample();
			}
		}
	}

	// Returns the buffer length required to render a certain amount of samples, based on the channel's playback speed.
	MPT_ATTR_ALWAYSINLINE MPT_INLINE_FORCE static uint32 DistanceToBufferLength(SamplePosition from, SamplePosition to, SamplePosition inc)
	{
		return static_cast<uint32>((to - from - SamplePosition(1)) / inc) + 1;
	}

	// Check how many samples can be rendered without encountering loop or sample end, and also update loop position / direction
	MPT_ATTR_ALWAYSINLINE MPT_INLINE_FORCE uint32 GetSampleCount(ModChannel &chn, uint32 nSamples) const
	{
		const int32 nLoopStart = chn.dwFlags[CHN_LOOP] ? chn.nLoopStart : 0;
		SamplePosition nInc = chn.increment;

		if(nSamples <= 0 || nInc.IsZero() || !chn.nLength || !samplePointer)
			return 0;

		// Part 1: Making sure the play position is valid, and if necessary, invert the play direction in case we reached a loop boundary of a ping-pong loop.
		chn.pCurrentSample = samplePointer;

		// Under zero ?
		if (chn.position.GetInt() < nLoopStart)
		{
			if (nInc.IsNegative())
			{
				// Invert loop direction for bidi loops
				chn.position = SamplePosition(nLoopStart + nLoopStart, 0) - chn.position;
				if ((chn.position.GetInt() < nLoopStart) || (chn.position.GetUInt() >= (nLoopStart + chn.nLength) / 2))
				{
					chn.position.Set(nLoopStart, 0);
				}
				if(chn.dwFlags[CHN_PINGPONGLOOP])
				{
					chn.dwFlags.reset(CHN_PINGPONGFLAG); // go forward
					nInc.Negate();
					chn.increment = nInc;
				} else
				{
					chn.position.SetInt(chn.nLength - 1);
				}
				if(!chn.dwFlags[CHN_LOOP] || chn.position.GetUInt() >= chn.nLength)
				{
					chn.position.Set(chn.nLength);
					return 0;
				}
			} else
			{
				// We probably didn't hit the loop end yet (first loop), so we do nothing
				if (chn.position.GetInt() < 0) chn.position.SetInt(0);
			}
		} else if (chn.position.GetUInt() >= chn.nLength)
		{
			// Past the end
			if(!chn.dwFlags[CHN_LOOP])
				return 0; // not looping -> stop this channel
			if(chn.dwFlags[CHN_PINGPONGLOOP])
			{
				// Invert loop
				if (nInc.IsPositive())
				{
					nInc.Negate();
					chn.increment = nInc;
				}
				chn.dwFlags.set(CHN_PINGPONGFLAG);

				// Adjust loop position
				if(precisePingPongLoops)
				{
					// More accurate loop end overshoot calculation.
					// Test cases: BidiPrecision.it, BidiPrecision.xm
					const auto overshoot = chn.position - SamplePosition(chn.nLength, 0);
					const auto loopLength = chn.nLoopEnd - chn.nLoopStart - ITPingPongDiff;
					if(overshoot.GetUInt() < loopLength)
						chn.position = SamplePosition(chn.nLength - ITPingPongDiff, 0) - overshoot;
					else
						chn.position = SamplePosition(chn.nLoopStart, 0);
				} else
				{
					SamplePosition invFract = chn.position.GetInvertedFract();
					chn.position = SamplePosition(chn.nLength - (chn.position.GetInt() - chn.nLength) - invFract.GetInt(), invFract.GetFract());
					if(chn.position.GetUInt() <= chn.nLoopStart || chn.position.GetUInt() >= chn.nLength)
					{
						// Impulse Tracker's software mixer would put a -2 (instead of -1) in the following line (doesn't happen on a GUS)
						chn.position.SetInt(chn.nLength - std::min(chn.nLength, static_cast<SmpLength>(ITPingPongDiff + 1)));
					}
				}
			} else
			{
				if (nInc.IsNegative()) // This is a bug
				{
					nInc.Negate();
					chn.increment = nInc;
				}
				// Restart at loop start
				chn.position += SamplePosition(nLoopStart - chn.nLength, 0);
				MPT_ASSERT(chn.position.GetInt() >= nLoopStart);
				// Interpolate correctly after wrapping around
				chn.dwFlags.set(CHN_WRAPPED_LOOP);
			}
		}

		// Part 2: Compute how many samples we can render until we reach the end of sample / loop boundary / etc.

		SamplePosition nPos = chn.position;
		const SmpLength nPosInt = nPos.GetUInt();
		if(nPos.GetInt() < nLoopStart)
		{
			// too big increment, and/or too small loop length
			if(nPos.IsNegative() || nInc.IsNegative())
				return 0;
		} else
		{
			// Not testing for equality since we might be going backwards from the very end of the sample
			if(nPosInt > chn.nLength)
				return 0;
			// If going forwards and we're preceisely at the end, there's no point in going further
			if(nPosInt == chn.nLength && nInc.IsPositive())
				return 0;
		}
		uint32 nSmpCount = nSamples;
		SamplePosition nInv = nInc;
		if (nInc.IsNegative())
		{
			nInv.Negate();
		}
		LimitMax(nSamples, maxSamples);
		SamplePosition incSamples = nInc * (nSamples - 1);
		int32 nPosDest = (nPos + incSamples).GetInt();

		const bool isAtLoopStart = (nPosInt >= chn.nLoopStart && nPosInt < chn.nLoopStart + InterpolationLookaheadBufferSize);
		if(!isAtLoopStart)
		{
			chn.dwFlags.reset(CHN_WRAPPED_LOOP);
		}

		// Loop wrap-around magic.
		bool checkDest = true;
		if(lookaheadPointer != nullptr)
		{
			if(nPosInt >= lookaheadStart)
			{
				if(nInc.IsNegative())
				{
					nSmpCount = DistanceToBufferLength(SamplePosition(lookaheadStart, 0), nPos, nInv);
					chn.pCurrentSample = lookaheadPointer;
				} else if(nPosInt <= chn.nLoopEnd)
				{
					nSmpCount = DistanceToBufferLength(nPos, SamplePosition(chn.nLoopEnd, 0), nInv);
					chn.pCurrentSample = lookaheadPointer;
				} else
				{
					nSmpCount = DistanceToBufferLength(nPos, SamplePosition(chn.nLength, 0), nInv);
				}
				checkDest = false;
			} else if(chn.dwFlags[CHN_WRAPPED_LOOP] && isAtLoopStart)
			{
				// We just restarted the loop, so interpolate correctly after wrapping around
				nSmpCount = DistanceToBufferLength(nPos, SamplePosition(nLoopStart + InterpolationLookaheadBufferSize, 0), nInv);
				chn.pCurrentSample = lookaheadPointer + (chn.nLength - nLoopStart) * chn.pModSample->GetBytesPerSample();
				checkDest = false;
			} else if(nInc.IsPositive() && static_cast<SmpLength>(nPosDest) >= lookaheadStart && nSmpCount > 1)
			{
				// We shouldn't read that far if we're not using the pre-computed wrap-around buffer.
				nSmpCount = DistanceToBufferLength(nPos, SamplePosition(lookaheadStart, 0), nInv);
				checkDest = false;
			}
		}

		if(checkDest)
		{
			// Fix up sample count if target position is invalid
			if (nInc.IsNegative())
			{
				if (nPosDest < nLoopStart)
				{
					nSmpCount = DistanceToBufferLength(SamplePosition(nLoopStart, 0), nPos, nInv);
				}
			} else
			{
				if (nPosDest >= (int32)chn.nLength)
				{
					nSmpCount = DistanceToBufferLength(nPos, SamplePosition(chn.nLength, 0), nInv);
				}
			}
		}

		Limit(nSmpCount, uint32(1u), nSamples);

#ifdef MPT_BUILD_DEBUG
		{
			SmpLength posDest = (nPos + nInc * (nSmpCount - 1)).GetUInt();
			MPT_MAYBE_CONSTANT_IF(posDest < 0 || posDest > chn.nLength)
			{
				// We computed an invalid delta!
				MPT_ASSERT_NOTREACHED();
				return 0;
			}
		}
#endif

		return nSmpCount;
	}
};


// Render count * number of channels samples
void CSoundFile::CreateStereoMix(int count)
{
	if(!count)
		return;

	// Resetting sound buffer
	StereoFill(MixSoundBuffer, count, m_dryROfsVol, m_dryLOfsVol);
	if(m_MixerSettings.gnChannels > 2)
		StereoFill(MixRearBuffer, count, m_surroundROfsVol, m_surroundLOfsVol);

	// Channels that are actually mixed and not skipped (because they are paused or muted)
	CHANNELINDEX numChannelsMixed = 0;

	for(uint32 nChn = 0; nChn < m_nMixChannels; nChn++)
	{
		if(MixChannel(count, m_PlayState.Chn[m_PlayState.ChnMix[nChn]], m_PlayState.ChnMix[nChn], numChannelsMixed < m_MixerSettings.m_nMaxMixChannels))
			numChannelsMixed++;
	}
	m_nMixStat = std::max(m_nMixStat, numChannelsMixed);
}

void CSoundFile::CreateSurroundMix(int count)
{
	if(!count)
		return;

	// Reset surround buffers
	std::fill(MixSurroundFL, MixSurroundFL + count, 0);
	std::fill(MixSurroundFR, MixSurroundFR + count, 0);
	std::fill(MixSurroundC, MixSurroundC + count, 0);
	std::fill(MixSurroundLFE, MixSurroundLFE + count, 0);
	std::fill(MixSurroundSL, MixSurroundSL + count, 0);
	std::fill(MixSurroundSR, MixSurroundSR + count, 0);

	// Mix channels using independent surround mixer (not stereo path)
	// Surround mixer routes directly to 6 separate buffers
	// Currently: distributes mono equally to all 5 speakers, LFE silent

	// Channels that are actually mixed and not skipped (because they are paused or muted)
	CHANNELINDEX numChannelsMixed = 0;

	for(uint32 nChn = 0; nChn < m_nMixChannels; nChn++)
	{
		if(MixChannelSurround(count, m_PlayState.Chn[m_PlayState.ChnMix[nChn]], m_PlayState.ChnMix[nChn], numChannelsMixed < m_MixerSettings.m_nMaxMixChannels))
			numChannelsMixed++;
	}
	m_nMixStat = std::max(m_nMixStat, numChannelsMixed);

	// Generate LFE channel: sum 5 main channels, lowpass at 120Hz, apply -10dB attenuation
	ProcessLFEChannel(count);
}


std::pair<mixsample_t *, mixsample_t *> CSoundFile::GetChannelOffsets(const ModChannel &chn, CHANNELINDEX channel)
{
	mixsample_t *pOfsR = &m_dryROfsVol;
	mixsample_t *pOfsL = &m_dryLOfsVol;
#ifndef NO_REVERB
	if(((m_MixerSettings.DSPMask & SNDDSP_REVERB) && !chn.dwFlags[CHN_NOREVERB]) || chn.dwFlags[CHN_REVERB])
	{
		pOfsR = &m_RvbROfsVol;
		pOfsL = &m_RvbLOfsVol;
	}
#endif
	if(chn.dwFlags[CHN_SURROUND] && m_MixerSettings.gnChannels > 2)
	{
		pOfsR = &m_surroundROfsVol;
		pOfsL = &m_surroundLOfsVol;
	}
	// Look for plugins associated with this implicit tracker channel.
	const PLUGINDEX mixPlugin = GetBestPlugin(chn, channel, PrioritiseInstrument, RespectMutes);
	if((mixPlugin > 0) && (mixPlugin <= MAX_MIXPLUGINS) && m_MixPlugins[mixPlugin - 1].pMixPlugin != nullptr)
	{
		// Render into plugin buffer instead of global buffer
		SNDMIXPLUGINSTATE &mixState = m_MixPlugins[mixPlugin - 1].pMixPlugin->m_MixState;
		if(mixState.pMixBuffer)
		{
			pOfsR = &mixState.nVolDecayR;
			pOfsL = &mixState.nVolDecayL;
		}
	}
	return std::make_pair(pOfsL, pOfsR);
}


bool CSoundFile::MixChannel(int count, ModChannel &chn, CHANNELINDEX channel, bool doMix)
{
	if(chn.pCurrentSample || chn.nLOfs || chn.nROfs)
	{
		auto [pOfsL, pOfsR] = GetChannelOffsets(chn, channel);

		uint32 functionNdx = MixFuncTable::ResamplingModeToMixFlags(static_cast<ResamplingMode>(chn.resamplingMode));
		if(chn.dwFlags[CHN_16BIT]) functionNdx |= MixFuncTable::ndx16Bit;
		if(chn.dwFlags[CHN_STEREO]) functionNdx |= MixFuncTable::ndxStereo;
#ifndef NO_FILTER
		if(chn.dwFlags[CHN_FILTER]) functionNdx |= MixFuncTable::ndxFilter;
#endif

		mixsample_t *pbuffer = MixSoundBuffer;
#ifndef NO_REVERB
		if(((m_MixerSettings.DSPMask & SNDDSP_REVERB) && !chn.dwFlags[CHN_NOREVERB]) || chn.dwFlags[CHN_REVERB])
		{
			m_Reverb.TouchReverbSendBuffer(ReverbSendBuffer, m_RvbROfsVol, m_RvbLOfsVol, count);
			pbuffer = ReverbSendBuffer;
		}
#endif
		if(chn.dwFlags[CHN_SURROUND] && m_MixerSettings.gnChannels > 2)
		{
			pbuffer = MixRearBuffer;
		}

		// Look for plugins associated with this implicit tracker channel.
		const PLUGINDEX mixPlugin = GetBestPlugin(chn, channel, PrioritiseInstrument, RespectMutes);
		if((mixPlugin > 0) && (mixPlugin <= MAX_MIXPLUGINS) && m_MixPlugins[mixPlugin - 1].pMixPlugin != nullptr)
		{
			// Render into plugin buffer instead of global buffer
			SNDMIXPLUGINSTATE &mixState = m_MixPlugins[mixPlugin - 1].pMixPlugin->m_MixState;
			if (mixState.pMixBuffer)
			{
				pbuffer = mixState.pMixBuffer;
				if (!(mixState.dwFlags & SNDMIXPLUGINSTATE::psfMixReady))
				{
					StereoFill(pbuffer, count, *pOfsR, *pOfsL);
					mixState.dwFlags |= SNDMIXPLUGINSTATE::psfMixReady;
				}
			}
		}

		if(chn.isPaused)
		{
			EndChannelOfs(chn, pbuffer, count);
			*pOfsR += chn.nROfs;
			*pOfsL += chn.nLOfs;
			chn.nROfs = chn.nLOfs = 0;
			return false;
		}

		MixLoopState mixLoopState(*this, chn);

		////////////////////////////////////////////////////
		bool addToMix = false;
		int nsamples = count;
		// Keep mixing this sample until the buffer is filled.
		do
		{
			uint32 nrampsamples = nsamples;
			int32 nSmpCount;
			if(chn.nRampLength > 0)
			{
				if (nrampsamples > chn.nRampLength) nrampsamples = chn.nRampLength;
			}

			if((nSmpCount = mixLoopState.GetSampleCount(chn, nrampsamples)) <= 0)
			{
				// Stopping the channel
				chn.pCurrentSample = nullptr;
				chn.nLength = 0;
				chn.position.Set(0);
				chn.nRampLength = 0;
				EndChannelOfs(chn, pbuffer, nsamples);
				*pOfsR += chn.nROfs;
				*pOfsL += chn.nLOfs;
				chn.nROfs = chn.nLOfs = 0;
				chn.dwFlags.reset(CHN_PINGPONGFLAG);
				break;
			}

			// Should we mix this channel?
			if(!doMix                                                   // Too many channels
			   || (!chn.nRampLength && !(chn.leftVol | chn.rightVol)))  // Channel is completely silent
			{
				chn.position += chn.increment * nSmpCount;
				chn.nROfs = chn.nLOfs = 0;
				pbuffer += nSmpCount * 2;
				addToMix = false;
			}
#ifdef MODPLUG_TRACKER
			else if(m_SamplePlayLengths != nullptr)
			{
				// Detecting the longest play time for each sample for optimization
				SmpLength pos = chn.position.GetUInt();
				chn.position += chn.increment * nSmpCount;
				if(!chn.increment.IsNegative())
				{
					pos = chn.position.GetUInt();
				}
				size_t smp = std::distance(static_cast<const ModSample*>(static_cast<std::decay<decltype(Samples)>::type>(Samples)), chn.pModSample);
				if(smp < m_SamplePlayLengths->size())
				{
					(*m_SamplePlayLengths)[smp] = std::max((*m_SamplePlayLengths)[smp], pos);
				}
			}
#endif
			else
			{
				// Do mixing
				mixsample_t *pbufmax = pbuffer + (nSmpCount * 2);
				chn.nROfs = -*(pbufmax - 2);
				chn.nLOfs = -*(pbufmax - 1);

#ifdef MPT_BUILD_DEBUG
				SamplePosition targetpos = chn.position + chn.increment * nSmpCount;
#endif
				MixFuncTable::Functions[functionNdx | (chn.nRampLength ? MixFuncTable::ndxRamp : 0)](chn, m_Resampler, pbuffer, nSmpCount);
#ifdef MPT_BUILD_DEBUG
				MPT_ASSERT(chn.position.GetUInt() == targetpos.GetUInt());
#endif

				chn.nROfs += *(pbufmax - 2);
				chn.nLOfs += *(pbufmax - 1);
				pbuffer = pbufmax;
				addToMix = true;
			}

			nsamples -= nSmpCount;
			if (chn.nRampLength)
			{
				if (chn.nRampLength <= static_cast<uint32>(nSmpCount))
				{
					// Ramping is done
					chn.nRampLength = 0;
					chn.leftVol = chn.newLeftVol;
					chn.rightVol = chn.newRightVol;
					chn.rightRamp = chn.leftRamp = 0;
					if(chn.dwFlags[CHN_NOTEFADE] && !chn.nFadeOutVol)
					{
						chn.nLength = 0;
						chn.pCurrentSample = nullptr;
					}
				} else
				{
					chn.nRampLength -= nSmpCount;
				}
			}

			const bool pastLoopEnd = chn.position.GetUInt() >= chn.nLoopEnd && chn.dwFlags[CHN_LOOP];
			const bool pastSampleEnd = chn.position.GetUInt() >= chn.nLength && !chn.dwFlags[CHN_LOOP] && chn.nLength && !chn.nMasterChn;
			const bool doSampleSwap = m_playBehaviour[kMODSampleSwap] && chn.swapSampleIndex && chn.swapSampleIndex <= GetNumSamples() && chn.pModSample != &Samples[chn.swapSampleIndex];
			if((pastLoopEnd || pastSampleEnd) && doSampleSwap)
			{
				// ProTracker compatibility: Instrument changes without a note do not happen instantly, but rather when the sample loop has finished playing.
				// Test case: PTInstrSwap.mod, PTSwapNoLoop.mod
#ifdef MODPLUG_TRACKER
				if(m_SamplePlayLengths != nullptr)
				{
					// Even if the sample was playing at zero volume, we need to retain its full length for correct sample swap timing
					size_t smp = std::distance(static_cast<const ModSample *>(static_cast<std::decay<decltype(Samples)>::type>(Samples)), chn.pModSample);
					if(smp < m_SamplePlayLengths->size())
					{
						(*m_SamplePlayLengths)[smp] = std::max((*m_SamplePlayLengths)[smp], std::min(chn.nLength, chn.position.GetUInt()));
					}
				}
#endif
				const ModSample &smp = Samples[chn.swapSampleIndex];
				chn.pModSample = &smp;
				chn.pCurrentSample = smp.samplev();
				chn.dwFlags = (chn.dwFlags & CHN_CHANNELFLAGS) | smp.uFlags;
				if(smp.uFlags[CHN_LOOP])
					chn.nLength = smp.nLoopEnd;
				else if(!m_playBehaviour[kMODOneShotLoops])
					chn.nLength = smp.nLength;
				else
					chn.nLength = 0; // non-looping sample continue in oneshot mode (i.e. they will most probably just play silence)
				chn.nLoopStart = smp.nLoopStart;
				chn.nLoopEnd = smp.nLoopEnd;
				chn.position.SetInt(chn.nLoopStart);
				chn.swapSampleIndex = 0;
				mixLoopState.UpdateLookaheadPointers(chn);
				if(!chn.pCurrentSample)
					break;
			} else if(pastLoopEnd && !doSampleSwap && m_playBehaviour[kMODOneShotLoops] && chn.nLoopStart == 0)
			{
				// ProTracker "oneshot" loops (if loop start is 0, play the whole sample once and then repeat until loop end)
				chn.position.SetInt(0);
				chn.nLoopEnd = chn.nLength = chn.pModSample->nLoopEnd;
			}
		} while(nsamples > 0);

		// Restore sample pointer in case it got changed through loop wrap-around
		chn.pCurrentSample = mixLoopState.samplePointer;
	
		if(addToMix && mixPlugin > 0 && mixPlugin <= MAX_MIXPLUGINS && m_MixPlugins[mixPlugin - 1].pMixPlugin)
		{
			m_MixPlugins[mixPlugin - 1].pMixPlugin->ResetSilence();
		}
		return addToMix;
	}
	return false;
}


// Surround mixer: separate from stereo path, routes to 6 independent channels
// No offset tracking, no reverb routing, direct buffer accumulation
bool CSoundFile::MixChannelSurround(int count, ModChannel &chn, CHANNELINDEX channel, bool doMix)
{
	if(chn.pCurrentSample || chn.nLOfs || chn.nROfs)
	{
		// Surround mixer does not use reverb, plugin, or surround flag routing
		// All samples accumulate directly to surround buffers

		// Look for plugins associated with this implicit tracker channel.
		// Note: surround mixer currently skips plugin buffer handling.

		if(chn.isPaused)
		{
			// Surround: no offset tracking, no cleanup needed
			chn.nROfs = chn.nLOfs = 0;
			return false;
		}

		MixLoopState mixLoopState(*this, chn);

		////////////////////////////////////////////////////
		bool addToMix = false;
		int nsamples = count;
		
		int surrBufOffset = 0;  // Current position in surround buffers
		
		// Keep mixing this sample until the buffer is filled.
		do
		{
			uint32 nrampsamples = nsamples;
			int32 nSmpCount;
			if(chn.nRampLength > 0)
			{
				if (nrampsamples > chn.nRampLength) nrampsamples = chn.nRampLength;
			}

			if((nSmpCount = mixLoopState.GetSampleCount(chn, nrampsamples)) <= 0)
			{
				// Stopping the channel
				chn.pCurrentSample = nullptr;
				chn.nLength = 0;
				chn.position.Set(0);
				chn.nRampLength = 0;
				// Surround: no EndChannelOfs needed
				chn.nROfs = chn.nLOfs = 0;
				chn.dwFlags.reset(CHN_PINGPONGFLAG);
				break;
			}

			// Should we mix this channel?
			if(!doMix                                                   // Too many channels
			   || (!chn.nRampLength && !(chn.leftVol | chn.rightVol)))  // Channel is completely silent
			{
				chn.position += chn.increment * nSmpCount;
				chn.nROfs = chn.nLOfs = 0;
				// Surround: advance buffer offset even when skipping (critical for buffer alignment)
				surrBufOffset += nSmpCount;
				addToMix = false;
			}
#ifdef MODPLUG_TRACKER
			else if(m_SamplePlayLengths != nullptr)
			{
				// Detecting the longest play time for each sample for optimization
				SmpLength pos = chn.position.GetUInt();
				chn.position += chn.increment * nSmpCount;
				if(!chn.increment.IsNegative())
				{
					pos = chn.position.GetUInt();
				}
				size_t smp = std::distance(static_cast<const ModSample*>(static_cast<std::decay<decltype(Samples)>::type>(Samples)), chn.pModSample);
				if(smp < m_SamplePlayLengths->size())
				{
					(*m_SamplePlayLengths)[smp] = std::max((*m_SamplePlayLengths)[smp], pos);
				}
			}
#endif
			else
			{
				// Surround panning gains are computed in Sndmix.cpp during the panning setup phase
				// The mixer will apply chn.newSurroundSL/FL/C/FR/SR per-sample

				// Prepare interleaved 5-channel buffer (SL, FL, C, FR, SR)
				mixsample_t surround5Buffer[MIXBUFFERSIZE * 5];
				std::fill(surround5Buffer, surround5Buffer + (nSmpCount * 5), mixsample_t(0));

#ifdef MPT_BUILD_DEBUG
				SamplePosition targetpos = chn.position + chn.increment * nSmpCount;
#endif

				const ResamplingMode resamplingMode = static_cast<ResamplingMode>(chn.resamplingMode);
				const bool is16Bit = chn.dwFlags[CHN_16BIT];
				const bool isStereo = chn.dwFlags[CHN_STEREO];
				const bool ramp = chn.nRampLength != 0;

				if(resamplingMode == SRCMODE_NEAREST)
				{
					if(!is16Bit && !isStereo)
#ifdef MPT_INTMIXER
						if(ramp)
							SampleLoop<Int8MToInt5, NoInterpolation<Int8MToInt5>, NoFilter<Int8MToInt5>, MixSurroundRamp<Int8MToInt5>>(chn, m_Resampler, surround5Buffer, nSmpCount);
						else
							SampleLoop<Int8MToInt5, NoInterpolation<Int8MToInt5>, NoFilter<Int8MToInt5>, MixSurroundNoRamp<Int8MToInt5>>(chn, m_Resampler, surround5Buffer, nSmpCount);
#else
						if(ramp)
							SampleLoop<Int8MToFloat5, NoInterpolation<Int8MToFloat5>, NoFilter<Int8MToFloat5>, MixSurroundRamp<Int8MToFloat5>>(chn, m_Resampler, surround5Buffer, nSmpCount);
						else
							SampleLoop<Int8MToFloat5, NoInterpolation<Int8MToFloat5>, NoFilter<Int8MToFloat5>, MixSurroundNoRamp<Int8MToFloat5>>(chn, m_Resampler, surround5Buffer, nSmpCount);
#endif
					else if(is16Bit && !isStereo)
#ifdef MPT_INTMIXER
						if(ramp)
							SampleLoop<Int16MToInt5, NoInterpolation<Int16MToInt5>, NoFilter<Int16MToInt5>, MixSurroundRamp<Int16MToInt5>>(chn, m_Resampler, surround5Buffer, nSmpCount);
						else
							SampleLoop<Int16MToInt5, NoInterpolation<Int16MToInt5>, NoFilter<Int16MToInt5>, MixSurroundNoRamp<Int16MToInt5>>(chn, m_Resampler, surround5Buffer, nSmpCount);
#else
						if(ramp)
							SampleLoop<Int16MToFloat5, NoInterpolation<Int16MToFloat5>, NoFilter<Int16MToFloat5>, MixSurroundRamp<Int16MToFloat5>>(chn, m_Resampler, surround5Buffer, nSmpCount);
						else
							SampleLoop<Int16MToFloat5, NoInterpolation<Int16MToFloat5>, NoFilter<Int16MToFloat5>, MixSurroundNoRamp<Int16MToFloat5>>(chn, m_Resampler, surround5Buffer, nSmpCount);
#endif
					else if(!is16Bit && isStereo)
#ifdef MPT_INTMIXER
						if(ramp)
							SampleLoop<Int8SToInt5, NoInterpolation<Int8SToInt5>, NoFilter<Int8SToInt5>, MixSurroundRamp<Int8SToInt5>>(chn, m_Resampler, surround5Buffer, nSmpCount);
						else
							SampleLoop<Int8SToInt5, NoInterpolation<Int8SToInt5>, NoFilter<Int8SToInt5>, MixSurroundNoRamp<Int8SToInt5>>(chn, m_Resampler, surround5Buffer, nSmpCount);
#else
						if(ramp)
							SampleLoop<Int8SToFloat5, NoInterpolation<Int8SToFloat5>, NoFilter<Int8SToFloat5>, MixSurroundRamp<Int8SToFloat5>>(chn, m_Resampler, surround5Buffer, nSmpCount);
						else
							SampleLoop<Int8SToFloat5, NoInterpolation<Int8SToFloat5>, NoFilter<Int8SToFloat5>, MixSurroundNoRamp<Int8SToFloat5>>(chn, m_Resampler, surround5Buffer, nSmpCount);
#endif
					else
#ifdef MPT_INTMIXER
						if(ramp)
							SampleLoop<Int16SToInt5, NoInterpolation<Int16SToInt5>, NoFilter<Int16SToInt5>, MixSurroundRamp<Int16SToInt5>>(chn, m_Resampler, surround5Buffer, nSmpCount);
						else
							SampleLoop<Int16SToInt5, NoInterpolation<Int16SToInt5>, NoFilter<Int16SToInt5>, MixSurroundNoRamp<Int16SToInt5>>(chn, m_Resampler, surround5Buffer, nSmpCount);
#else
						if(ramp)
							SampleLoop<Int16SToFloat5, NoInterpolation<Int16SToFloat5>, NoFilter<Int16SToFloat5>, MixSurroundRamp<Int16SToFloat5>>(chn, m_Resampler, surround5Buffer, nSmpCount);
						else
							SampleLoop<Int16SToFloat5, NoInterpolation<Int16SToFloat5>, NoFilter<Int16SToFloat5>, MixSurroundNoRamp<Int16SToFloat5>>(chn, m_Resampler, surround5Buffer, nSmpCount);
#endif
				}
				else if(resamplingMode == SRCMODE_LINEAR)
				{
					if(!is16Bit && !isStereo)
#ifdef MPT_INTMIXER
						if(ramp)
							SampleLoop<Int8MToInt5, LinearInterpolation<Int8MToInt5>, NoFilter<Int8MToInt5>, MixSurroundRamp<Int8MToInt5>>(chn, m_Resampler, surround5Buffer, nSmpCount);
						else
							SampleLoop<Int8MToInt5, LinearInterpolation<Int8MToInt5>, NoFilter<Int8MToInt5>, MixSurroundNoRamp<Int8MToInt5>>(chn, m_Resampler, surround5Buffer, nSmpCount);
#else
						if(ramp)
							SampleLoop<Int8MToFloat5, LinearInterpolation<Int8MToFloat5>, NoFilter<Int8MToFloat5>, MixSurroundRamp<Int8MToFloat5>>(chn, m_Resampler, surround5Buffer, nSmpCount);
						else
							SampleLoop<Int8MToFloat5, LinearInterpolation<Int8MToFloat5>, NoFilter<Int8MToFloat5>, MixSurroundNoRamp<Int8MToFloat5>>(chn, m_Resampler, surround5Buffer, nSmpCount);
#endif
					else if(is16Bit && !isStereo)
#ifdef MPT_INTMIXER
						if(ramp)
							SampleLoop<Int16MToInt5, LinearInterpolation<Int16MToInt5>, NoFilter<Int16MToInt5>, MixSurroundRamp<Int16MToInt5>>(chn, m_Resampler, surround5Buffer, nSmpCount);
						else
							SampleLoop<Int16MToInt5, LinearInterpolation<Int16MToInt5>, NoFilter<Int16MToInt5>, MixSurroundNoRamp<Int16MToInt5>>(chn, m_Resampler, surround5Buffer, nSmpCount);
#else
						if(ramp)
							SampleLoop<Int16MToFloat5, LinearInterpolation<Int16MToFloat5>, NoFilter<Int16MToFloat5>, MixSurroundRamp<Int16MToFloat5>>(chn, m_Resampler, surround5Buffer, nSmpCount);
						else
							SampleLoop<Int16MToFloat5, LinearInterpolation<Int16MToFloat5>, NoFilter<Int16MToFloat5>, MixSurroundNoRamp<Int16MToFloat5>>(chn, m_Resampler, surround5Buffer, nSmpCount);
#endif
					else if(!is16Bit && isStereo)
#ifdef MPT_INTMIXER
						if(ramp)
							SampleLoop<Int8SToInt5, LinearInterpolation<Int8SToInt5>, NoFilter<Int8SToInt5>, MixSurroundRamp<Int8SToInt5>>(chn, m_Resampler, surround5Buffer, nSmpCount);
						else
							SampleLoop<Int8SToInt5, LinearInterpolation<Int8SToInt5>, NoFilter<Int8SToInt5>, MixSurroundNoRamp<Int8SToInt5>>(chn, m_Resampler, surround5Buffer, nSmpCount);
#else
						if(ramp)
							SampleLoop<Int8SToFloat5, LinearInterpolation<Int8SToFloat5>, NoFilter<Int8SToFloat5>, MixSurroundRamp<Int8SToFloat5>>(chn, m_Resampler, surround5Buffer, nSmpCount);
						else
							SampleLoop<Int8SToFloat5, LinearInterpolation<Int8SToFloat5>, NoFilter<Int8SToFloat5>, MixSurroundNoRamp<Int8SToFloat5>>(chn, m_Resampler, surround5Buffer, nSmpCount);
#endif
					else
#ifdef MPT_INTMIXER
						if(ramp)
							SampleLoop<Int16SToInt5, LinearInterpolation<Int16SToInt5>, NoFilter<Int16SToInt5>, MixSurroundRamp<Int16SToInt5>>(chn, m_Resampler, surround5Buffer, nSmpCount);
						else
							SampleLoop<Int16SToInt5, LinearInterpolation<Int16SToInt5>, NoFilter<Int16SToInt5>, MixSurroundNoRamp<Int16SToInt5>>(chn, m_Resampler, surround5Buffer, nSmpCount);
#else
						if(ramp)
							SampleLoop<Int16SToFloat5, LinearInterpolation<Int16SToFloat5>, NoFilter<Int16SToFloat5>, MixSurroundRamp<Int16SToFloat5>>(chn, m_Resampler, surround5Buffer, nSmpCount);
						else
							SampleLoop<Int16SToFloat5, LinearInterpolation<Int16SToFloat5>, NoFilter<Int16SToFloat5>, MixSurroundNoRamp<Int16SToFloat5>>(chn, m_Resampler, surround5Buffer, nSmpCount);
#endif
				}
				else
				{
					// Default to linear for other modes (TODO: add cubic/sinc support)
					if(!is16Bit && !isStereo)
#ifdef MPT_INTMIXER
						if(ramp)
							SampleLoop<Int8MToInt5, LinearInterpolation<Int8MToInt5>, NoFilter<Int8MToInt5>, MixSurroundRamp<Int8MToInt5>>(chn, m_Resampler, surround5Buffer, nSmpCount);
						else
							SampleLoop<Int8MToInt5, LinearInterpolation<Int8MToInt5>, NoFilter<Int8MToInt5>, MixSurroundNoRamp<Int8MToInt5>>(chn, m_Resampler, surround5Buffer, nSmpCount);
#else
						if(ramp)
							SampleLoop<Int8MToFloat5, LinearInterpolation<Int8MToFloat5>, NoFilter<Int8MToFloat5>, MixSurroundRamp<Int8MToFloat5>>(chn, m_Resampler, surround5Buffer, nSmpCount);
						else
							SampleLoop<Int8MToFloat5, LinearInterpolation<Int8MToFloat5>, NoFilter<Int8MToFloat5>, MixSurroundNoRamp<Int8MToFloat5>>(chn, m_Resampler, surround5Buffer, nSmpCount);
#endif
					else if(is16Bit && !isStereo)
#ifdef MPT_INTMIXER
						if(ramp)
							SampleLoop<Int16MToInt5, LinearInterpolation<Int16MToInt5>, NoFilter<Int16MToInt5>, MixSurroundRamp<Int16MToInt5>>(chn, m_Resampler, surround5Buffer, nSmpCount);
						else
							SampleLoop<Int16MToInt5, LinearInterpolation<Int16MToInt5>, NoFilter<Int16MToInt5>, MixSurroundNoRamp<Int16MToInt5>>(chn, m_Resampler, surround5Buffer, nSmpCount);
#else
						if(ramp)
							SampleLoop<Int16MToFloat5, LinearInterpolation<Int16MToFloat5>, NoFilter<Int16MToFloat5>, MixSurroundRamp<Int16MToFloat5>>(chn, m_Resampler, surround5Buffer, nSmpCount);
						else
							SampleLoop<Int16MToFloat5, LinearInterpolation<Int16MToFloat5>, NoFilter<Int16MToFloat5>, MixSurroundNoRamp<Int16MToFloat5>>(chn, m_Resampler, surround5Buffer, nSmpCount);
#endif
					else if(!is16Bit && isStereo)
#ifdef MPT_INTMIXER
						if(ramp)
							SampleLoop<Int8SToInt5, LinearInterpolation<Int8SToInt5>, NoFilter<Int8SToInt5>, MixSurroundRamp<Int8SToInt5>>(chn, m_Resampler, surround5Buffer, nSmpCount);
						else
							SampleLoop<Int8SToInt5, LinearInterpolation<Int8SToInt5>, NoFilter<Int8SToInt5>, MixSurroundNoRamp<Int8SToInt5>>(chn, m_Resampler, surround5Buffer, nSmpCount);
#else
						if(ramp)
							SampleLoop<Int8SToFloat5, LinearInterpolation<Int8SToFloat5>, NoFilter<Int8SToFloat5>, MixSurroundRamp<Int8SToFloat5>>(chn, m_Resampler, surround5Buffer, nSmpCount);
						else
							SampleLoop<Int8SToFloat5, LinearInterpolation<Int8SToFloat5>, NoFilter<Int8SToFloat5>, MixSurroundNoRamp<Int8SToFloat5>>(chn, m_Resampler, surround5Buffer, nSmpCount);
#endif
					else
#ifdef MPT_INTMIXER
						if(ramp)
							SampleLoop<Int16SToInt5, LinearInterpolation<Int16SToInt5>, NoFilter<Int16SToInt5>, MixSurroundRamp<Int16SToInt5>>(chn, m_Resampler, surround5Buffer, nSmpCount);
						else
							SampleLoop<Int16SToInt5, LinearInterpolation<Int16SToInt5>, NoFilter<Int16SToInt5>, MixSurroundNoRamp<Int16SToInt5>>(chn, m_Resampler, surround5Buffer, nSmpCount);
#else
						if(ramp)
							SampleLoop<Int16SToFloat5, LinearInterpolation<Int16SToFloat5>, NoFilter<Int16SToFloat5>, MixSurroundRamp<Int16SToFloat5>>(chn, m_Resampler, surround5Buffer, nSmpCount);
						else
							SampleLoop<Int16SToFloat5, LinearInterpolation<Int16SToFloat5>, NoFilter<Int16SToFloat5>, MixSurroundNoRamp<Int16SToFloat5>>(chn, m_Resampler, surround5Buffer, nSmpCount);
#endif
				}

#ifdef MPT_BUILD_DEBUG
				MPT_ASSERT(chn.position.GetUInt() == targetpos.GetUInt());
#endif

				// Deinterleave surround buffer into speaker accumulators
				for(int32 i = 0; i < nSmpCount; i++)
				{
					MixSurroundSL[surrBufOffset + i] += surround5Buffer[i * 5 + 0];
					MixSurroundFL[surrBufOffset + i] += surround5Buffer[i * 5 + 1];
					MixSurroundC [surrBufOffset + i] += surround5Buffer[i * 5 + 2];
					MixSurroundFR[surrBufOffset + i] += surround5Buffer[i * 5 + 3];
					MixSurroundSR[surrBufOffset + i] += surround5Buffer[i * 5 + 4];
				}
				surrBufOffset += nSmpCount;
				addToMix = true;
			}

			nsamples -= nSmpCount;
			if (chn.nRampLength)
			{
				if (chn.nRampLength <= static_cast<uint32>(nSmpCount))
				{
					// Ramping is done
					chn.nRampLength = 0;
					chn.leftVol = chn.newLeftVol;
					chn.rightVol = chn.newRightVol;
					chn.rightRamp = chn.leftRamp = 0;
					if(chn.dwFlags[CHN_NOTEFADE] && !chn.nFadeOutVol)
					{
						chn.nLength = 0;
						chn.pCurrentSample = nullptr;
					}
				} else
				{
					chn.nRampLength -= nSmpCount;
				}
			}

			const bool pastLoopEnd = chn.position.GetUInt() >= chn.nLoopEnd && chn.dwFlags[CHN_LOOP];
			const bool pastSampleEnd = chn.position.GetUInt() >= chn.nLength && !chn.dwFlags[CHN_LOOP] && chn.nLength && !chn.nMasterChn;
			const bool doSampleSwap = m_playBehaviour[kMODSampleSwap] && chn.swapSampleIndex && chn.swapSampleIndex <= GetNumSamples() && chn.pModSample != &Samples[chn.swapSampleIndex];
			if((pastLoopEnd || pastSampleEnd) && doSampleSwap)
			{
				// ProTracker compatibility: Instrument changes without a note do not happen instantly, but rather when the sample loop has finished playing.
				// Test case: PTInstrSwap.mod, PTSwapNoLoop.mod
#ifdef MODPLUG_TRACKER
				if(m_SamplePlayLengths != nullptr)
				{
					// Even if the sample was playing at zero volume, we need to retain its full length for correct sample swap timing
					size_t smp = std::distance(static_cast<const ModSample *>(static_cast<std::decay<decltype(Samples)>::type>(Samples)), chn.pModSample);
					if(smp < m_SamplePlayLengths->size())
					{
						(*m_SamplePlayLengths)[smp] = std::max((*m_SamplePlayLengths)[smp], std::min(chn.nLength, chn.position.GetUInt()));
					}
				}
#endif
				const ModSample &smp = Samples[chn.swapSampleIndex];
				chn.pModSample = &smp;
				chn.pCurrentSample = smp.samplev();
				chn.dwFlags = (chn.dwFlags & CHN_CHANNELFLAGS) | smp.uFlags;
				if(smp.uFlags[CHN_LOOP])
					chn.nLength = smp.nLoopEnd;
				else if(!m_playBehaviour[kMODOneShotLoops])
					chn.nLength = smp.nLength;
				else
					chn.nLength = 0; // non-looping sample continue in oneshot mode (i.e. they will most probably just play silence)
				chn.nLoopStart = smp.nLoopStart;
				chn.nLoopEnd = smp.nLoopEnd;
				chn.position.SetInt(chn.nLoopStart);
				chn.swapSampleIndex = 0;
				mixLoopState.UpdateLookaheadPointers(chn);
				if(!chn.pCurrentSample)
					break;
			} else if(pastLoopEnd && !doSampleSwap && m_playBehaviour[kMODOneShotLoops] && chn.nLoopStart == 0)
			{
				// ProTracker "oneshot" loops (if loop start is 0, play the whole sample once and then repeat until loop end)
				chn.position.SetInt(0);
				chn.nLoopEnd = chn.nLength = chn.pModSample->nLoopEnd;
			}
		} while(nsamples > 0);

		// Restore sample pointer in case it got changed through loop wrap-around
		chn.pCurrentSample = mixLoopState.samplePointer;
	
		// Surround: no plugin reset needed (plugins not supported yet)
		return addToMix;
	}
	return false;
}


// Initialize LFE lowpass filter coefficients for 2nd-order Butterworth at 120Hz
void CSoundFile::InitializeLFEFilter()
{
	const uint32 sampleRate = m_MixerSettings.gdwMixingFreq;
	
	// Only recalculate if sample rate changed
	if(m_lfeFilterSampleRate == sampleRate && sampleRate > 0)
		return;
	
	m_lfeFilterSampleRate = sampleRate;
	
	// Reset filter state
	m_lfeFilterX1 = m_lfeFilterX2 = 0.0;
	m_lfeFilterY1 = m_lfeFilterY2 = 0.0;
	
	if(sampleRate == 0)
		return;
	
	// 2nd-order Butterworth lowpass at 120Hz
	// Using bilinear transform (Cookbook formulae by Robert Bristow-Johnson)
	const double cutoffFreq = 120.0;
	const double omega = 2.0 * 3.14159265358979323846 * cutoffFreq / static_cast<double>(sampleRate);
	const double sinOmega = std::sin(omega);
	const double cosOmega = std::cos(omega);
	const double alpha = sinOmega / (2.0 * 0.7071067811865476);  // Q = 1/sqrt(2) for Butterworth
	
	// Coefficients (normalized)
	const double a0 = 1.0 + alpha;
	m_lfeFilterB0 = ((1.0 - cosOmega) / 2.0) / a0;
	m_lfeFilterB1 = (1.0 - cosOmega) / a0;
	m_lfeFilterB2 = ((1.0 - cosOmega) / 2.0) / a0;
	m_lfeFilterA1 = (-2.0 * cosOmega) / a0;
	m_lfeFilterA2 = (1.0 - alpha) / a0;
}


// Process LFE channel: sum 5 main channels, lowpass at 120Hz, apply -10dB attenuation
void CSoundFile::ProcessLFEChannel(int count)
{
	// Initialize filter if needed
	InitializeLFEFilter();
	
	// LFE gain: -10dB = 10^(-10/20) ≈ 0.3162
	constexpr double lfeGain = 0.31622776601683794;
	
	for(int i = 0; i < count; i++)
	{
		// Sum all 5 main channels
		double sum = static_cast<double>(MixSurroundFL[i]) 
		           + static_cast<double>(MixSurroundFR[i])
		           + static_cast<double>(MixSurroundC[i])
		           + static_cast<double>(MixSurroundSL[i])
		           + static_cast<double>(MixSurroundSR[i]);
		
		// Apply 2nd-order IIR lowpass filter (Direct Form II Transposed)
		// y[n] = b0*x[n] + b1*x[n-1] + b2*x[n-2] - a1*y[n-1] - a2*y[n-2]
		const double output = m_lfeFilterB0 * sum 
		                    + m_lfeFilterB1 * m_lfeFilterX1 
		                    + m_lfeFilterB2 * m_lfeFilterX2
		                    - m_lfeFilterA1 * m_lfeFilterY1
		                    - m_lfeFilterA2 * m_lfeFilterY2;
		
		// Update filter state
		m_lfeFilterX2 = m_lfeFilterX1;
		m_lfeFilterX1 = sum;
		m_lfeFilterY2 = m_lfeFilterY1;
		m_lfeFilterY1 = output;
		
		// Apply -10dB gain and store to LFE buffer
		MixSurroundLFE[i] = static_cast<mixsample_t>(output * lfeGain);
	}
}


void CSoundFile::ProcessPlugins(uint32 nCount)
{
	// If any sample channels are active or any plugin has some input, possibly suspended master plugins need to be woken up.
	bool masterHasInput = (m_nMixStat > 0);

#ifdef MPT_INTMIXER
	const float IntToFloat = m_PlayConfig.getIntToFloat();
	const float FloatToInt = m_PlayConfig.getFloatToInt();
#endif // MPT_INTMIXER

	// Setup float inputs from samples
	for(PLUGINDEX plug = 0; plug < MAX_MIXPLUGINS; plug++)
	{
		SNDMIXPLUGIN &plugin = m_MixPlugins[plug];
		if(plugin.pMixPlugin != nullptr
			&& plugin.pMixPlugin->m_MixState.pMixBuffer != nullptr
			&& plugin.pMixPlugin->m_mixBuffer.Ok())
		{
			IMixPlugin *mixPlug = plugin.pMixPlugin;
			SNDMIXPLUGINSTATE &state = mixPlug->m_MixState;

			//We should only ever reach this point if the song is playing.
			if (!mixPlug->IsSongPlaying())
			{
				//Plugin doesn't know it is in a song that is playing;
				//we must have added it during playback. Initialise it!
				mixPlug->NotifySongPlaying(true);
				mixPlug->Resume();
			}


			// Setup float input
			float *plugInputL = mixPlug->m_mixBuffer.GetInputBuffer(0);
			float *plugInputR = mixPlug->m_mixBuffer.GetInputBuffer(1);
			if (state.dwFlags & SNDMIXPLUGINSTATE::psfMixReady)
			{
#ifdef MPT_INTMIXER
				StereoMixToFloat(state.pMixBuffer, plugInputL, plugInputR, nCount, IntToFloat);
#else
				DeinterleaveStereo(state.pMixBuffer, plugInputL, plugInputR, nCount);
#endif // MPT_INTMIXER
			} else if (state.nVolDecayR || state.nVolDecayL)
			{
				StereoFill(state.pMixBuffer, nCount, state.nVolDecayR, state.nVolDecayL);
#ifdef MPT_INTMIXER
				StereoMixToFloat(state.pMixBuffer, plugInputL, plugInputR, nCount, IntToFloat);
#else
				DeinterleaveStereo(state.pMixBuffer, plugInputL, plugInputR, nCount);
#endif // MPT_INTMIXER
			} else
			{
				memset(plugInputL, 0, nCount * sizeof(plugInputL[0]));
				memset(plugInputR, 0, nCount * sizeof(plugInputR[0]));
			}
			state.dwFlags &= ~SNDMIXPLUGINSTATE::psfMixReady;
			
			if(!plugin.IsMasterEffect() && !(state.dwFlags & SNDMIXPLUGINSTATE::psfSilenceBypass))
			{
				masterHasInput = true;
			}
		}
	}
	// Convert mix buffer
#ifdef MPT_INTMIXER
	StereoMixToFloat(MixSoundBuffer, MixFloatBuffer[0], MixFloatBuffer[1], nCount, IntToFloat);
#else
	DeinterleaveStereo(MixSoundBuffer, MixFloatBuffer[0], MixFloatBuffer[1], nCount);
#endif // MPT_INTMIXER
	float *pMixL = MixFloatBuffer[0];
	float *pMixR = MixFloatBuffer[1];

	const bool positionChanged = HasPositionChanged();

	// Process Plugins
	for(PLUGINDEX plug = 0; plug < MAX_MIXPLUGINS; plug++)
	{
		SNDMIXPLUGIN &plugin = m_MixPlugins[plug];
		if (plugin.pMixPlugin != nullptr
			&& plugin.pMixPlugin->m_MixState.pMixBuffer != nullptr
			&& plugin.pMixPlugin->m_mixBuffer.Ok())
		{
			IMixPlugin *pObject = plugin.pMixPlugin;
			if(!plugin.IsMasterEffect() && !plugin.pMixPlugin->ShouldProcessSilence() && !(plugin.pMixPlugin->m_MixState.dwFlags & SNDMIXPLUGINSTATE::psfHasInput))
			{
				// If plugin has no inputs and isn't a master plugin, we shouldn't let it process silence if possible.
				// I have yet to encounter a VST plugin which actually sets this flag.
				bool hasInput = false;
				for(PLUGINDEX inPlug = 0; inPlug < plug; inPlug++)
				{
					if(m_MixPlugins[inPlug].GetOutputPlugin() == plug)
					{
						hasInput = true;
						break;
					}
				}
				if(!hasInput)
				{
					continue;
				}
			}

			bool isMasterMix = false;
			float *plugInputL = pObject->m_mixBuffer.GetInputBuffer(0);
			float *plugInputR = pObject->m_mixBuffer.GetInputBuffer(1);

			if (pMixL == plugInputL)
			{
				isMasterMix = true;
				pMixL = MixFloatBuffer[0];
				pMixR = MixFloatBuffer[1];
			}
			SNDMIXPLUGINSTATE &state = plugin.pMixPlugin->m_MixState;
			float *pOutL = pMixL;
			float *pOutR = pMixR;

			if (!plugin.IsOutputToMaster())
			{
				PLUGINDEX nOutput = plugin.GetOutputPlugin();
				if(nOutput > plug && nOutput < MAX_MIXPLUGINS
					&& m_MixPlugins[nOutput].pMixPlugin != nullptr)
				{
					IMixPlugin *outPlugin = m_MixPlugins[nOutput].pMixPlugin;
					if(!(state.dwFlags & SNDMIXPLUGINSTATE::psfSilenceBypass)) outPlugin->ResetSilence();

					if(outPlugin->m_mixBuffer.Ok())
					{
						pOutL = outPlugin->m_mixBuffer.GetInputBuffer(0);
						pOutR = outPlugin->m_mixBuffer.GetInputBuffer(1);
					}
				}
			}

			/*
			if (plugin.multiRouting) {
				int nOutput=0;
				for (int nOutput=0; nOutput < plugin.nOutputs / 2; nOutput++) {
					destinationPlug = plugin.multiRoutingDestinations[nOutput];
					pOutState = m_MixPlugins[destinationPlug].pMixState;
					pOutputs[2 * nOutput] = plugInputL;
					pOutputs[2 * (nOutput + 1)] = plugInputR;
				}

			}*/

			if (plugin.IsMasterEffect())
			{
				if (!isMasterMix)
				{
					float *pInL = plugInputL;
					float *pInR = plugInputR;
					for (uint32 i=0; i<nCount; i++)
					{
						pInL[i] += pMixL[i];
						pInR[i] += pMixR[i];
						pMixL[i] = 0;
						pMixR[i] = 0;
					}
				}
				pMixL = pOutL;
				pMixR = pOutR;

				if(masterHasInput)
				{
					// Samples or plugins are being rendered, so turn off auto-bypass for this master effect.
					if(plugin.pMixPlugin != nullptr) plugin.pMixPlugin->ResetSilence();
					SNDMIXPLUGIN *chain = &plugin;
					PLUGINDEX out = chain->GetOutputPlugin(), prevOut = plug;
					while(out > prevOut && out < MAX_MIXPLUGINS)
					{
						chain = &m_MixPlugins[out];
						prevOut = out;
						out = chain->GetOutputPlugin();
						if(chain->pMixPlugin)
						{
							chain->pMixPlugin->ResetSilence();
						}
					}
				}
			}

			if(plugin.IsBypassed() || (plugin.IsAutoSuspendable() && (state.dwFlags & SNDMIXPLUGINSTATE::psfSilenceBypass)))
			{
				const float * const pInL = plugInputL;
				const float * const pInR = plugInputR;
				for (uint32 i=0; i<nCount; i++)
				{
					pOutL[i] += pInL[i];
					pOutR[i] += pInR[i];
				}
			} else
			{
				if(positionChanged)
					pObject->PositionChanged();
				pObject->Process(pOutL, pOutR, nCount);

				state.inputSilenceCount += nCount;
				if(plugin.IsAutoSuspendable() && pObject->GetNumOutputChannels() > 0 && state.inputSilenceCount >= m_MixerSettings.gdwMixingFreq * 4)
				{
					bool isSilent = true;
					for(uint32 i = 0; i < nCount; i++)
					{
						if(pOutL[i] >= FLT_EPSILON || pOutL[i] <= -FLT_EPSILON
							|| pOutR[i] >= FLT_EPSILON || pOutR[i] <= -FLT_EPSILON)
						{
							isSilent = false;
							break;
						}
					}
					if(isSilent)
					{
						state.dwFlags |= SNDMIXPLUGINSTATE::psfSilenceBypass;
					} else
					{
						state.inputSilenceCount = 0;
					}
				}
			}
			state.dwFlags &= ~SNDMIXPLUGINSTATE::psfHasInput;
		}
	}
#ifdef MPT_INTMIXER
	FloatToStereoMix(pMixL, pMixR, MixSoundBuffer, nCount, FloatToInt);
#else
	InterleaveStereo(pMixL, pMixR, MixSoundBuffer, nCount);
#endif // MPT_INTMIXER
}


OPENMPT_NAMESPACE_END
