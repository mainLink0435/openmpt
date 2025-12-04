/*
 * IntMixer.h
 * ----------
 * Purpose: Fixed point mixer classes
 * Notes  : (currently none)
 * Authors: Olivier Lapicque
 *          OpenMPT Devs
 * The OpenMPT source code is released under the BSD license. Read LICENSE for more details.
 */


#pragma once

#include "openmpt/all/BuildSettings.hpp"

#include "Resampler.h"
#include "MixerInterface.h"
#include "Paula.h"

OPENMPT_NAMESPACE_BEGIN

template<int channelsOut, int channelsIn, typename out, typename in, size_t mixPrecision>
struct IntToIntTraits : public MixerTraits<channelsOut, channelsIn, out, in>
{
	using base_t = MixerTraits<channelsOut, channelsIn, out, in>;
	using input_t = typename base_t::input_t;
	using output_t = typename base_t::output_t;

	MPT_ATTR_ALWAYSINLINE MPT_INLINE_FORCE constexpr static output_t Convert(const input_t x)
	{
		static_assert(std::numeric_limits<input_t>::is_integer, "Input must be integer");
		static_assert(std::numeric_limits<output_t>::is_integer, "Output must be integer");
		static_assert(sizeof(out) * 8 >= mixPrecision, "Mix precision is higher than output type can handle");
		static_assert(sizeof(in) * 8 <= mixPrecision, "Mix precision is lower than input type");
		return static_cast<output_t>(x) * (1<<(mixPrecision - sizeof(in) * 8));
	}
};

using Int8MToIntS = IntToIntTraits<2, 1, mixsample_t, int8,  16>;
using Int16MToIntS = IntToIntTraits<2, 1, mixsample_t, int16, 16>;
using Int8SToIntS = IntToIntTraits<2, 2, mixsample_t, int8,  16>;
using Int16SToIntS = IntToIntTraits<2, 2, mixsample_t, int16, 16>;

// 5-channel surround output traits
using Int8MToInt5 = IntToIntTraits<5, 1, mixsample_t, int8,  16>;
using Int16MToInt5 = IntToIntTraits<5, 1, mixsample_t, int16, 16>;
using Int8SToInt5 = IntToIntTraits<5, 2, mixsample_t, int8,  16>;
using Int16SToInt5 = IntToIntTraits<5, 2, mixsample_t, int16, 16>;


//////////////////////////////////////////////////////////////////////////
// Interpolation templates


template<class Traits>
struct AmigaBlepInterpolation
{
	SamplePosition subIncrement;
	Paula::State &paula;
	const Paula::BlepArray &WinSincIntegral;
	const int numSteps;
	unsigned int remainingSamples = 0;

	MPT_ATTR_ALWAYSINLINE MPT_INLINE_FORCE AmigaBlepInterpolation(ModChannel &chn, const CResampler &resampler, unsigned int numSamples)
		: paula{chn.paulaState}
		, WinSincIntegral{resampler.blepTables.GetAmigaTable(resampler.m_Settings.emulateAmiga, chn.dwFlags[CHN_AMIGAFILTER])}
		, numSteps{chn.paulaState.numSteps}
	{
		if(numSteps)
		{
			subIncrement = chn.increment / numSteps;
			// May we read past the start or end of sample if we do partial sample increments?
			// If that's the case, don't apply any sub increments on the source sample if we reached the last output sample
			// Note that this should only happen with notes well outside the Amiga note range, e.g. in software-mixed formats like MED
			const int32 targetPos = (chn.position + chn.increment * numSamples).GetInt();
			if(static_cast<SmpLength>(targetPos) > chn.nLength)
				remainingSamples = numSamples;
		}
		
	}

	MPT_ATTR_ALWAYSINLINE MPT_INLINE_FORCE void operator() (typename Traits::outbuf_t &outSample, const typename Traits::input_t * const MPT_RESTRICT inBuffer, const uint32 posLo)
	{
		if(--remainingSamples == 0)
			subIncrement = {};

		SamplePosition pos(0, posLo);
		// First, process steps of full length (one Amiga clock interval)
		for(int step = numSteps; step > 0; step--)
		{
			typename Traits::output_t inSample = 0;
			int32 posInt = pos.GetInt() * Traits::numChannelsIn;
			for(int32 i = 0; i < Traits::numChannelsIn; i++)
				inSample += Traits::Convert(inBuffer[posInt + i]);
			paula.InputSample(static_cast<int16>(inSample / (4 * Traits::numChannelsIn)));
			paula.Clock(Paula::MINIMUM_INTERVAL);
			pos += subIncrement;
		}
		paula.remainder += paula.stepRemainder;

		// Now, process any remaining integer clock amount < MINIMUM_INTERVAL
		uint32 remainClocks = paula.remainder.GetInt();
		if(remainClocks)
		{
			typename Traits::output_t inSample = 0;
			int32 posInt = pos.GetInt() * Traits::numChannelsIn;
			for(int32 i = 0; i < Traits::numChannelsIn; i++)
				inSample += Traits::Convert(inBuffer[posInt + i]);
			paula.InputSample(static_cast<int16>(inSample / (4 * Traits::numChannelsIn)));
			paula.Clock(remainClocks);
			paula.remainder.RemoveInt();
		}

		auto out = paula.OutputSample(WinSincIntegral);
		for(int i = 0; i < Traits::numChannelsOut; i++)
			outSample[i] = out;
	}
};


template<class Traits>
struct LinearInterpolation
{
	MPT_ATTR_ALWAYSINLINE MPT_INLINE_FORCE LinearInterpolation(const ModChannel &, const CResampler &, unsigned int) { }

	MPT_ATTR_ALWAYSINLINE MPT_INLINE_FORCE void operator() (typename Traits::outbuf_t &outSample, const typename Traits::input_t * const MPT_RESTRICT inBuffer, const uint32 posLo)
	{
		static_assert(static_cast<int>(Traits::numChannelsIn) <= static_cast<int>(Traits::numChannelsOut), "Too many input channels");
		const typename Traits::output_t fract = posLo >> 18u;

		for(int i = 0; i < Traits::numChannelsIn; i++)
		{
			typename Traits::output_t srcVol = Traits::Convert(inBuffer[i]);
			typename Traits::output_t destVol = Traits::Convert(inBuffer[i + Traits::numChannelsIn]);

			outSample[i] = srcVol + ((fract * (destVol - srcVol)) / 16384);
		}
	}
};


template<class Traits>
struct FastSincInterpolation
{
	MPT_ATTR_ALWAYSINLINE MPT_INLINE_FORCE FastSincInterpolation(const ModChannel &, const CResampler &, unsigned int) { }

	MPT_ATTR_ALWAYSINLINE MPT_INLINE_FORCE void operator() (typename Traits::outbuf_t &outSample, const typename Traits::input_t * const MPT_RESTRICT inBuffer, const uint32 posLo)
	{
		static_assert(static_cast<int>(Traits::numChannelsIn) <= static_cast<int>(Traits::numChannelsOut), "Too many input channels");
		const int16 *lut = CResampler::FastSincTable + ((posLo >> 22) & 0x3FC);

		for(int i = 0; i < Traits::numChannelsIn; i++)
		{
			outSample[i] =
				 (lut[0] * Traits::Convert(inBuffer[i - Traits::numChannelsIn])
				+ lut[1] * Traits::Convert(inBuffer[i])
				+ lut[2] * Traits::Convert(inBuffer[i + Traits::numChannelsIn])
				+ lut[3] * Traits::Convert(inBuffer[i + 2 * Traits::numChannelsIn])) / 16384;
		}
	}
};


template<class Traits>
struct PolyphaseInterpolation
{
	const SINC_TYPE *sinc;

	MPT_ATTR_ALWAYSINLINE MPT_INLINE_FORCE PolyphaseInterpolation(const ModChannel &chn, const CResampler &resampler, unsigned int)
	{
		#ifdef MODPLUG_TRACKER
			// Otherwise causes "warning C4100: 'resampler' : unreferenced formal parameter"
			// because all 3 tables are static members.
			// #pragma warning fails with this templated case for unknown reasons.
			MPT_UNREFERENCED_PARAMETER(resampler);
		#endif // MODPLUG_TRACKER
		sinc = (((chn.increment > SamplePosition(0x130000000ll)) || (chn.increment < SamplePosition(-0x130000000ll))) ?
			(((chn.increment > SamplePosition(0x180000000ll)) || (chn.increment < SamplePosition(-0x180000000ll))) ? resampler.gDownsample2x : resampler.gDownsample13x) : resampler.gKaiserSinc);
	}

	MPT_ATTR_ALWAYSINLINE MPT_INLINE_FORCE void operator() (typename Traits::outbuf_t &outSample, const typename Traits::input_t * const MPT_RESTRICT inBuffer, const uint32 posLo)
	{
		static_assert(static_cast<int>(Traits::numChannelsIn) <= static_cast<int>(Traits::numChannelsOut), "Too many input channels");
		const SINC_TYPE *lut = sinc + ((posLo >> (32 - SINC_PHASES_BITS)) & SINC_MASK) * SINC_WIDTH;

		for(int i = 0; i < Traits::numChannelsIn; i++)
		{
			outSample[i] =
				 (lut[0] * Traits::Convert(inBuffer[i - 3 * Traits::numChannelsIn])
				+ lut[1] * Traits::Convert(inBuffer[i - 2 * Traits::numChannelsIn])
				+ lut[2] * Traits::Convert(inBuffer[i - Traits::numChannelsIn])
				+ lut[3] * Traits::Convert(inBuffer[i])
				+ lut[4] * Traits::Convert(inBuffer[i + Traits::numChannelsIn])
				+ lut[5] * Traits::Convert(inBuffer[i + 2 * Traits::numChannelsIn])
				+ lut[6] * Traits::Convert(inBuffer[i + 3 * Traits::numChannelsIn])
				+ lut[7] * Traits::Convert(inBuffer[i + 4 * Traits::numChannelsIn])) / (1 << SINC_QUANTSHIFT);
		}
	}
};


template<class Traits>
struct FIRFilterInterpolation
{
	const int16 *WFIRlut;

	MPT_ATTR_ALWAYSINLINE MPT_INLINE_FORCE FIRFilterInterpolation(const ModChannel &, const CResampler &resampler, unsigned int)
	{
		WFIRlut = resampler.m_WindowedFIR.lut;
	}

	MPT_ATTR_ALWAYSINLINE MPT_INLINE_FORCE void operator() (typename Traits::outbuf_t &outSample, const typename Traits::input_t * const MPT_RESTRICT inBuffer, const uint32 posLo)
	{
		static_assert(static_cast<int>(Traits::numChannelsIn) <= static_cast<int>(Traits::numChannelsOut), "Too many input channels");
		const int16 * const lut = WFIRlut + ((((posLo >> 16) + WFIR_FRACHALVE) >> WFIR_FRACSHIFT) & WFIR_FRACMASK);

		for(int i = 0; i < Traits::numChannelsIn; i++)
		{
			typename Traits::output_t vol1 =
				  (lut[0] * Traits::Convert(inBuffer[i - 3 * Traits::numChannelsIn]))
				+ (lut[1] * Traits::Convert(inBuffer[i - 2 * Traits::numChannelsIn]))
				+ (lut[2] * Traits::Convert(inBuffer[i - Traits::numChannelsIn]))
				+ (lut[3] * Traits::Convert(inBuffer[i]));
			typename Traits::output_t vol2 =
				  (lut[4] * Traits::Convert(inBuffer[i + 1 * Traits::numChannelsIn]))
				+ (lut[5] * Traits::Convert(inBuffer[i + 2 * Traits::numChannelsIn]))
				+ (lut[6] * Traits::Convert(inBuffer[i + 3 * Traits::numChannelsIn]))
				+ (lut[7] * Traits::Convert(inBuffer[i + 4 * Traits::numChannelsIn]));
			outSample[i] = ((vol1 / 2) + (vol2 / 2)) / (1 << (WFIR_16BITSHIFT - 1));
		}
	}
};


//////////////////////////////////////////////////////////////////////////
// Mixing templates (add sample to stereo mix)

template<class Traits>
struct NoRamp
{
	typename Traits::output_t lVol, rVol;

	MPT_ATTR_ALWAYSINLINE MPT_INLINE_FORCE NoRamp(const ModChannel &chn)
	{
		lVol = chn.leftVol;
		rVol = chn.rightVol;
	}
};


struct Ramp
{
	ModChannel &channel;
	int32 lRamp, rRamp;

	MPT_ATTR_ALWAYSINLINE MPT_INLINE_FORCE Ramp(ModChannel &chn)
		: channel{chn}
	{
		lRamp = chn.rampLeftVol;
		rRamp = chn.rampRightVol;
	}

	MPT_ATTR_ALWAYSINLINE MPT_INLINE_FORCE ~Ramp()
	{
		channel.rampLeftVol = lRamp; channel.leftVol = lRamp >> VOLUMERAMPPRECISION;
		channel.rampRightVol = rRamp; channel.rightVol = rRamp >> VOLUMERAMPPRECISION;
	}
};


// Legacy optimization: If chn.nLeftVol == chn.nRightVol, save one multiplication instruction
template<class Traits>
struct MixMonoFastNoRamp : public NoRamp<Traits>
{
	using base_t = NoRamp<Traits>;
	MPT_ATTR_ALWAYSINLINE MPT_INLINE_FORCE void operator() (const typename Traits::outbuf_t &outSample, const ModChannel &, typename Traits::output_t * const MPT_RESTRICT outBuffer)
	{
		typename Traits::output_t vol = outSample[0] * base_t::lVol;
		for(int i = 0; i < Traits::numChannelsOut; i++)
		{
			outBuffer[i] += vol;
		}
	}
};


template<class Traits>
struct MixMonoNoRamp : public NoRamp<Traits>
{
	using base_t = NoRamp<Traits>;
	MPT_ATTR_ALWAYSINLINE MPT_INLINE_FORCE void operator() (const typename Traits::outbuf_t &outSample, const ModChannel &, typename Traits::output_t * const MPT_RESTRICT outBuffer)
	{
		outBuffer[0] += outSample[0] * base_t::lVol;
		outBuffer[1] += outSample[0] * base_t::rVol;
	}
};


template<class Traits>
struct MixMonoRamp : public Ramp
{
	MPT_ATTR_ALWAYSINLINE MPT_INLINE_FORCE void operator() (const typename Traits::outbuf_t &outSample, const ModChannel &chn, typename Traits::output_t * const MPT_RESTRICT outBuffer)
	{
		lRamp += chn.leftRamp;
		rRamp += chn.rightRamp;
		outBuffer[0] += outSample[0] * (lRamp >> VOLUMERAMPPRECISION);
		outBuffer[1] += outSample[0] * (rRamp >> VOLUMERAMPPRECISION);
	}
};


template<class Traits>
struct MixStereoNoRamp : public NoRamp<Traits>
{
	using base_t = NoRamp<Traits>;
	MPT_ATTR_ALWAYSINLINE MPT_INLINE_FORCE void operator() (const typename Traits::outbuf_t &outSample, const ModChannel &, typename Traits::output_t * const MPT_RESTRICT outBuffer)
	{
		outBuffer[0] += outSample[0] * base_t::lVol;
		outBuffer[1] += outSample[1] * base_t::rVol;
	}
};


template<class Traits>
struct MixStereoRamp : public Ramp
{
	MPT_ATTR_ALWAYSINLINE MPT_INLINE_FORCE void operator() (const typename Traits::outbuf_t &outSample, const ModChannel &chn, typename Traits::output_t * const MPT_RESTRICT outBuffer)
	{
		lRamp += chn.leftRamp;
		rRamp += chn.rightRamp;
		outBuffer[0] += outSample[0] * (lRamp >> VOLUMERAMPPRECISION);
		outBuffer[1] += outSample[1] * (rRamp >> VOLUMERAMPPRECISION);
	}
};


//////////////////////////////////////////////////////////////////////////
// 5-channel surround mixing templates

// Surround mix: apply 5 separate gains to mono input, output to 5 speakers
template<class Traits>
struct MixSurroundNoRamp
{
	ModChannel &channel;
	int32 gainSL, gainFL, gainC, gainFR, gainSR;
	uint32 attackRemain;
	uint32 attackLen;

	MPT_ATTR_ALWAYSINLINE MPT_INLINE_FORCE MixSurroundNoRamp(ModChannel &chn)
		: channel{chn}
	{
		// Gains are pre-calculated and stored in channel (0-256 range)
		gainSL = chn.newSurroundSL;
		gainFL = chn.newSurroundFL;
		gainC = chn.newSurroundC;
		gainFR = chn.newSurroundFR;
		gainSR = chn.newSurroundSR;
		attackRemain = chn.attackRampRemaining;
		attackLen = chn.attackRampLength;
	}

	MPT_ATTR_ALWAYSINLINE MPT_INLINE_FORCE ~MixSurroundNoRamp()
	{
		channel.attackRampRemaining = attackRemain;
	}

	MPT_ATTR_ALWAYSINLINE MPT_INLINE_FORCE void operator() (const typename Traits::outbuf_t &outSample, const ModChannel &chn, typename Traits::output_t * const MPT_RESTRICT outBuffer)
	{
		// Input: mono or stereo sample - fold to mono first
		typename Traits::output_t mono = outSample[0];
		if(Traits::numChannelsIn == 2)
		{
			mono = (outSample[0] + outSample[1]) / 2;
		}

		// Apply short attack fade if active (fade is 0-256)
		if(attackRemain > 0 && attackLen > 0)
		{
			const uint32 progressed = attackLen - attackRemain;
			const int32 fade = static_cast<int32>(std::min<uint32>(attackLen, progressed + 1) * 256u / attackLen);
			mono = (mono * fade) >> 8;
			attackRemain--;
		}

		// Apply gains directly like stereo mixer does (gains are absolute volumes from ReadNote)
		outBuffer[0] += mono * gainSL;
		outBuffer[1] += mono * gainFL;
		outBuffer[2] += mono * gainC;
		outBuffer[3] += mono * gainFR;
		outBuffer[4] += mono * gainSR;
	}
};

// Surround mix with volume ramping
template<class Traits>
struct MixSurroundRamp
{
	ModChannel &channel;
	int32 lRamp, rRamp;
	int32 slRamp, flRamp, cRamp, frRamp, srRamp;
	int32 slVol, flVol, cVol, frVol, srVol;
	uint32 attackRemain;
	uint32 attackLen;

	MPT_ATTR_ALWAYSINLINE MPT_INLINE_FORCE MixSurroundRamp(ModChannel &chn)
		: channel{chn}
	{
		lRamp = chn.rampLeftVol;
		rRamp = chn.rampRightVol;
		slRamp = chn.rampSurroundSL;
		flRamp = chn.rampSurroundFL;
		cRamp = chn.rampSurroundC;
		frRamp = chn.rampSurroundFR;
		srRamp = chn.rampSurroundSR;
		slVol = chn.rampVolSurroundSL;
		flVol = chn.rampVolSurroundFL;
		cVol = chn.rampVolSurroundC;
		frVol = chn.rampVolSurroundFR;
		srVol = chn.rampVolSurroundSR;
		attackRemain = chn.attackRampRemaining;
		attackLen = chn.attackRampLength;
	}

	MPT_ATTR_ALWAYSINLINE MPT_INLINE_FORCE ~MixSurroundRamp()
	{
		channel.rampLeftVol = lRamp; channel.leftVol = lRamp >> VOLUMERAMPPRECISION;
		channel.rampRightVol = rRamp; channel.rightVol = rRamp >> VOLUMERAMPPRECISION;
		channel.rampSurroundSL = slRamp; channel.volSurroundSL = slVol >> VOLUMERAMPPRECISION;
		channel.rampSurroundFL = flRamp; channel.volSurroundFL = flVol >> VOLUMERAMPPRECISION;
		channel.rampSurroundC = cRamp; channel.volSurroundC = cVol >> VOLUMERAMPPRECISION;
		channel.rampSurroundFR = frRamp; channel.volSurroundFR = frVol >> VOLUMERAMPPRECISION;
		channel.rampSurroundSR = srRamp; channel.volSurroundSR = srVol >> VOLUMERAMPPRECISION;
		channel.rampVolSurroundSL = slVol;
		channel.rampVolSurroundFL = flVol;
		channel.rampVolSurroundC = cVol;
		channel.rampVolSurroundFR = frVol;
		channel.rampVolSurroundSR = srVol;
		channel.attackRampRemaining = attackRemain;
	}

	MPT_ATTR_ALWAYSINLINE MPT_INLINE_FORCE void operator() (const typename Traits::outbuf_t &outSample, const ModChannel &chn, typename Traits::output_t * const MPT_RESTRICT outBuffer)
	{
		lRamp += chn.leftRamp;
		rRamp += chn.rightRamp;
		slVol += slRamp;
		flVol += flRamp;
		cVol += cRamp;
		frVol += frRamp;
		srVol += srRamp;
		
		// Input: mono or stereo sample - fold to mono first
		typename Traits::output_t mono = outSample[0];
		if(Traits::numChannelsIn == 2)
		{
			mono = (outSample[0] + outSample[1]) / 2;
		}

		// Apply short attack fade if active
		if(attackRemain > 0 && attackLen > 0)
		{
			const uint32 progressed = attackLen - attackRemain;
			const int32 fade = static_cast<int32>(std::min<uint32>(attackLen, progressed + 1) * 256u / attackLen);
			mono = (mono * fade) >> 8;
			attackRemain--;
		}

		// Get ramped surround gains (20.12 fixed point -> regular int)
		// These smoothly transition from old to new values per-sample
		const int32 gainSL = slVol >> VOLUMERAMPPRECISION;
		const int32 gainFL = flVol >> VOLUMERAMPPRECISION;
		const int32 gainC = cVol >> VOLUMERAMPPRECISION;
		const int32 gainFR = frVol >> VOLUMERAMPPRECISION;
		const int32 gainSR = srVol >> VOLUMERAMPPRECISION;

		// Apply ramped gains directly (same as stereo ramp mixer does)
		outBuffer[0] += mono * gainSL;
		outBuffer[1] += mono * gainFL;
		outBuffer[2] += mono * gainC;
		outBuffer[3] += mono * gainFR;
		outBuffer[4] += mono * gainSR;
	}
};


//////////////////////////////////////////////////////////////////////////
// Filter templates


template<class Traits>
struct NoFilter
{
	MPT_ATTR_ALWAYSINLINE MPT_INLINE_FORCE NoFilter(const ModChannel &) { }

	MPT_ATTR_ALWAYSINLINE MPT_INLINE_FORCE void operator() (const typename Traits::outbuf_t &, const ModChannel &) { }
};


// Resonant filter
template<class Traits>
struct ResonantFilter
{
	ModChannel &channel;
	// Filter history
	typename Traits::output_t fy[Traits::numChannelsIn][2];

	MPT_ATTR_ALWAYSINLINE MPT_INLINE_FORCE ResonantFilter(ModChannel &chn)
		: channel{chn}
	{
		for(int i = 0; i < Traits::numChannelsIn; i++)
		{
			fy[i][0] = chn.nFilter_Y[i][0];
			fy[i][1] = chn.nFilter_Y[i][1];
		}
	}

	MPT_ATTR_ALWAYSINLINE MPT_INLINE_FORCE ~ResonantFilter()
	{
		for(int i = 0; i < Traits::numChannelsIn; i++)
		{
			channel.nFilter_Y[i][0] = fy[i][0];
			channel.nFilter_Y[i][1] = fy[i][1];
		}
	}

	// To avoid a precision loss in the state variables especially with quiet samples at low cutoff and high mix rate, we pre-amplify the sample.
#define MIXING_FILTER_PREAMP 256
	// Filter values are clipped to double the input range
#define ClipFilter(x) Clamp<typename Traits::output_t, typename Traits::output_t>(x, int16_min * 2 * MIXING_FILTER_PREAMP, int16_max * 2 * MIXING_FILTER_PREAMP)

	MPT_ATTR_ALWAYSINLINE MPT_INLINE_FORCE void operator() (typename Traits::outbuf_t &outSample, const ModChannel &chn)
	{
		static_assert(static_cast<int>(Traits::numChannelsIn) <= static_cast<int>(Traits::numChannelsOut), "Too many input channels");

		for(int i = 0; i < Traits::numChannelsIn; i++)
		{
			const auto inputAmp = outSample[i] * MIXING_FILTER_PREAMP;
			typename Traits::output_t val = static_cast<typename Traits::output_t>(mpt::rshift_signed(
				Util::mul32to64(inputAmp, chn.nFilter_A0) +
				Util::mul32to64(ClipFilter(fy[i][0]), chn.nFilter_B0) +
				Util::mul32to64(ClipFilter(fy[i][1]), chn.nFilter_B1) +
				(1 << (MIXING_FILTER_PRECISION - 1)), MIXING_FILTER_PRECISION));
			fy[i][1] = fy[i][0];
			fy[i][0] = val - (inputAmp & chn.nFilter_HP);
			outSample[i] = val / MIXING_FILTER_PREAMP;
		}
	}

#undef ClipFilter
};


OPENMPT_NAMESPACE_END
