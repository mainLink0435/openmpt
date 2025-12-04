/*
 * FloatMixer.h
 * ------------
 * Purpose: Floating point mixer classes
 * Notes  : (currently none)
 * Authors: OpenMPT Devs
 * The OpenMPT source code is released under the BSD license. Read LICENSE for more details.
 */


#pragma once

#include "openmpt/all/BuildSettings.hpp"

#include "MixerInterface.h"
#include "Resampler.h"

OPENMPT_NAMESPACE_BEGIN

template<int channelsOut, int channelsIn, typename out, typename in, int int2float>
struct IntToFloatTraits : public MixerTraits<channelsOut, channelsIn, out, in>
{
	static_assert(std::numeric_limits<input_t>::is_integer, "Input must be integer");
	static_assert(!std::numeric_limits<output_t>::is_integer, "Output must be floating point");

	MPT_ATTR_ALWAYSINLINE MPT_INLINE_FORCE constexpr static output_t Convert(const input_t x)
	{
		return static_cast<output_t>(x) * (static_cast<output_t>(1) / static_cast<output_t>(int2float));
	}
};

using Int8MToFloatS = IntToFloatTraits<2, 1, mixsample_t, int8,  -int8_min>;
using Int16MToFloatS = IntToFloatTraits<2, 1, mixsample_t, int16, -int16_min>;
using Int8SToFloatS = IntToFloatTraits<2, 2, mixsample_t, int8,  -int8_min>;
using Int16SToFloatS  = IntToFloatTraits<2, 2, mixsample_t, int16, -int16_min>;

// 5-channel surround output traits
using Int8MToFloat5 = IntToFloatTraits<5, 1, mixsample_t, int8,  -int8_min>;
using Int16MToFloat5 = IntToFloatTraits<5, 1, mixsample_t, int16, -int16_min>;
using Int8SToFloat5 = IntToFloatTraits<5, 2, mixsample_t, int8,  -int8_min>;
using Int16SToFloat5  = IntToFloatTraits<5, 2, mixsample_t, int16, -int16_min>;


//////////////////////////////////////////////////////////////////////////
// Interpolation templates

template<class Traits>
struct LinearInterpolation
{
	MPT_ATTR_ALWAYSINLINE MPT_INLINE_FORCE LinearInterpolation(const ModChannel &, const CResampler &, unsigned int) { }

	MPT_ATTR_ALWAYSINLINE MPT_INLINE_FORCE void operator() (typename Traits::outbuf_t &outSample, const typename Traits::input_t * const inBuffer, const uint32 posLo)
	{
		static_assert(static_cast<int>(Traits::numChannelsIn) <= static_cast<int>(Traits::numChannelsOut), "Too many input channels");
		const typename Traits::output_t fract = posLo / static_cast<typename Traits::output_t>(0x100000000); //CResampler::LinearTablef[posLo >> 24];

		for(int i = 0; i < Traits::numChannelsIn; i++)
		{
			typename Traits::output_t srcVol = Traits::Convert(inBuffer[i]);
			typename Traits::output_t destVol = Traits::Convert(inBuffer[i + Traits::numChannelsIn]);

			outSample[i] = srcVol + fract * (destVol - srcVol);
		}
	}
};


template<class Traits>
struct FastSincInterpolation
{
	MPT_ATTR_ALWAYSINLINE MPT_INLINE_FORCE FastSincInterpolation(const ModChannel &, const CResampler &, unsigned int) { }

	MPT_ATTR_ALWAYSINLINE MPT_INLINE_FORCE void operator() (typename Traits::outbuf_t &outSample, const typename Traits::input_t * const inBuffer, const uint32 posLo)
	{
		static_assert(static_cast<int>(Traits::numChannelsIn) <= static_cast<int>(Traits::numChannelsOut), "Too many input channels");
		const typename Traits::output_t *lut = CResampler::FastSincTablef + ((posLo >> 22) & 0x3FC);

		for(int i = 0; i < Traits::numChannelsIn; i++)
		{
			outSample[i] =
				  lut[0] * Traits::Convert(inBuffer[i - Traits::numChannelsIn])
				+ lut[1] * Traits::Convert(inBuffer[i])
				+ lut[2] * Traits::Convert(inBuffer[i + Traits::numChannelsIn])
				+ lut[3] * Traits::Convert(inBuffer[i + 2 * Traits::numChannelsIn]);
		}
	}
};


template<class Traits>
struct PolyphaseInterpolation
{
	const typename Traits::output_t *sinc;

	MPT_ATTR_ALWAYSINLINE MPT_INLINE_FORCE PolyphaseInterpolation(const ModChannel &chn, const CResampler &resampler, unsigned int)
	{
		sinc = (((chn.increment > SamplePosition(0x130000000ll)) || (chn.increment < -SamplePosition(-0x130000000ll))) ?
			(((chn.increment > SamplePosition(0x180000000ll)) || (chn.increment < SamplePosition(-0x180000000ll))) ? resampler.gDownsample2x : resampler.gDownsample13x) : resampler.gKaiserSinc);
	}

	MPT_ATTR_ALWAYSINLINE MPT_INLINE_FORCE void operator() (typename Traits::outbuf_t &outSample, const typename Traits::input_t * const inBuffer, const uint32 posLo)
	{
		static_assert(static_cast<int>(Traits::numChannelsIn) <= static_cast<int>(Traits::numChannelsOut), "Too many input channels");
		const typename Traits::output_t *lut = sinc + ((posLo >> (32 - SINC_PHASES_BITS)) & SINC_MASK) * SINC_WIDTH;

		for(int i = 0; i < Traits::numChannelsIn; i++)
		{
			outSample[i] =
				  lut[0] * Traits::Convert(inBuffer[i - 3 * Traits::numChannelsIn])
				+ lut[1] * Traits::Convert(inBuffer[i - 2 * Traits::numChannelsIn])
				+ lut[2] * Traits::Convert(inBuffer[i - Traits::numChannelsIn])
				+ lut[3] * Traits::Convert(inBuffer[i])
				+ lut[4] * Traits::Convert(inBuffer[i + Traits::numChannelsIn])
				+ lut[5] * Traits::Convert(inBuffer[i + 2 * Traits::numChannelsIn])
				+ lut[6] * Traits::Convert(inBuffer[i + 3 * Traits::numChannelsIn])
				+ lut[7] * Traits::Convert(inBuffer[i + 4 * Traits::numChannelsIn]);
		}
	}
};


template<class Traits>
struct FIRFilterInterpolation
{
	const typename Traits::output_t *WFIRlut;

	MPT_ATTR_ALWAYSINLINE MPT_INLINE_FORCE FIRFilterInterpolation(const ModChannel &, const CResampler &resampler, unsigned int)
	{
		WFIRlut = resampler.m_WindowedFIR.lut;
	}

	MPT_ATTR_ALWAYSINLINE MPT_INLINE_FORCE void operator() (typename Traits::outbuf_t &outSample, const typename Traits::input_t * const inBuffer, const uint32 posLo)
	{
		static_assert(static_cast<int>(Traits::numChannelsIn) <= static_cast<int>(Traits::numChannelsOut), "Too many input channels");
		const typename Traits::output_t * const lut = WFIRlut + ((((posLo >> 16) + WFIR_FRACHALVE) >> WFIR_FRACSHIFT) & WFIR_FRACMASK);

		for(int i = 0; i < Traits::numChannelsIn; i++)
		{
			outSample[i] =
				  lut[0] * Traits::Convert(inBuffer[i - 3 * Traits::numChannelsIn])
				+ lut[1] * Traits::Convert(inBuffer[i - 2 * Traits::numChannelsIn])
				+ lut[2] * Traits::Convert(inBuffer[i - Traits::numChannelsIn])
				+ lut[3] * Traits::Convert(inBuffer[i])
				+ lut[4] * Traits::Convert(inBuffer[i + Traits::numChannelsIn])
				+ lut[5] * Traits::Convert(inBuffer[i + 2 * Traits::numChannelsIn])
				+ lut[6] * Traits::Convert(inBuffer[i + 3 * Traits::numChannelsIn])
				+ lut[7] * Traits::Convert(inBuffer[i + 4 * Traits::numChannelsIn]);
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
		lVol = static_cast<Traits::output_t>(chn.leftVol) * (1.0f / 4096.0f);
		rVol = static_cast<Traits::output_t>(chn.rightVol) * (1.0f / 4096.0f);
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
	MPT_ATTR_ALWAYSINLINE MPT_INLINE_FORCE void operator() (const typename Traits::outbuf_t &outSample, const ModChannel &chn, typename Traits::output_t * const outBuffer)
	{
		typename Traits::output_t vol = outSample[0] * lVol;
		for(int i = 0; i < Traits::numChannelsOut; i++)
		{
			outBuffer[i] += vol;
		}
	}
};


template<class Traits>
struct MixMonoNoRamp : public NoRamp<Traits>
{
	MPT_ATTR_ALWAYSINLINE MPT_INLINE_FORCE void operator() (const typename Traits::outbuf_t &outSample, const ModChannel &, typename Traits::output_t * const outBuffer)
	{
		outBuffer[0] += outSample[0] * lVol;
		outBuffer[1] += outSample[0] * rVol;
	}
};


template<class Traits>
struct MixMonoRamp : public Ramp
{
	MPT_ATTR_ALWAYSINLINE MPT_INLINE_FORCE void operator() (const typename Traits::outbuf_t &outSample, const ModChannel &chn, typename Traits::output_t * const outBuffer)
	{
		// TODO volume is not float, can we optimize this?
		lRamp += chn.leftRamp;
		rRamp += chn.rightRamp;
		outBuffer[0] += outSample[0] * (lRamp >> VOLUMERAMPPRECISION) * (1.0f / 4096.0f);
		outBuffer[1] += outSample[0] * (rRamp >> VOLUMERAMPPRECISION) * (1.0f / 4096.0f);
	}
};


template<class Traits>
struct MixStereoNoRamp : public NoRamp<Traits>
{
	MPT_ATTR_ALWAYSINLINE MPT_INLINE_FORCE void operator() (const typename Traits::outbuf_t &outSample, const ModChannel &, typename Traits::output_t * const outBuffer)
	{
		outBuffer[0] += outSample[0] * lVol;
		outBuffer[1] += outSample[1] * rVol;
	}
};


template<class Traits>
struct MixStereoRamp : public Ramp
{
	MPT_ATTR_ALWAYSINLINE MPT_INLINE_FORCE void operator() (const typename Traits::outbuf_t &outSample, const ModChannel &chn, typename Traits::output_t * const outBuffer)
	{
		// TODO volume is not float, can we optimize this?
		lRamp += chn.leftRamp;
		rRamp += chn.rightRamp;
		outBuffer[0] += outSample[0] * (lRamp >> VOLUMERAMPPRECISION) * (1.0f / 4096.0f);
		outBuffer[1] += outSample[1] * (rRamp >> VOLUMERAMPPRECISION) * (1.0f / 4096.0f);
	}
};


//////////////////////////////////////////////////////////////////////////
// 5-channel surround mixing templates

// Surround mix: apply 5 separate gains to mono input, output to 5 speakers
template<class Traits>
struct MixSurroundNoRamp
{
	ModChannel &channel;
	float gainSL, gainFL, gainC, gainFR, gainSR;
	uint32 attackRemain;
	uint32 attackLen;

	MPT_ATTR_ALWAYSINLINE MPT_INLINE_FORCE MixSurroundNoRamp(ModChannel &chn)
		: channel{chn}
	{
		// Gains are pre-calculated and stored in channel (0-256 range)
		gainSL = static_cast<float>(chn.newSurroundSL) * (1.0f / 256.0f);
		gainFL = static_cast<float>(chn.newSurroundFL) * (1.0f / 256.0f);
		gainC = static_cast<float>(chn.newSurroundC) * (1.0f / 256.0f);
		gainFR = static_cast<float>(chn.newSurroundFR) * (1.0f / 256.0f);
		gainSR = static_cast<float>(chn.newSurroundSR) * (1.0f / 256.0f);
		attackRemain = chn.attackRampRemaining;
		attackLen = chn.attackRampLength;
	}

	MPT_ATTR_ALWAYSINLINE MPT_INLINE_FORCE ~MixSurroundNoRamp()
	{
		channel.attackRampRemaining = attackRemain;
	}

	MPT_ATTR_ALWAYSINLINE MPT_INLINE_FORCE void operator() (const typename Traits::outbuf_t &outSample, const ModChannel &chn, typename Traits::output_t * const outBuffer)
	{
		// Input: mono or stereo sample - fold to mono first
		typename Traits::output_t mono = outSample[0];
		if(Traits::numChannelsIn == 2)
		{
			mono = (outSample[0] + outSample[1]) * 0.5f;
		}

		// Apply attack fade if active
		if(attackRemain > 0 && attackLen > 0)
		{
			const uint32 progressed = attackLen - attackRemain;
			const float fade = static_cast<float>(std::min<uint32>(attackLen, progressed + 1)) / static_cast<float>(attackLen);
			mono *= fade;
			attackRemain--;
		}

		// The gains are converted to 0-1 range and already include volume from ReadNote()
		// Apply them directly to distribute mono to 5 speakers
		outBuffer[0] += mono * gainSL;
		outBuffer[1] += mono * gainFL;
		outBuffer[2] += mono * gainC;
		outBuffer[3] += mono * gainFR;
		outBuffer[4] += mono * gainSR;
	}
};


// Surround mix with volume ramping
template<class Traits>
struct MixSurroundRamp : public Ramp
{
	float slVol, flVol, cVol, frVol, srVol;
	float slRamp, flRamp, cRamp, frRamp, srRamp;
	uint32 attackRemain;
	uint32 attackLen;

	MPT_ATTR_ALWAYSINLINE MPT_INLINE_FORCE MixSurroundRamp(ModChannel &chn)
		: Ramp(chn)
	{
		slVol = static_cast<float>(chn.rampVolSurroundSL) * (1.0f / 4096.0f);
		flVol = static_cast<float>(chn.rampVolSurroundFL) * (1.0f / 4096.0f);
		cVol = static_cast<float>(chn.rampVolSurroundC) * (1.0f / 4096.0f);
		frVol = static_cast<float>(chn.rampVolSurroundFR) * (1.0f / 4096.0f);
		srVol = static_cast<float>(chn.rampVolSurroundSR) * (1.0f / 4096.0f);
		
		slRamp = static_cast<float>(chn.rampSurroundSL) * (1.0f / 4096.0f);
		flRamp = static_cast<float>(chn.rampSurroundFL) * (1.0f / 4096.0f);
		cRamp = static_cast<float>(chn.rampSurroundC) * (1.0f / 4096.0f);
		frRamp = static_cast<float>(chn.rampSurroundFR) * (1.0f / 4096.0f);
		srRamp = static_cast<float>(chn.rampSurroundSR) * (1.0f / 4096.0f);
		
		attackRemain = chn.attackRampRemaining;
		attackLen = chn.attackRampLength;
	}

	MPT_ATTR_ALWAYSINLINE MPT_INLINE_FORCE ~MixSurroundRamp()
	{
		channel.volSurroundSL = static_cast<int32>(slVol * 4096.0f + 0.5f);
		channel.volSurroundFL = static_cast<int32>(flVol * 4096.0f + 0.5f);
		channel.volSurroundC = static_cast<int32>(cVol * 4096.0f + 0.5f);
		channel.volSurroundFR = static_cast<int32>(frVol * 4096.0f + 0.5f);
		channel.volSurroundSR = static_cast<int32>(srVol * 4096.0f + 0.5f);
		channel.rampVolSurroundSL = static_cast<int32>(slVol * 4096.0f);
		channel.rampVolSurroundFL = static_cast<int32>(flVol * 4096.0f);
		channel.rampVolSurroundC = static_cast<int32>(cVol * 4096.0f);
		channel.rampVolSurroundFR = static_cast<int32>(frVol * 4096.0f);
		channel.rampVolSurroundSR = static_cast<int32>(srVol * 4096.0f);
		channel.attackRampRemaining = attackRemain;
	}

	MPT_ATTR_ALWAYSINLINE MPT_INLINE_FORCE void operator() (const typename Traits::outbuf_t &outSample, const ModChannel &chn, typename Traits::output_t * const outBuffer)
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
			mono = (outSample[0] + outSample[1]) * 0.5f;
		}

		// Apply short attack fade if active
		if(attackRemain > 0 && attackLen > 0)
		{
			const uint32 progressed = attackLen - attackRemain;
			const float fade = static_cast<float>(std::min<uint32>(attackLen, progressed + 1)) / static_cast<float>(attackLen);
			mono *= fade;
			attackRemain--;
		}

		// Apply ramped gains directly (same as stereo ramp mixer does)
		// Gains are already in 0-1 range and smoothly ramped per-sample
		outBuffer[0] += mono * slVol;
		outBuffer[1] += mono * flVol;
		outBuffer[2] += mono * cVol;
		outBuffer[3] += mono * frVol;
		outBuffer[4] += mono * srVol;
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

	MPT_ATTR_ALWAYSINLINE MPT_INLINE_FORCE ~ResonantFilter(ModChannel &chn)
	{
		for(int i = 0; i < Traits::numChannelsIn; i++)
		{
			channel.nFilter_Y[i][0] = fy[i][0];
			channel.nFilter_Y[i][1] = fy[i][1];
		}
	}

	// Filter values are clipped to double the input range
#define ClipFilter(x) Clamp(x, static_cast<Traits::output_t>(-2.0f), static_cast<Traits::output_t>(2.0f))

	MPT_ATTR_ALWAYSINLINE MPT_INLINE_FORCE void operator() (typename Traits::outbuf_t &outSample, const ModChannel &chn)
	{
		static_assert(static_cast<int>(Traits::numChannelsIn) <= static_cast<int>(Traits::numChannelsOut), "Too many input channels");

		for(int i = 0; i < Traits::numChannelsIn; i++)
		{
			typename Traits::output_t val = outSample[i] * chn.nFilter_A0 + ClipFilter(fy[i][0]) * chn.nFilter_B0 + ClipFilter(fy[i][1]) * chn.nFilter_B1;
			fy[i][1] = fy[i][0];
			fy[i][0] = val - (outSample[i] * chn.nFilter_HP);
			outSample[i] = val;
		}
	}

#undef ClipFilter
};


OPENMPT_NAMESPACE_END
