#include "audio_path.h"

namespace {

const int16_t ADC_MIN_CENTERED = -AUDIO_ADC_CENTER;
const int16_t ADC_MAX_CENTERED = AUDIO_ADC_CENTER - 1;
const int32_t FILTER_Q = INT32_C(256);

const uint8_t FILTER_TRANSITION_HALF = AUDIO_FILTER_TRANSITION_FRAMES / 2;

int16_t clampCentered(int32_t value)
{
	if (value < ADC_MIN_CENTERED)
	{
		return ADC_MIN_CENTERED;
	}
	if (value > ADC_MAX_CENTERED)
	{
		return ADC_MAX_CENTERED;
	}
	return (int16_t)value;
}

int32_t roundedQ8(int32_t value)
{
	const int32_t half = FILTER_Q / 2;
	return (value >= 0) ? ((value + half) / FILTER_Q)
		: ((value - half) / FILTER_Q);
}

} // namespace

AudioFilterMode audioFilterModeFromStored(uint8_t storedMode)
{
	switch (storedMode)
	{
		case AUDIO_FILTER_OFF:
		case AUDIO_FILTER_200_HZ:
		case AUDIO_FILTER_400_HZ:
		case AUDIO_FILTER_800_HZ:
			return (AudioFilterMode)storedMode;
		default:
			return AUDIO_FILTER_DEFAULT_MODE;
	}
}

AudioFilterMode nextAudioFilterMode(AudioFilterMode mode, bool up)
{
	mode = audioFilterModeFromStored((uint8_t)mode);
	if (up)
	{
		switch (mode)
		{
			case AUDIO_FILTER_OFF:
				return AUDIO_FILTER_200_HZ;
			case AUDIO_FILTER_200_HZ:
				return AUDIO_FILTER_400_HZ;
			case AUDIO_FILTER_400_HZ:
				return AUDIO_FILTER_800_HZ;
			case AUDIO_FILTER_800_HZ:
			default:
				return AUDIO_FILTER_OFF;
		}
	}
	switch (mode)
	{
		case AUDIO_FILTER_OFF:
			return AUDIO_FILTER_800_HZ;
		case AUDIO_FILTER_200_HZ:
			return AUDIO_FILTER_OFF;
		case AUDIO_FILTER_400_HZ:
			return AUDIO_FILTER_200_HZ;
		case AUDIO_FILTER_800_HZ:
		default:
			return AUDIO_FILTER_400_HZ;
	}
}

uint16_t audioFilterBandwidthHz(AudioFilterMode mode)
{
	switch (audioFilterModeFromStored((uint8_t)mode))
	{
		case AUDIO_FILTER_200_HZ:
			return 200;
		case AUDIO_FILTER_400_HZ:
			return 400;
		case AUDIO_FILTER_800_HZ:
			return 800;
		case AUDIO_FILTER_OFF:
		default:
			return 0;
	}
}

int16_t audioScaleAndSaturate(int16_t centeredSample, uint8_t gain,
	int16_t injectedSample)
{
	int32_t scaled = centeredSample;
	if (gain > AUDIO_NEUTRAL_GAIN)
	{
		uint8_t shift = gain - AUDIO_NEUTRAL_GAIN;
		if (shift > 6)
		{
			shift = 6;
		}
		scaled *= (INT32_C(1) << shift);
	}
	else if (gain < AUDIO_NEUTRAL_GAIN)
	{
		scaled /= (INT32_C(1) << (AUDIO_NEUTRAL_GAIN - gain));
	}

	/* The injected tone is signed and zero-centered, so starting or stopping it
	 * cannot create the modulo-1024 discontinuity of the former unsigned path. */
	scaled += injectedSample;
	return clampCentered(scaled);
}

uint16_t audioCenteredToDac(int16_t centeredSample)
{
	const int16_t bounded = clampCentered(centeredSample);
	return (uint16_t)((bounded / 4) + AUDIO_DAC_CENTER);
}

AudioPathProcessor::AudioPathProcessor()
{
	Reset();
}

void AudioPathProcessor::Reset(AudioFilterMode mode)
{
	decimationSum_ = 0;
	decimationCount_ = 0;
	activeMode_ = audioFilterModeFromStored((uint8_t)mode);
	pendingMode_ = activeMode_;
	transitionFramesRemaining_ = 0;
	ActivateFilterMode(activeMode_);
	lastDacValue_ = AUDIO_DAC_CENTER;
}

void AudioPathProcessor::ClearBandpassState(void)
{
	for (uint8_t index = 0; index < 2; ++index)
	{
		bandpass_[index].x1 = 0;
		bandpass_[index].x2 = 0;
		bandpass_[index].y1 = 0;
		bandpass_[index].y2 = 0;
	}
}

void AudioPathProcessor::ActivateFilterMode(AudioFilterMode mode)
{
	activeMode_ = audioFilterModeFromStored((uint8_t)mode);
	/* These Q8 constant-peak RBJ profiles are generated offline for a 12 kHz
	 * sample rate and 800 Hz center. Loading literals only when the mode changes
	 * keeps floating point and coefficient-table reads out of the audio ISR. */
	switch (activeMode_)
	{
		case AUDIO_FILTER_400_HZ:
			coefficients_.b0 = INT16_C(37);
			coefficients_.b2 = -INT16_C(37);
			coefficients_.a1 = -INT16_C(401);
			coefficients_.a2 = INT16_C(182);
			break;
		case AUDIO_FILTER_800_HZ:
			coefficients_.b0 = INT16_C(65);
			coefficients_.b2 = -INT16_C(65);
			coefficients_.a1 = -INT16_C(349);
			coefficients_.a2 = INT16_C(126);
			break;
		case AUDIO_FILTER_200_HZ:
		case AUDIO_FILTER_OFF:
		default:
			coefficients_.b0 = INT16_C(19);
			coefficients_.b2 = -INT16_C(19);
			coefficients_.a1 = -INT16_C(433);
			coefficients_.a2 = INT16_C(217);
			break;
	}
	ClearBandpassState();
}

bool AudioPathProcessor::WillProduceFrame(void) const
{
	return decimationCount_ == (AUDIO_DECIMATION_FACTOR - 1);
}

int16_t AudioPathProcessor::ProcessBandpassSection(int16_t input,
	BiquadState* state, const BiquadCoefficients& coefficients)
{
	/* AVR int is 16 bits. Cast every product before multiplication; assigning a
	 * wrapped 16-bit expression to a 32-bit accumulator is already too late. */
	const int32_t accumulator =
		((int32_t)coefficients.b0 * input) +
		((int32_t)coefficients.b2 * state->x2) -
		((int32_t)coefficients.a1 * state->y1) -
		((int32_t)coefficients.a2 * state->y2);
	const int16_t output = clampCentered(roundedQ8(accumulator));

	state->x2 = state->x1;
	state->x1 = input;
	state->y2 = state->y1;
	state->y1 = output;
	return output;
}

int16_t AudioPathProcessor::ProcessListeningSample(int16_t source)
{
	if (activeMode_ == AUDIO_FILTER_OFF)
	{
		return source;
	}
	const int16_t firstStage = ProcessBandpassSection(source, &bandpass_[0],
		coefficients_);
	return ProcessBandpassSection(firstStage, &bandpass_[1], coefficients_);
}

bool AudioPathProcessor::CaptureAdcSample(uint16_t adcSample,
	int16_t* decimatedSample)
{
	if (adcSample > 4095)
	{
		adcSample = 4095;
	}
	decimationSum_ += (int16_t)adcSample - AUDIO_ADC_CENTER;
	++decimationCount_;
	if (decimationCount_ < AUDIO_DECIMATION_FACTOR)
	{
		return false;
	}

	const int16_t decimated = clampCentered(decimationSum_ /
		AUDIO_DECIMATION_FACTOR);
	decimationSum_ = 0;
	decimationCount_ = 0;
	if (decimatedSample)
	{
		*decimatedSample = decimated;
	}
	return true;
}

void AudioPathProcessor::ProcessFrame(int16_t decimatedSample, uint8_t gain,
	bool feedthrough, int16_t injectedSample, AudioFilterMode requestedMode,
	AudioFrame* frame)
{
	requestedMode = audioFilterModeFromStored((uint8_t)requestedMode);
	if (requestedMode != pendingMode_)
	{
		pendingMode_ = requestedMode;
		transitionFramesRemaining_ = AUDIO_FILTER_TRANSITION_FRAMES;
	}

	const int16_t source = feedthrough ? decimatedSample : 0;
	int16_t listeningSample = ProcessListeningSample(source);
	if (transitionFramesRemaining_)
	{
		uint8_t rampNumerator;
		if (transitionFramesRemaining_ > FILTER_TRANSITION_HALF)
		{
			rampNumerator = transitionFramesRemaining_ - FILTER_TRANSITION_HALF;
			transitionFramesRemaining_--;
			if (transitionFramesRemaining_ == FILTER_TRANSITION_HALF)
			{
				ActivateFilterMode(pendingMode_);
			}
		}
		else
		{
			rampNumerator = FILTER_TRANSITION_HALF + 1U -
				transitionFramesRemaining_;
			transitionFramesRemaining_--;
		}
		listeningSample = (int16_t)(((int32_t)listeningSample * rampNumerator) /
			FILTER_TRANSITION_HALF);
	}
	/* Apply listening gain after filtering so rejected energy cannot consume
	 * internal headroom. Injected UI tones retain their established fixed level. */
	const int16_t output = audioScaleAndSaturate(listeningSample, gain,
		injectedSample);
	lastDacValue_ = audioCenteredToDac(output);

	if (frame)
	{
		frame->dacValue = lastDacValue_;
		frame->analysisSample = (uint16_t)(decimatedSample + AUDIO_ADC_CENTER);
	}

}

bool AudioPathProcessor::ProcessAdcSample(uint16_t adcSample, uint8_t gain,
	bool feedthrough, int16_t injectedSample, AudioFilterMode requestedMode,
	AudioFrame* frame)
{
	int16_t decimatedSample = 0;
	if (!CaptureAdcSample(adcSample, &decimatedSample))
	{
		return false;
	}
	ProcessFrame(decimatedSample, gain, feedthrough, injectedSample,
		requestedMode, frame);
	return true;
}

uint16_t AudioPathProcessor::LastDacValue(void) const
{
	return lastDacValue_;
}
