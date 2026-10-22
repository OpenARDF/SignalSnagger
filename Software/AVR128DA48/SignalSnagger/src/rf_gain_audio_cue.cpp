#include "rf_gain_audio_cue.h"

RfGainAudioCue::RfGainAudioCue()
{
	Cancel();
}

void RfGainAudioCue::Cancel(void)
{
	phase_ = Idle;
	framesRemaining_ = 0;
	displayedGain_ = RF_GAIN_CUE_NO_REQUEST;
	elementIndex_ = 0;
	elementCount_ = 0;
	leadingDashCount_ = 0;
	useLongPulses_ = false;
	toneStartPending_ = false;
}

void RfGainAudioCue::Schedule(uint8_t displayedGain)
{
	if (displayedGain > RF_GAIN_CUE_MAX_DISPLAYED_GAIN)
	{
		Cancel();
		return;
	}

	/* Wait for the knob to stop before announcing the absolute value. This keeps
	 * rapid turns responsive without queueing obsolete intermediate settings. */
	displayedGain_ = displayedGain;
	phase_ = Settling;
	framesRemaining_ = RF_GAIN_CUE_SETTLE_FRAMES;
	elementIndex_ = 0;
	elementCount_ = 0;
	leadingDashCount_ = 0;
	useLongPulses_ = false;
	toneStartPending_ = false;
}

bool RfGainAudioCue::Active(void) const
{
	return phase_ != Idle;
}

uint8_t RfGainAudioCue::DisplayedGain(void) const
{
	return displayedGain_;
}

void RfGainAudioCue::BeginPattern(void)
{
	elementIndex_ = 0;
	useLongPulses_ = displayedGain_ == 0 || displayedGain_ == 10;

	if (displayedGain_ == 0)
	{
		elementCount_ = 1;
		leadingDashCount_ = 0;
	}
	else if (displayedGain_ <= 5)
	{
		/* Gain 1-5 sends Morse E, I, S, H, and 5: one to five dots. */
		elementCount_ = displayedGain_;
		leadingDashCount_ = 0;
	}
	else if (displayedGain_ < 10)
	{
		/* Gain 6-9 sends the corresponding Morse digit: leading dashes
		 * followed by enough dots to retain five total elements. */
		elementCount_ = 5;
		leadingDashCount_ = displayedGain_ - 5;
	}
	else
	{
		/* Gain 10 is modified Morse M: two pulses, each matching zero's
		 * deliberately distinctive 300 ms pulse. */
		elementCount_ = 2;
		leadingDashCount_ = 0;
	}

	BeginElement();
}

void RfGainAudioCue::BeginElement(void)
{
	if (useLongPulses_)
	{
		framesRemaining_ = RF_GAIN_CUE_LONG_FRAMES;
	}
	else if (elementIndex_ < leadingDashCount_)
	{
		framesRemaining_ = RF_GAIN_CUE_DASH_FRAMES;
	}
	else
	{
		framesRemaining_ = RF_GAIN_CUE_DOT_FRAMES;
	}
	phase_ = Tone;
	toneStartPending_ = true;
}

void RfGainAudioCue::CompleteElement(void)
{
	++elementIndex_;
	if (elementIndex_ >= elementCount_)
	{
		phase_ = Idle;
		framesRemaining_ = 0;
		return;
	}

	phase_ = Gap;
	framesRemaining_ = RF_GAIN_CUE_GAP_FRAMES;
}

bool RfGainAudioCue::NextFrame(bool* toneStarted)
{
	if (toneStarted)
	{
		*toneStarted = false;
	}

	switch (phase_)
	{
		case Settling:
			if (--framesRemaining_ == 0)
			{
				BeginPattern();
			}
			return false;

		case Gap:
			if (--framesRemaining_ == 0)
			{
				BeginElement();
			}
			return false;

		case Tone:
			if (toneStarted)
			{
				*toneStarted = toneStartPending_;
			}
			toneStartPending_ = false;
			if (--framesRemaining_ == 0)
			{
				CompleteElement();
			}
			return true;

		case Idle:
		default:
			return false;
	}
}
