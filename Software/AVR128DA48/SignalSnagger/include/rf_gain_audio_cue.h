#ifndef RF_GAIN_AUDIO_CUE_H_
#define RF_GAIN_AUDIO_CUE_H_

#include <stdint.h>

/* Timing is expressed in 12 kHz output frames so cue generation stays integer,
 * deterministic, and cheap enough to advance from the audio ISR. At 25 WPM a
 * Morse dot and intra-character gap are 576 frames; a dash is three dots. */
#define RF_GAIN_CUE_DOT_FRAMES UINT16_C(576)
#define RF_GAIN_CUE_DASH_FRAMES UINT16_C(1728)
#define RF_GAIN_CUE_GAP_FRAMES UINT16_C(576)
#define RF_GAIN_CUE_LONG_FRAMES UINT16_C(3600)
#define RF_GAIN_CUE_SETTLE_FRAMES UINT16_C(3600)
#define RF_GAIN_CUE_TONE_HZ UINT16_C(250)
#define RF_GAIN_CUE_TONE_PERIOD_FRAMES UINT8_C(48)
#define RF_GAIN_CUE_MAX_DISPLAYED_GAIN UINT8_C(10)
#define RF_GAIN_CUE_NO_REQUEST UINT8_C(0xff)

class RfGainAudioCue
{
public:
	RfGainAudioCue();

	/* Scheduling is latest-value-wins: another detent restarts the settling
	 * interval and replaces any pattern already playing. */
	void Schedule(uint8_t displayedGain);
	void Cancel(void);
	bool NextFrame(bool* toneStarted);
	bool Active(void) const;
	uint8_t DisplayedGain(void) const;

private:
	enum Phase : uint8_t
	{
		Idle,
		Settling,
		Tone,
		Gap
	};

	void BeginPattern(void);
	void BeginElement(void);
	void CompleteElement(void);

	Phase phase_;
	uint16_t framesRemaining_;
	uint8_t displayedGain_;
	uint8_t elementIndex_;
	uint8_t elementCount_;
	uint8_t leadingDashCount_;
	bool useLongPulses_;
	bool toneStartPending_;
};

#endif /* RF_GAIN_AUDIO_CUE_H_ */
