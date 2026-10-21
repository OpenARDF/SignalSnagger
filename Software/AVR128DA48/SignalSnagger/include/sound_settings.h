#ifndef SOUND_SETTINGS_H_
#define SOUND_SETTINGS_H_

#include <stdint.h>

enum PushbuttonAudioFeedbackSetting : uint8_t
{
	/* Reserved encodings make legacy or erased EEPROM bytes fall back to the
	 * enabled default instead of silently muting pushbutton feedback. */
	PUSHBUTTON_AUDIO_FEEDBACK_DISABLED = 0xa4,
	PUSHBUTTON_AUDIO_FEEDBACK_ENABLED = 0xa5
};

enum TurnTickAudioFeedbackSetting : uint8_t
{
	/* Keep legacy and erased bytes enabled, matching the historical behavior. */
	TURN_TICK_AUDIO_FEEDBACK_DISABLED = 0xa6,
	TURN_TICK_AUDIO_FEEDBACK_ENABLED = 0xa7
};

#define PUSHBUTTON_AUDIO_FEEDBACK_DEFAULT true
#define TURN_TICK_AUDIO_FEEDBACK_DEFAULT true

extern volatile bool g_pushbutton_audio_feedback_enabled;
extern volatile bool g_turn_tick_audio_feedback_enabled;

inline bool pushbuttonAudioFeedbackFromStored(uint8_t storedSetting)
{
	return storedSetting != PUSHBUTTON_AUDIO_FEEDBACK_DISABLED;
}

inline uint8_t pushbuttonAudioFeedbackToStored(bool enabled)
{
	return enabled ? PUSHBUTTON_AUDIO_FEEDBACK_ENABLED
		: PUSHBUTTON_AUDIO_FEEDBACK_DISABLED;
}

inline bool turnTickAudioFeedbackFromStored(uint8_t storedSetting)
{
	return storedSetting != TURN_TICK_AUDIO_FEEDBACK_DISABLED;
}

inline uint8_t turnTickAudioFeedbackToStored(bool enabled)
{
	return enabled ? TURN_TICK_AUDIO_FEEDBACK_ENABLED
		: TURN_TICK_AUDIO_FEEDBACK_DISABLED;
}

#endif /* SOUND_SETTINGS_H_ */
