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

#define PUSHBUTTON_AUDIO_FEEDBACK_DEFAULT true

extern volatile bool g_pushbutton_audio_feedback_enabled;

inline bool pushbuttonAudioFeedbackFromStored(uint8_t storedSetting)
{
	return storedSetting != PUSHBUTTON_AUDIO_FEEDBACK_DISABLED;
}

inline uint8_t pushbuttonAudioFeedbackToStored(bool enabled)
{
	return enabled ? PUSHBUTTON_AUDIO_FEEDBACK_ENABLED
		: PUSHBUTTON_AUDIO_FEEDBACK_DISABLED;
}

#endif /* SOUND_SETTINGS_H_ */
