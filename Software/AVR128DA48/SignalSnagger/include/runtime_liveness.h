#ifndef SIGNALSNAGGER_RUNTIME_LIVENESS_H_
#define SIGNALSNAGGER_RUNTIME_LIVENESS_H_

#include <stdint.h>

/* At 12 kframe/s, 64 consecutive unconsumed DSP frames represent more than
 * five milliseconds of starvation. Normal scheduling must recover well before
 * this point; tripping stops the high-priority source so the watchdog can
 * restore the complete receiver instead of leaving the UI permanently wedged. */
#define AUDIO_BACKLOG_TRIP_FRAMES UINT8_C(64)

static inline uint8_t audioBacklogAfterFrame(uint8_t consecutiveDrops,
	bool frameDropped)
{
	if (!frameDropped)
	{
		return 0;
	}
	return consecutiveDrops < AUDIO_BACKLOG_TRIP_FRAMES
		? (uint8_t)(consecutiveDrops + 1U)
		: AUDIO_BACKLOG_TRIP_FRAMES;
}

static inline bool audioBacklogMustTrip(uint8_t consecutiveDrops)
{
	return consecutiveDrops >= AUDIO_BACKLOG_TRIP_FRAMES;
}

struct RuntimeLivenessSnapshot
{
	uint8_t switchService;
	uint8_t adcCapture;
	uint8_t audioDsp;
};

/* The foreground owns this gate. A watchdog feed is authorized only after
 * every service that is expected in the current mode has advanced since the
 * previous feed. Merely executing a high-priority audio ISR is insufficient. */
class RuntimeLivenessGate
{
public:
	RuntimeLivenessGate() : previous_{0, 0, 0}, initialized_(false) {}

	void Reset(const RuntimeLivenessSnapshot& current)
	{
		previous_ = current;
		initialized_ = true;
	}

	bool ShouldFeed(const RuntimeLivenessSnapshot& current,
		bool audioExpected, bool audioLockout)
	{
		if (!initialized_)
		{
			Reset(current);
			return false;
		}
		if (audioLockout || current.switchService == previous_.switchService)
		{
			return false;
		}
		if (audioExpected &&
			(current.adcCapture == previous_.adcCapture ||
			current.audioDsp == previous_.audioDsp))
		{
			return false;
		}

		previous_ = current;
		return true;
	}

private:
	RuntimeLivenessSnapshot previous_;
	bool initialized_;
};

#endif /* SIGNALSNAGGER_RUNTIME_LIVENESS_H_ */
