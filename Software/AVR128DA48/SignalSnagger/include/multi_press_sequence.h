#ifndef SIGNALSNAGGER_MULTI_PRESS_SEQUENCE_H_
#define SIGNALSNAGGER_MULTI_PRESS_SEQUENCE_H_

#include <stdint.h>

/* SignalSlinger's proven 40-scan inter-press window is 0.8 seconds on the
 * shared 50 Hz debounced-input timebase. */
#define MULTI_PRESS_SEQUENCE_TIMEOUT_TICKS UINT8_C(40)
#define MULTI_PRESS_SEQUENCE_MAX_COUNT UINT8_C(9)

/**
 * Count matching input closures and expire the partial sequence unless every
 * next closure arrives within a fresh 0.8-second window.
 */
template <uint8_t RequiredPresses>
class MultiPressSequence
{
public:
	MultiPressSequence()
		: input_(0), pressCount_(0), timeoutTicks_(0), rejected_(false)
	{
		static_assert(RequiredPresses >= 1,
			"A multi-press sequence must require at least one press");
		static_assert(RequiredPresses <= MULTI_PRESS_SEQUENCE_MAX_COUNT,
			"A multi-press sequence cannot require more than nine presses");
	}

	void RecordPress(uint8_t input)
	{
		if ((pressCount_ || rejected_) && input != input_)
		{
			/* A different input cancels the burst and is deliberately not reused
			 * as the first press of another command. */
			Cancel();
			return;
		}

		if (rejected_)
		{
			/* Keep an overlong burst rejected until the user pauses for the full
			 * inter-press interval; do not reinterpret its tail as a new command. */
			timeoutTicks_ = MULTI_PRESS_SEQUENCE_TIMEOUT_TICKS;
			return;
		}

		if (!pressCount_)
		{
			input_ = input;
			pressCount_ = 1;
		}
		else if (pressCount_ < MULTI_PRESS_SEQUENCE_MAX_COUNT)
		{
			pressCount_++;
		}
		else
		{
			pressCount_ = 0;
			rejected_ = true;
		}

		/* SignalSlinger restarts its complete inter-press window after every
		 * accepted closure; keep the same invariant for SignalSnagger. */
		timeoutTicks_ = MULTI_PRESS_SEQUENCE_TIMEOUT_TICKS;
	}

	bool Tick()
	{
		if (!timeoutTicks_ || --timeoutTicks_)
		{
			return false;
		}

		/* Resolve only after the quiet period so an undefined longer burst
		 * cannot accidentally execute a shorter defined command. */
		const bool recognized = !rejected_ && pressCount_ == RequiredPresses;
		Cancel();
		return recognized;
	}

	void Cancel()
	{
		pressCount_ = 0;
		timeoutTicks_ = 0;
		rejected_ = false;
	}

private:
	uint8_t input_;
	uint8_t pressCount_;
	uint8_t timeoutTicks_;
	bool rejected_;
};

#endif /* SIGNALSNAGGER_MULTI_PRESS_SEQUENCE_H_ */
