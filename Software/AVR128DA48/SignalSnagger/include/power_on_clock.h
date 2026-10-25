#ifndef SIGNALSNAGGER_POWER_ON_CLOCK_H_
#define SIGNALSNAGGER_POWER_ON_CLOCK_H_

#include <stdint.h>

#include "multi_press_sequence.h"

#define POWER_ON_CLOCK_TIME_ROW_SIZE 7
#define POWER_ON_CLOCK_CYCLE_ROW_SIZE 11
#define POWER_ON_CLOCK_MAX_TRACKED_SECONDS UINT32_C(299400)

enum ClockSenseButton
{
	CLOCK_SENSE_LEFT,
	CLOCK_SENSE_RIGHT
};

struct PowerOnClockReading
{
	uint16_t minutes;
	uint8_t seconds;
	uint16_t cycle;
};

/** Convert elapsed uptime into the independently saturated clock fields. */
PowerOnClockReading powerOnClockReading(uint32_t elapsedSeconds);

/** Format the two ten-column LCD rows without display-position prefixes. */
void formatPowerOnClockRows(uint32_t elapsedSeconds,
	char timeRow[POWER_ON_CLOCK_TIME_ROW_SIZE],
	char cycleRow[POWER_ON_CLOCK_CYCLE_ROW_SIZE]);

class ClockDisplayGesture
{
public:
	ClockDisplayGesture();

	/** Record one sense-button closure for resolution after the quiet period. */
	void RecordSensePress(ClockSenseButton button);

	/** Advance the timeout and return true only for an exact five-press burst. */
	bool Tick();

	/** Cancel a partial sequence after any unrelated button or encoder activity. */
	void Cancel();

private:
	MultiPressSequence<5> sequence_;
};

#endif /* SIGNALSNAGGER_POWER_ON_CLOCK_H_ */
