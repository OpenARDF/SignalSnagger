#include "power_on_clock.h"

#include <stdio.h>

namespace
{
	const uint32_t MAX_DISPLAYED_TIME_SECONDS = UINT32_C(999) * 60U + 59U;
	const uint16_t MAX_DISPLAYED_CYCLE = UINT16_C(999);
	const uint16_t SECONDS_PER_CYCLE = UINT16_C(300);
	const uint8_t REQUIRED_SENSE_PRESSES = UINT8_C(5);
}

PowerOnClockReading powerOnClockReading(uint32_t elapsedSeconds)
{
	const uint32_t displayedSeconds = elapsedSeconds > MAX_DISPLAYED_TIME_SECONDS
		? MAX_DISPLAYED_TIME_SECONDS
		: elapsedSeconds;
	/* Cycles are one-based: elapsed seconds 0 through 299 are cycle 1. */
	uint32_t cycle = elapsedSeconds / SECONDS_PER_CYCLE + 1U;
	if (cycle > MAX_DISPLAYED_CYCLE)
	{
		cycle = MAX_DISPLAYED_CYCLE;
	}

	PowerOnClockReading reading;
	reading.minutes = (uint16_t)(displayedSeconds / 60U);
	reading.seconds = (uint8_t)(displayedSeconds % 60U);
	reading.cycle = (uint16_t)cycle;
	return reading;
}

void formatPowerOnClockRows(uint32_t elapsedSeconds,
	char timeRow[POWER_ON_CLOCK_TIME_ROW_SIZE],
	char cycleRow[POWER_ON_CLOCK_CYCLE_ROW_SIZE])
{
	const PowerOnClockReading reading = powerOnClockReading(elapsedSeconds);
	snprintf(timeRow, POWER_ON_CLOCK_TIME_ROW_SIZE, "%03u:%02u",
		(unsigned int)reading.minutes,
		(unsigned int)reading.seconds);
	snprintf(cycleRow, POWER_ON_CLOCK_CYCLE_ROW_SIZE, "Cycle: %u",
		(unsigned int)reading.cycle);
}

ClockDisplayGesture::ClockDisplayGesture()
	: button_(CLOCK_SENSE_LEFT), pressCount_(0)
{
}

bool ClockDisplayGesture::RecordSensePress(ClockSenseButton button)
{
	if (!pressCount_)
	{
		button_ = button;
		pressCount_ = 1;
		return false;
	}

	/* A press of the other sense button cancels the old sequence and is not
	 * reused as the first press of a new one. This makes cancellation explicit. */
	if (button != button_)
	{
		Cancel();
		return false;
	}

	pressCount_++;
	if (pressCount_ == REQUIRED_SENSE_PRESSES)
	{
		Cancel();
		return true;
	}

	return false;
}

void ClockDisplayGesture::Cancel()
{
	pressCount_ = 0;
}
