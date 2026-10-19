#include "periodic_countdown.h"

bool periodicCountdownTick(uint8_t* ticksRemaining, uint8_t periodTicks)
{
	if (!ticksRemaining || !periodTicks)
	{
		return false;
	}

	if (*ticksRemaining > 1)
	{
		--(*ticksRemaining);
		return false;
	}

	/* A zero value is also recovered as an expired countdown. */
	*ticksRemaining = periodTicks;
	return true;
}
