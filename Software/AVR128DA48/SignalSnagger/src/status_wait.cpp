#include "status_wait.h"

bool waitForStatusClear(StatusReader readStatus, TimeoutPoll pollTimeout, uint32_t timeoutMs)
{
	if (!readStatus || !pollTimeout || timeoutMs == 0)
	{
		return false;
	}

	/* The timer helper is stateful, so every independent wait starts clean. */
	pollTimeout(0);
	while (readStatus())
	{
		if (!pollTimeout(timeoutMs))
		{
			pollTimeout(0);
			return false;
		}
	}

	pollTimeout(0);
	return true;
}
