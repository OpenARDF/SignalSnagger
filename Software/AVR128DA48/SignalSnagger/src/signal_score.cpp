#include "signal_score.h"
#include <math.h>

uint16_t receiverSignalScore(float detectedThreshold, float nominalNoise)
{
	const float baseline = nominalNoise + 20.0f;
	if (!(detectedThreshold > 0.0f) || !(baseline > 0.0f))
	{
		return 0;
	}

	const float score = 100.0f * (log10f(detectedThreshold) - log10f(baseline));
	/* This also rejects NaN because every ordered comparison with NaN is false. */
	if (!(score > 0.0f))
	{
		return 0;
	}
	if (score >= (float)UINT16_MAX)
	{
		return UINT16_MAX;
	}

	return (uint16_t)score;
}
