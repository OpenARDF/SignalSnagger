#include "signal_score.h"
#include <math.h>

SignalDetectorStability::SignalDetectorStability()
{
	Reset();
}

void SignalDetectorStability::Reset()
{
	smoothedLevel_ = 0.0f;
	smoothingInitialized_ = false;
	signalDetected_ = false;
}

float SignalDetectorStability::Smooth(float level)
{
	if (!smoothingInitialized_)
	{
		smoothedLevel_ = level;
		smoothingInitialized_ = true;
	}
	else
	{
		/* Equal weighting suppresses isolated block-to-block magnitude changes
		 * without adding more than one detector update of significant lag. */
		smoothedLevel_ = 0.5f * (smoothedLevel_ + level);
	}

	return smoothedLevel_;
}

bool SignalDetectorStability::Update(float level, float nominalNoise,
	float detectedThreshold)
{
	if (!(detectedThreshold > nominalNoise))
	{
		signalDetected_ = false;
		return false;
	}

	if (!signalDetected_)
	{
		signalDetected_ = level > detectedThreshold;
	}
	else
	{
		const float releaseThreshold = nominalNoise +
			0.7f * (detectedThreshold - nominalNoise);
		/* The ordered comparison also releases a latched detection for NaN. */
		signalDetected_ = level >= releaseThreshold;
	}

	return signalDetected_;
}

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
