#include "battery_estimator.h"

#include <stddef.h>

namespace {

struct BatteryCurvePoint
{
	uint16_t millivolts;
	uint8_t percent;
};

/* A representative resting-voltage curve gives a useful estimate across
 * different 2S LiPo packs. Loaded, cold, or aged packs will read somewhat
 * lower, so this remains an intentionally approximate fuel gauge. */
const BatteryCurvePoint kBatteryCurve[] = {
	{8400, 100},
	{8200, 90},
	{8000, 80},
	{7900, 70},
	{7800, 60},
	{7700, 50},
	{7600, 40},
	{7500, 25},
	{7400, 10},
	{7200, 5},
	{6600, 0}
};

float clampPercent(float percent)
{
	if(percent < 0.0f)
	{
		return 0.0f;
	}
	if(percent > 100.0f)
	{
		return 100.0f;
	}
	return percent;
}

}

float batteryVoltageFromAdcCounts(float adcCounts)
{
	/* The divider maps 28.8 V to the 12-bit ADC range. The measured hardware
	 * needs the intentional +0.2 V calibration correction. */
	return adcCounts * 28.8f / 4096.0f + 0.2f;
}

float batteryChargePercentFromVoltage(float voltage)
{
	const float batteryMillivolts = voltage * 1000.0f;
	const size_t pointCount = sizeof(kBatteryCurve) / sizeof(kBatteryCurve[0]);

	if(batteryMillivolts >= kBatteryCurve[0].millivolts)
	{
		return 100.0f;
	}
	if(batteryMillivolts <= kBatteryCurve[pointCount - 1].millivolts)
	{
		return 0.0f;
	}

	for(size_t i = 1; i < pointCount; ++i)
	{
		const BatteryCurvePoint& upper = kBatteryCurve[i - 1];
		const BatteryCurvePoint& lower = kBatteryCurve[i];
		if(batteryMillivolts >= lower.millivolts)
		{
			const float position =
				(batteryMillivolts - lower.millivolts) /
				(upper.millivolts - lower.millivolts);
			return lower.percent + position * (upper.percent - lower.percent);
		}
	}

	return 0.0f;
}

uint8_t batteryPercentForDisplay(float percent)
{
	const uint8_t rounded = (uint8_t)(clampPercent(percent) + 2.5f);
	return (uint8_t)((rounded / 5U) * 5U);
}

uint16_t batteryVoltageTenthsForDisplay(float voltage)
{
	if(voltage <= 0.0f)
	{
		return 0;
	}
	return (uint16_t)(voltage * 10.0f + 0.5f);
}
