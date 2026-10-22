#ifndef BATTERY_ESTIMATOR_H
#define BATTERY_ESTIMATOR_H

#include <stdint.h>

float batteryVoltageFromAdcCounts(float adcCounts);
float batteryChargePercentFromVoltage(float voltage);
uint8_t batteryPercentForDisplay(float percent);
uint16_t batteryVoltageTenthsForDisplay(float voltage);

#endif
