#ifndef SI5351_FRACTION_H_
#define SI5351_FRACTION_H_

#include <stdint.h>

struct Si5351Fraction
{
	uint32_t integer;
	uint32_t numerator;
	uint32_t denominator;
};

/**
 * Express a PLL ratio as integer + numerator / denominator. Exact fractions
 * are retained when they fit; otherwise the nearest fraction using the maximum
 * hardware denominator is returned.
 */
bool si5351CalculateFraction(uint32_t multipliedFrequencyHz,
	uint32_t referenceFrequencyHz, uint32_t maximumDenominator,
	Si5351Fraction* result);

#endif /* SI5351_FRACTION_H_ */
