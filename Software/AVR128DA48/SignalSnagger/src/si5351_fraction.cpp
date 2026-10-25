#include "si5351_fraction.h"

namespace {

uint32_t greatestCommonDivisor(uint32_t left, uint32_t right)
{
	while (right != 0)
	{
		const uint32_t remainder = left % right;
		left = right;
		right = remainder;
	}
	return left;
}

} // namespace

bool si5351CalculateFraction(uint32_t multipliedFrequencyHz,
	uint32_t referenceFrequencyHz, uint32_t maximumDenominator,
	Si5351Fraction* result)
{
	if (!referenceFrequencyHz || !maximumDenominator || !result)
	{
		return false;
	}

	Si5351Fraction candidate = {};
	candidate.integer = multipliedFrequencyHz / referenceFrequencyHz;
	const uint32_t remainder = multipliedFrequencyHz % referenceFrequencyHz;
	if (remainder == 0)
	{
		candidate.denominator = 1;
		*result = candidate;
		return true;
	}

	const uint32_t divisor = greatestCommonDivisor(remainder,
		referenceFrequencyHz);
	candidate.numerator = remainder / divisor;
	candidate.denominator = referenceFrequencyHz / divisor;
	if (candidate.denominator <= maximumDenominator)
	{
		*result = candidate;
		return true;
	}

	/* The exact fraction for some 10 Hz steps needs more than the Si5351's
	 * 20-bit denominator. A 64-bit intermediate prevents overflow while the
	 * rounded maximum-denominator approximation keeps output error below 0.1 Hz. */
	candidate.denominator = maximumDenominator;
	candidate.numerator = (uint32_t)(((uint64_t)remainder * maximumDenominator +
		referenceFrequencyHz / 2U) / referenceFrequencyHz);
	if (candidate.numerator >= candidate.denominator)
	{
		++candidate.integer;
		candidate.numerator = 0;
		candidate.denominator = 1;
	}

	*result = candidate;
	return true;
}
