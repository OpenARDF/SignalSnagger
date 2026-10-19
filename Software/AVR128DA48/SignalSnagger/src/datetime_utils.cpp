#include "datetime_utils.h"

#include <limits.h>
#include <string.h>

namespace {

bool parseDigits(const char* text, size_t offset, size_t count, int* value)
{
	int parsed = 0;
	for (size_t i = 0; i < count; ++i)
	{
		char digit = text[offset + i];
		if (digit < '0' || digit > '9')
		{
			return false;
		}
		parsed = parsed * 10 + (digit - '0');
	}
	*value = parsed;
	return true;
}

bool isLeapYear(int year)
{
	return (year % 4 == 0 && year % 100 != 0) || year % 400 == 0;
}

int daysInMonth(int year, int month)
{
	static const uint8_t days[] = { 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31 };
	return month == 2 && isLeapYear(year) ? 29 : days[month - 1];
}

} // namespace

bool mystrptime(const char* text, struct tm* parsedTime)
{
	if (!text || !parsedTime)
	{
		return true;
	}

	size_t length = strlen(text);
	bool hasSeconds = length == 19 || length == 20;
	bool hasZuluSuffix = length == 17 || length == 20;
	if ((length != 16 && length != 17 && length != 19 && length != 20) ||
		(hasZuluSuffix && text[length - 1] != 'Z'))
	{
		return true;
	}

	if (text[4] != '-' || text[7] != '-' || text[10] != 'T' || text[13] != ':' ||
		(hasSeconds && text[16] != ':'))
	{
		return true;
	}

	int year;
	int month;
	int day;
	int hour;
	int minute;
	int second = 0;
	if (!parseDigits(text, 0, 4, &year) || !parseDigits(text, 5, 2, &month) ||
		!parseDigits(text, 8, 2, &day) || !parseDigits(text, 11, 2, &hour) ||
		!parseDigits(text, 14, 2, &minute) ||
		(hasSeconds && !parseDigits(text, 17, 2, &second)))
	{
		return true;
	}

	if (year < 1900 || year > 2100 || month < 1 || month > 12 ||
		day < 1 || day > daysInMonth(year, month) || hour > 23 || minute > 59 || second > 59)
	{
		return true;
	}

	struct tm result = {};
	result.tm_year = year - 1900;
	result.tm_mon = month - 1;
	result.tm_mday = day;
	result.tm_hour = hour;
	result.tm_min = minute;
	result.tm_sec = second;
	for (int priorMonth = 1; priorMonth < month; ++priorMonth)
	{
		result.tm_yday += daysInMonth(year, priorMonth);
	}
	result.tm_yday += day - 1;

	/* Commit only after complete validation so callers never receive partial state. */
	*parsedTime = result;
	return false;
}

uint32_t convertTimeStringToEpoch(const char* text)
{
	struct tm parsed = {};
	if (mystrptime(text, &parsed) || parsed.tm_year < 70)
	{
		return 0;
	}

	int year = parsed.tm_year + 1900;
	uint32_t days = 0;
	for (int priorYear = 1970; priorYear < year; ++priorYear)
	{
		days += isLeapYear(priorYear) ? 366U : 365U;
	}
	days += (uint32_t)parsed.tm_yday;

	uint64_t seconds = (uint64_t)days * 86400U + (uint32_t)parsed.tm_hour * 3600U +
		(uint32_t)parsed.tm_min * 60U + (uint32_t)parsed.tm_sec;
	return seconds <= UINT32_MAX ? (uint32_t)seconds : 0;
}
