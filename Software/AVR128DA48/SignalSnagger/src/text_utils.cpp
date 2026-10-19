#include "text_utils.h"

bool only_digits(const char* text)
{
	if (!text)
	{
		return false;
	}

	while (*text)
	{
		if (*text < '0' || *text > '9')
		{
			return false;
		}
		++text;
	}
	return true;
}

bool isDisplayable(const char* text, size_t maximumLength)
{
	if (!text)
	{
		return false;
	}

	for (size_t i = 0; i < maximumLength && text[i]; ++i)
	{
		unsigned char value = (unsigned char)text[i];
		if (value < 32 || value > 126)
		{
			return false;
		}
	}
	return true;
}
