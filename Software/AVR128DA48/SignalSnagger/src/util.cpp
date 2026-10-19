/*
 *  MIT License
 *
 *  Copyright (c) 2022 DigitalConfections
 *
 *  Permission is hereby granted, free of charge, to any person obtaining a copy
 *  of this software and associated documentation files (the "Software"), to deal
 *  in the Software without restriction, including without limitation the rights
 *  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *  copies of the Software, and to permit persons to whom the Software is
 *  furnished to do so, subject to the following conditions:
 *
 *  The above copyright notice and this permission notice shall be included in all
 *  copies or substantial portions of the Software.
 *
 *  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *  SOFTWARE.
 */


#include "util.h"
#include <avr/eeprom.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <math.h>

/**
 * Returns a-b
 * It appears difftime might not be handling subtraction of unsigned arguments correctly with current compiler. This function avoids any problems.
 */
int32_t timeDif(time_t a, time_t b)
{
	int32_t dif; // = difftime(now, g_event_start_epoch); // returns arg1 - arg2
	if(a > b)
		dif = a - b;
	else
		dif = -(b - a);

	return dif;
}


/***********************************************************************************************
 *  Print Formatting Utility Functions
 ************************************************************************************************/

/** 
 * Checks a string to see if it contains only numerical characters
 */
bool only_digits(char *s)
{
	while(*s)
	{
		if(isdigit(*s++) == 0)
		{
			return( false);
		}
	}

	return( true);
}


/** 
 * Convert a frequency string to a proper Hz value and string format based on assumptions 
 * related to the size and decimal properties of the number contained in the string.
 * result = pointer to a character sting to hold the frequency string
 * freq = the frequency value to be represented as a string
 * Returns 1 if an error is detected
 */
bool frequencyString(char* result, uint32_t freq)
{
	bool failure = true;
	
	if(!result)
	{
		return(failure);
	}
	
	if((freq > 3200000) && (freq < 8000000)) // Accept only a Hz value to be expressed in kHz
	{
		uint32_t k = (freq % 1000000) / 1000;
		uint32_t c = (freq % 1000);
		sprintf(result, "%lu.%03lu.%03lu", freq/1000000, k, c);
//		sprintf(result, "%lu.%03lu kHz", freq/1000000, k);
		
		failure = false;
	}
	
	return(failure);	
}

/** 
 * Convert a frequency string to a proper Hz value and string format based on assumptions 
 * related to the size and decimal properties of the number contained in the string.
 * str = pointer to a string containing the frequency string
 * result = pointer to a Frequency_Hz variable to hold the frequency in Hz
 * Returns 1 if an error is detected
 */
bool frequencyVal(char* str, Frequency_Hz* result)
{
	bool failure = true;
	
	if(!str)
	{
		return(failure);
	}
	
	int decimal = '.';
	char* decimalLocation = strchr(str, decimal);
	Frequency_Hz temp;
	
	if(decimalLocation) // Assume Hz or kHz
	{
		float f = atof(str);
		
		if((f > 3.5) && (f < 4.0))
		{
			f *= 1000000.;
			failure = false;
		}
		else if((f > 3500.) && (f < 4000.))
		{
			f *= 1000.;
			failure = false;
		}
		else if((f > 3500000.) && (f < 4000000.))
		{
			failure = false;
		}
		
		if(!failure)
		{
			temp = (Frequency_Hz)ceilf(f);
			temp = temp - (temp % 100);
			if(result) *result = temp;
			sprintf(str, "%4.1f kHz", (double)f);
		}
	}
	else
	{
		Frequency_Hz f = (Frequency_Hz)atol(str);
		
		if((f > 3500) && (f < 4000))
		{
			f *= 1000;
			failure = false;
		}
		else if((f > 3500000) && (f < 4000000))
		{
			f = f - (f % 100);
			failure = false;
		}
		
		if(!failure)
		{
			if(result) *result = f;
			sprintf(str, "%lu.%1lu kHz", f/1000, (f % 1000)/100);
		}
	}
	
	return(failure);	
}
