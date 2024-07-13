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

#include "defs.h"
#include "adc.h"
#include <avr/io.h>
#include <stdbool.h>
#include <driver_init.h>
#include <compiler.h>
#include "Goertzel.h"

#define SAMPLE_RATE 13889
#define Goertzel_N 101
const int N = Goertzel_N;
const float threshold = 500000. * (Goertzel_N / 100.);
const float sampling_freq = SAMPLE_RATE;
// const float x_frequencies[4] = { 1209., 1336., 1477., 1633. };
// const float pitch_frequencies[4] = { 697., 770., 852., 941. };
	
#define FREE_RUNNING true
#define SINGLE_CONVERSION false
	
volatile int16_t g_adcVal;
Goertzel g_goertzel(N, sampling_freq);

static void PORT_init(void);
static void ADC0_Freerun_Init(bool twelveBit);
static void ADC0_Single_Init(bool twelveBit);

typedef enum {
	ADC_NOT_INITIALIZED,
	ADC_FREE_RUN_INITIALIZED,
	ADC_SINGLE_CONVERSION_INITIALIZED
	} ADC_Init_t;
	
static ADC_Init_t g_adc_initialization = ADC_NOT_INITIALIZED;


void ADC0_startConversion(void)
{
	if(g_adc_initialization != ADC_NOT_INITIALIZED)
	{
		ADC0.COMMAND = ADC_STCONV_bm; /* Start conversion */
	}
}

bool ADC0_conversionDone(void)
{
	/* Check if the conversion is done  */
	return (ADC0.INTFLAGS & ADC_RESRDY_bm);
}

int ADC0_read(void)
{
	return ADC0.RES; 	/* Reading the result also clears the interrupt flag */
}

int16_t temperatureC(void)
{
	uint16_t sigrow_offset = SIGROW.TEMPSENSE1; // Read unsigned value from signature row
	uint16_t sigrow_slope = SIGROW.TEMPSENSE0; // Read unsigned value from signature row
	static uint32_t wait = 10000;
	uint16_t adc_reading;
	int16_t temperature_in_C = -273.15;
	uint8_t holdMux;
	
	holdMux = ADC0.MUXPOS;
	ADC0.CTRLA |= ADC_ENABLE_bm /* ADC Enable: enabled */
	| ADC_RESSEL_12BIT_gc;      /* 12-bit mode */
	ADC0.MUXPOS = ADC_MUXPOS_TEMPSENSE_gc;
	ADC0_startConversion();
	
	while((!ADC0_conversionDone()) && wait--);
	
	if(wait)
	{
		adc_reading = ADC0.RES;
		int32_t temp = sigrow_offset - adc_reading;
		temp *= sigrow_slope; // Result will overflow 16-bit variable
		temp += 0x0800; // Add 4096/2 to get correct rounding on division below
		temp >>= 12; // Round off to nearest degree in Kelvin, by dividing with 2^12 (4096)
		temperature_in_C += (int16_t)temp;
	}
	
	ADC0.MUXPOS = holdMux; /* Restore ADC registers */
	
	return(temperature_in_C);
}


static void PORT_init(void)
{
	/* Disable interrupt and digital input buffer on PD0 */
	PORTD.PIN0CTRL &= ~PORT_ISC_gm;
	PORTD.PIN0CTRL |= PORT_ISC_INPUT_DISABLE_gc;
	/* Disable interrupt and digital input buffer on PD1 */
	PORTD.PIN1CTRL &= ~PORT_ISC_gm;
	PORTD.PIN1CTRL |= PORT_ISC_INPUT_DISABLE_gc;
	/* Disable interrupt and digital input buffer on PD2 */
	PORTD.PIN2CTRL &= ~PORT_ISC_gm;
	PORTD.PIN2CTRL |= PORT_ISC_INPUT_DISABLE_gc;
	/* Disable interrupt and digital input buffer on PD3 */
	PORTD.PIN3CTRL &= ~PORT_ISC_gm;
	PORTD.PIN3CTRL |= PORT_ISC_INPUT_DISABLE_gc;
	/* Disable interrupt and digital input buffer on PD4 */
	PORTD.PIN4CTRL &= ~PORT_ISC_gm;
	PORTD.PIN4CTRL |= PORT_ISC_INPUT_DISABLE_gc;
	/* Disable interrupt and digital input buffer on PD5 */
	PORTD.PIN5CTRL &= ~PORT_ISC_gm;
	PORTD.PIN5CTRL |= PORT_ISC_INPUT_DISABLE_gc;
	/* Disable interrupt and digital input buffer on PD7 */
	PORTD.PIN7CTRL &= ~PORT_ISC_gm;
	PORTD.PIN7CTRL |= PORT_ISC_INPUT_DISABLE_gc;
	
	/* Disable pull-up resistor */
	PORTD.PIN0CTRL &= ~PORT_PULLUPEN_bm;
	PORTD.PIN1CTRL &= ~PORT_PULLUPEN_bm;
	PORTD.PIN2CTRL &= ~PORT_PULLUPEN_bm;
	PORTD.PIN3CTRL &= ~PORT_PULLUPEN_bm;
	PORTD.PIN4CTRL &= ~PORT_PULLUPEN_bm;
	PORTD.PIN5CTRL &= ~PORT_PULLUPEN_bm;
	PORTD.PIN7CTRL &= ~PORT_PULLUPEN_bm;
}

static void ADC0_Freerun_Init(bool twelveBit)
{
	if(twelveBit)
	{
		VREF.ADC0REF = 0x80 |VREF_REFSEL_2V048_gc;  /* Internal 2.048V reference */
		ADC0.CTRLC = ADC_PRESC_DIV128_gc;   /* CLK_PER divided by 128 and by 13.5 (10-bit conversion time = 13889 sps */
		ADC0.CTRLA = ADC_ENABLE_bm /* ADC Enable: enabled */
		| ADC_RESSEL_12BIT_gc      /* 12-bit mode */
		| ADC_FREERUN_bm;          /* Enable Free-Run mode */
		ADC0.INTCTRL = 0x01; /* Enable interrupt */
		g_adc_initialization = ADC_FREE_RUN_INITIALIZED;
	}
	else
	{
		VREF.ADC0REF = 0x80 | VREF_REFSEL_2V048_gc;  /* Internal 2.048V reference */
		ADC0.CTRLC = ADC_PRESC_DIV128_gc;   /* CLK_PER divided by 128 and by 13.5 (10-bit conversion time = 13889 sps */
		ADC0.CTRLA = ADC_ENABLE_bm /* ADC Enable: enabled */
		| ADC_RESSEL_10BIT_gc      /* 10-bit mode */
		| ADC_FREERUN_bm;          /* Enable Free-Run mode */
		ADC0.INTCTRL = 0x01; /* Enable interrupt */
		g_adc_initialization = ADC_FREE_RUN_INITIALIZED;
	}
}

static void ADC0_Single_Init(bool twelveBit)
{
	if(twelveBit)
	{
		VREF.ADC0REF = VREF_REFSEL_2V048_gc;  /* Internal 2.048V reference */
		ADC0.CTRLC = ADC_PRESC_DIV4_gc;   /* CLK_PER divided by 128. Never set below DIV4 */
		ADC0.CTRLA |= ADC_ENABLE_bm /* ADC Enable: enabled */
		| ADC_RESSEL_12BIT_gc;      /* 12-bit mode */
		ADC0.INTCTRL &= 0xFE; /* Disable interrupt */
		g_adc_initialization = ADC_SINGLE_CONVERSION_INITIALIZED;
	}
	else
	{
		VREF.ADC0REF = VREF_REFSEL_2V048_gc;  /* Internal 2.048V reference */
		ADC0.CTRLC = ADC_PRESC_DIV4_gc;   /* CLK_PER divided by 128 and by 13.5 (10-bit conversion time = 13889 sps; Never set below DIV4 */
		ADC0.CTRLA = ADC_ENABLE_bm /* ADC Enable: enabled */
		| ADC_RESSEL_10BIT_gc;      /* 10-bit mode */
		ADC0.INTCTRL &= 0xFE; /* Disable interrupt */
		g_adc_initialization = ADC_SINGLE_CONVERSION_INITIALIZED;
	}
}

void ADC0_SYSTEM_init(bool twelveBit, bool freerun)
{
	PORT_init();
	if(freerun)
	{
		ADC0_Freerun_Init(twelveBit);
	}
	else
	{
		ADC0_Single_Init(twelveBit);
	}
	
}

void ADC0_SYSTEM_shutdown(void)
{
	ADC0.INTCTRL = 0x00; /* Disable interrupt */
	ADC0.CTRLA = ADC_RESSEL_12BIT_gc; /* Turn off ADC leaving 12-bit resolution set */
	g_adc_initialization = ADC_NOT_INITIALIZED;
}

// ISR(ADC0_RESRDY_vect)
// {
// 	/* Clear the interrupt flag by reading the result */
// 	int val = ADC0_read();
// //	LED_toggle_level();
// 	if(g_goertzel.DataPoint(val))
// 	{
// 		ADC0.INTCTRL = 0x00; /* disable ADC interrupt */
// 	}
// }