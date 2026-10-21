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
#include "audio_path.h"
#include "Goertzel.h"

#define ADC_CONVERSION_SPIN_GUARD UINT32_C(100000)

Goertzel g_goertzel(AUDIO_GOERTZEL_SAMPLE_COUNT, AUDIO_DSP_SAMPLE_RATE_HZ);

static void PORT_init(void);
static void ADC0_Audio_Init(bool twelveBit);
static void ADC0_Single_Init(bool twelveBit);
static void ADC0_AudioTimerStart(void);
static void ADC0_AudioTimerStop(void);

typedef enum {
	ADC_NOT_INITIALIZED,
	ADC_AUDIO_INITIALIZED,
	ADC_SINGLE_CONVERSION_INITIALIZED
	} ADC_Init_t;
	
static ADC_Init_t g_adc_initialization = ADC_NOT_INITIALIZED;


void ADC0_startConversion(void)
{
	if(g_adc_initialization == ADC_SINGLE_CONVERSION_INITIALIZED)
	{
		ADC0.COMMAND = ADC_STCONV_bm; /* Start conversion */
	}
}

bool ADC0_conversionDone(void)
{
	/* Check if the conversion is done  */
	return (ADC0.INTFLAGS & ADC_RESRDY_bm);
}

bool ADC0_waitForConversion(void)
{
	uint32_t spinGuard = ADC_CONVERSION_SPIN_GUARD;
	while(!ADC0_conversionDone() && spinGuard)
	{
		spinGuard--;
	}

	return ADC0_conversionDone();
}

int ADC0_read(void)
{
	return ADC0.RES; 	/* Reading the result also clears the interrupt flag */
}

int16_t temperatureC(void)
{
	uint16_t sigrow_offset = SIGROW.TEMPSENSE1; // Read unsigned value from signature row
	uint16_t sigrow_slope = SIGROW.TEMPSENSE0; // Read unsigned value from signature row
	uint16_t adc_reading;
	int16_t temperature_in_C = -273.15;
	uint8_t holdMux;
	
	holdMux = ADC0.MUXPOS;
	ADC0.CTRLA |= ADC_ENABLE_bm /* ADC Enable: enabled */
	| ADC_RESSEL_12BIT_gc;      /* 12-bit mode */
	ADC0.MUXPOS = ADC_MUXPOS_TEMPSENSE_gc;
	ADC0_startConversion();
	
	if(ADC0_waitForConversion())
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

static void ADC0_AudioTimerStop(void)
{
	TCA1.SINGLE.INTCTRL = 0;
	TCA1.SINGLE.CTRLA = 0;
	ADC0.EVCTRL = 0;
	EVSYS.USERADC0START = 0;
	EVSYS.CHANNEL0 = 0;
	CPUINT.LVL1VEC = TCB2_INT_vect_num;
}

static void ADC0_AudioTimerStart(void)
{
#if (F_CPU % AUDIO_ADC_SAMPLE_RATE_HZ) != 0
#error F_CPU must divide evenly into the requested ADC sample rate
#endif
	TCA1.SINGLE.CTRLA = 0;
	TCA1.SINGLE.CTRLB = TCA_SINGLE_WGMODE_NORMAL_gc;
	TCA1.SINGLE.CNT = 0;
	TCA1.SINGLE.PER = (F_CPU / AUDIO_ADC_SAMPLE_RATE_HZ) - 1;
	/* Start lower-priority DSP after the ADC conversion and capture ISR. */
	TCA1.SINGLE.CMP0 = ((F_CPU / AUDIO_ADC_SAMPLE_RATE_HZ) * 4 / 5);
	TCA1.SINGLE.INTCTRL = 0;
	TCA1.SINGLE.INTFLAGS = TCA_SINGLE_OVF_bm | TCA_SINGLE_CMP0_bm |
		TCA_SINGLE_CMP1_bm | TCA_SINGLE_CMP2_bm;

	EVSYS.CHANNEL0 = EVSYS_CHANNEL0_TCA1_OVF_LUNF_gc;
	EVSYS.USERADC0START = EVSYS_USER_CHANNEL0_gc;
	ADC0.EVCTRL = ADC_STARTEI_bm;
	CPUINT.LVL1VEC = ADC0_RESRDY_vect_num;
	TCA1.SINGLE.CTRLA = TCA_SINGLE_CLKSEL_DIV1_gc | TCA_SINGLE_ENABLE_bm;
}

void ADC0_scheduleAudioFrame(void)
{
	/* Clear any stale compare and arm one low-priority DSP pass at the current
	 * or next compare point; the ADC remains the level-1 interrupt. */
	TCA1.SINGLE.INTFLAGS = TCA_SINGLE_CMP0_bm;
	TCA1.SINGLE.INTCTRL |= TCA_SINGLE_CMP0_bm;
}

void ADC0_audioFrameHandled(void)
{
	TCA1.SINGLE.INTCTRL &= (uint8_t)~TCA_SINGLE_CMP0_bm;
	TCA1.SINGLE.INTFLAGS = TCA_SINGLE_CMP0_bm;
}

static void ADC0_Audio_Init(bool twelveBit)
{
	VREF.ADC0REF = VREF_ALWAYSON_bm | VREF_REFSEL_2V048_gc;
	ADC0.CTRLC = ADC_PRESC_DIV16_gc;
	if(twelveBit)
	{
		ADC0.CTRLA = ADC_ENABLE_bm /* ADC Enable: enabled */
			| ADC_RESSEL_12BIT_gc;    /* 12-bit mode */
	}
	else
	{
		ADC0.CTRLA = ADC_ENABLE_bm /* ADC Enable: enabled */
			| ADC_RESSEL_10BIT_gc;    /* 10-bit mode */
	}
	ADC0.INTFLAGS = ADC_RESRDY_bm | ADC_WCMP_bm;
	ADC0.INTCTRL = ADC_RESRDY_bm;
	g_adc_initialization = ADC_AUDIO_INITIALIZED;
	/* TCA1 overflow starts each conversion through EVSYS, giving an exact
	 * 48 ksample/s cadence without a second timer interrupt. */
	ADC0_AudioTimerStart();
}

static void ADC0_Single_Init(bool twelveBit)
{
	if(twelveBit)
	{
		VREF.ADC0REF = VREF_REFSEL_2V048_gc;  /* Internal 2.048V reference */
		ADC0.CTRLC = ADC_PRESC_DIV128_gc;   /* CLK_PER divided by 128. Never set below DIV4 */
		ADC0.CTRLA |= ADC_ENABLE_bm /* ADC Enable: enabled */
		| ADC_RESSEL_12BIT_gc;      /* 12-bit mode */
		ADC0.INTCTRL &= 0xFE; /* Disable interrupt */
		g_adc_initialization = ADC_SINGLE_CONVERSION_INITIALIZED;
	}
	else
	{
		VREF.ADC0REF = VREF_REFSEL_2V048_gc;  /* Internal 2.048V reference */
		ADC0.CTRLC = ADC_PRESC_DIV128_gc;   /* CLK_PER divided by 128 and by 13.5 (10-bit conversion time = 13889 sps; Never set below DIV4 */
		ADC0.CTRLA = ADC_ENABLE_bm /* ADC Enable: enabled */
		| ADC_RESSEL_10BIT_gc;      /* 10-bit mode */
		ADC0.INTCTRL &= 0xFE; /* Disable interrupt */
		g_adc_initialization = ADC_SINGLE_CONVERSION_INITIALIZED;
	}
}

void ADC0_SYSTEM_init(bool twelveBit, bool freerun)
{
	ADC0_SYSTEM_shutdown();
	PORT_init();
	if(freerun)
	{
		ADC0_Audio_Init(twelveBit);
	}
	else
	{
		ADC0_Single_Init(twelveBit);
	}
	
}

void ADC0_SYSTEM_shutdown(void)
{
	ADC0.INTCTRL = 0x00; /* Disable interrupt */
	ADC0_AudioTimerStop();
	ADC0.CTRLA = ADC_RESSEL_12BIT_gc; /* Turn off ADC leaving 12-bit resolution set */
	ADC0.INTFLAGS = ADC_RESRDY_bm | ADC_WCMP_bm;
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
