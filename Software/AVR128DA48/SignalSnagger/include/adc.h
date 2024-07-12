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


#ifndef ADC_H_
#define ADC_H_

#define ADC12BIT 1
#define ADC10BIT 0

typedef enum {
	ADC_AUDIO_I = ADC_MUXPOS_AIN0_gc,
	ADC_AUDIO_Q = ADC_MUXPOS_AIN1_gc,
	ADC_I_AMPED = ADC_MUXPOS_AIN2_gc,
	ADC_Q_AMPED = ADC_MUXPOS_AIN3_gc,
	ADC_ASSI_NEAR = ADC_MUXPOS_AIN4_gc,
	ADC_ASSI_FAR = ADC_MUXPOS_AIN5_gc,
	ADCBatteryVoltage = ADC_MUXPOS_AIN7_gc,
	ADCTemperature = ADC_MUXPOS_TEMPSENSE_gc,
	ADC_NONE = MAX_UINT16
} ADC_Active_Channel_t;

void ADC0_startConversion(void);
bool ADC0_conversionDone(void);
int ADC0_read(void);
int16_t temperatureC(void);
void ADC0_SYSTEM_init(bool twelveBit, bool freerun);
void ADC0_SYSTEM_shutdown(void);


#endif /* ADC_H_ */