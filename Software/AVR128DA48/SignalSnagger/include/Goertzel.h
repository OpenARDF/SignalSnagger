/*
 *  MIT License
 *
 *  Copyright (c) 2021 DigitalConfections
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
 *
 *  See https://www.embedded.com/the-goertzel-algorithm
 *
 */


/* ensure this library description is only included once */
#ifndef Goertzel_h
#define Goertzel_h

#if !defined(INIT_EEPROM_ONLY) || !INIT_EEPROM_ONLY

#define MAXN 209
#define ADCCENTER 512

/* Adjustments that can be made to tune performance:
	N,
	Sampling Frequency (Sample Rate),
	Threshold,
	ADCCENTER,
	Minimum tone detection time
 */
class Goertzel
{
public:
Goertzel(float, float);
~Goertzel() = default;
Goertzel(const Goertzel&) = delete;
Goertzel& operator=(const Goertzel&) = delete;
void SetTargetFrequency(float);
bool DataPoint(int);
float Magnitude2(int *highCount);
bool SamplesReady(void);
void Flush(void);

private:
void ProcessSample(int);
void ResetGoertzel(void);

float samplingFrequency_;
float targetFrequency_;
int sampleCount_;
float coefficient_;
float q1_;
float q2_;
volatile int index_;
/* One-byte readiness is written by the ADC ISR and observed by foreground code. */
volatile bool samplesReady_;
int highValueCount_;
/* Fixed storage avoids allocation failure and heap fragmentation on the AVR. */
int testData_[MAXN];
};

#endif // !defined(INIT_EEPROM_ONLY) || !INIT_EEPROM_ONLY

#endif
