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

#include <math.h>
#include <stdint.h>

#define MAXN 209
#define MINN 21
#define ADCCENTER 2048
#define SATURATION (9*ADCCENTER/5)
#define MAX_GOERTZEL_TARGETS 3

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
void SetTargetFrequencies(const float*, uint8_t);
bool DataPoint(int);
float Magnitude2(int *highCount);
uint8_t Magnitudes2(float*, uint8_t, int *highCount);
bool SamplesReady(void);
/** Release the analyzed block so the ISR can publish the next completed block. */
void ReleaseSamples(void);
void Flush(void);

private:
void ProcessSample(int);
void ResetAnalysis(void);

float samplingFrequency_;
uint8_t sampleCount_;
uint8_t targetCount_;
float coefficients_[MAX_GOERTZEL_TARGETS];
float q1_[MAX_GOERTZEL_TARGETS];
float q2_[MAX_GOERTZEL_TARGETS];
volatile uint8_t captureIndex_;
volatile uint8_t captureBuffer_;
volatile uint8_t readyBuffer_;
/* One-byte readiness is written by the ADC ISR and foreground code. */
volatile bool samplesReady_;
int highValueCount_;
/* Two buffers keep the published block immutable during foreground analysis. */
int testData_[2][MAXN];
};

#endif // !defined(INIT_EEPROM_ONLY) || !INIT_EEPROM_ONLY

#endif
