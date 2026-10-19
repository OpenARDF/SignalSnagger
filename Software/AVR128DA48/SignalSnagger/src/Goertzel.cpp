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

#include "Goertzel.h"
#include <math.h>
#include <string.h>

#if !defined(INIT_EEPROM_ONLY) || !INIT_EEPROM_ONLY

namespace {

uint8_t boundedSampleCount(float requested)
{
	if (requested < MINN)
	{
		return MINN;
	}
	if (requested > MAXN)
	{
		return MAXN;
	}
	return (uint8_t)requested;
}

} // namespace

Goertzel::Goertzel(float N, float sampling_frequency) :
	samplingFrequency_(sampling_frequency),
	targetFrequency_(0),
	sampleCount_(boundedSampleCount(N)),
	coefficient_(0),
	q1_(0),
	q2_(0),
	captureIndex_(0),
	captureBuffer_(0),
	readyBuffer_(0),
	samplesReady_(false),
	highValueCount_(0)
{
	Flush();
}

void Goertzel::Flush(void)
{
	memset(testData_, 0x00, sizeof(testData_));
	captureIndex_ = 0;
	captureBuffer_ = 0;
	readyBuffer_ = 0;
	samplesReady_ = false;
	ResetAnalysis();
}

void Goertzel::SetTargetFrequency(float target_frequency)
{
	float floatN = (float)sampleCount_;

	targetFrequency_ = target_frequency;   /* Ideally, an integer multiple of sample rate/N. */
	int k = (int)(0.5 + ((floatN * targetFrequency_) / samplingFrequency_));
	const double pi = 3.141592653589793;
	float w = (2.0 * pi * k) / floatN;
	coefficient_ = 2.0 * cos(w);
	ResetAnalysis();
}

void Goertzel::ResetAnalysis(void)
{
	q2_ = 0;
	q1_ = 0;
}

/* Call this routine for every sample. */
void Goertzel::ProcessSample(int sample)
{
	float q0 = coefficient_ * q1_ - q2_ + (float)(sample - ADCCENTER);

	if(sample > SATURATION)
	{
		highValueCount_++;
	}

	q2_ = q1_;
	q1_ = q0;
}

void Goertzel::PublishCapturedBlock(void)
{
	if (captureIndex_ < sampleCount_ || samplesReady_)
	{
		return;
	}

	readyBuffer_ = captureBuffer_;
	captureBuffer_ ^= 1;
	captureIndex_ = 0;
	/* Publish last so foreground never sees partially initialized ownership. */
	samplesReady_ = true;
}

bool Goertzel::DataPoint(int data)

{
	/* Publish a completed pending buffer after foreground releases the prior one. */
	PublishCapturedBlock();

	if (captureIndex_ < sampleCount_)
	{
		testData_[captureBuffer_][captureIndex_++] = data;
	}
	PublishCapturedBlock();

	return samplesReady_;
}

bool Goertzel::SamplesReady(void)
{
	return samplesReady_;
}

void Goertzel::ReleaseSamples(void)
{
	/* A bool is one byte on AVR; the ISR owns every other capture-state write. */
	samplesReady_ = false;
}

float Goertzel::Magnitude2(int *highCount)
{
	float magnitude2;

	if (!samplesReady_)
	{
		if (highCount) *highCount = 0;
		return 0;
	}

	highValueCount_ = 0;
	ResetAnalysis();
	uint8_t buffer = readyBuffer_;

	/* Process the samples. */
	for(uint8_t index = 0; index < sampleCount_; index++)
	{
		ProcessSample(testData_[buffer][index]);
	}

	if(highCount) *highCount = highValueCount_;

	magnitude2 = q1_ * q1_ + q2_ * q2_ - coefficient_ * q1_ * q2_;

	return( magnitude2);
}
#endif // !defined(INIT_EEPROM_ONLY) || !INIT_EEPROM_ONLY
