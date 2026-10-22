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
	sampleCount_(boundedSampleCount(N)),
	targetCount_(1),
	captureIndex_(0),
	captureBuffer_(0),
	readyBuffer_(0),
	samplesReady_(false),
	highValueCount_(0)
{
	memset(coefficients_, 0x00, sizeof(coefficients_));
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
	SetTargetFrequencies(&target_frequency, 1);
}

void Goertzel::SetTargetFrequencies(const float* targetFrequencies,
	uint8_t targetCount)
{
	if (!targetFrequencies || targetCount == 0)
	{
		return;
	}
	if (targetCount > MAX_GOERTZEL_TARGETS)
	{
		targetCount = MAX_GOERTZEL_TARGETS;
	}
	targetCount_ = targetCount;

	const float floatN = (float)sampleCount_;
	const double pi = 3.141592653589793;
	for (uint8_t target = 0; target < targetCount_; ++target)
	{
		/* Exact sample-rate/N bin centers avoid spectral leakage. */
		const int k = (int)(0.5 + ((floatN * targetFrequencies[target]) /
			samplingFrequency_));
		const float w = (float)((2.0 * pi * k) / floatN);
		coefficients_[target] = 2.0f * cosf(w);
	}
	ResetAnalysis();
}

void Goertzel::ResetAnalysis(void)
{
	memset(q1_, 0x00, sizeof(q1_));
	memset(q2_, 0x00, sizeof(q2_));
}

/* Call this routine for every sample. */
void Goertzel::ProcessSample(int sample)
{
	if(sample > SATURATION)
	{
		highValueCount_++;
	}

	const float centered = (float)(sample - ADCCENTER);
	for (uint8_t target = 0; target < targetCount_; ++target)
	{
		const float q0 = coefficients_[target] * q1_[target] -
			q2_[target] + centered;
		q2_[target] = q1_[target];
		q1_[target] = q0;
	}
}

bool Goertzel::DataPoint(int data)

{
	/* If both buffers are occupied, hold the completed capture until foreground
	 * releases the published block. No sample can overwrite either owner. */
	if (captureIndex_ >= sampleCount_)
	{
		if (samplesReady_)
		{
			return true;
		}
		readyBuffer_ = captureBuffer_;
		captureBuffer_ ^= 1;
		captureIndex_ = 0;
		samplesReady_ = true;
	}

	testData_[captureBuffer_][captureIndex_++] = data;
	if (captureIndex_ >= sampleCount_ && !samplesReady_)
	{
		readyBuffer_ = captureBuffer_;
		captureBuffer_ ^= 1;
		captureIndex_ = 0;
		/* Publish last so foreground never sees partial ownership state. */
		samplesReady_ = true;
	}

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
	float magnitude = 0;
	(void)Magnitudes2(&magnitude, 1, highCount);
	return magnitude;
}

uint8_t Goertzel::Magnitudes2(float* magnitudes, uint8_t capacity,
	int *highCount)
{
	if (!magnitudes || capacity == 0)
	{
		if (highCount) *highCount = 0;
		return 0;
	}

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

	const uint8_t resultCount = (capacity < targetCount_) ? capacity : targetCount_;
	for (uint8_t target = 0; target < resultCount; ++target)
	{
		magnitudes[target] = q1_[target] * q1_[target] +
			q2_[target] * q2_[target] - coefficients_[target] *
			q1_[target] * q2_[target];
	}
	return resultCount;
}
#endif // !defined(INIT_EEPROM_ONLY) || !INIT_EEPROM_ONLY
