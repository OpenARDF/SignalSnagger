#include "receiver.h"
#include "receiver_test_stubs.h"

namespace {

EC setFrequencyResult = ERROR_CODE_NO_ERROR;
unsigned setFrequencyCallCount = 0;
unsigned initializeQuadratureCallCount = 0;
uint32_t lastFrequency = 0;

} // namespace

void resetReceiverHardwareStubs(void)
{
	setFrequencyResult = ERROR_CODE_NO_ERROR;
	setFrequencyCallCount = 0;
	initializeQuadratureCallCount = 0;
	lastFrequency = 0;
}

void setReceiverFrequencyResult(uint8_t result)
{
	setFrequencyResult = result;
}

unsigned receiverSetFrequencyCalls(void)
{
	return setFrequencyCallCount;
}

unsigned receiverInitializeQuadratureCalls(void)
{
	return initializeQuadratureCallCount;
}

uint32_t receiverLastFrequency(void)
{
	return lastFrequency;
}

bool si5351_init(uint8_t, int32_t)
{
	return false;
}

EC si5351_drive_strength(uint8_t, uint8_t)
{
	return ERROR_CODE_NO_ERROR;
}

EC si5351_clock_enable(uint8_t, uint8_t)
{
	return ERROR_CODE_NO_ERROR;
}

EC si5351_init_for_quad(Frequency_Hz frequencyHz)
{
	++initializeQuadratureCallCount;
	lastFrequency = frequencyHz;
	return setFrequencyResult;
}

EC si5351_set_quad_frequency(Frequency_Hz frequencyHz)
{
	++setFrequencyCallCount;
	lastFrequency = frequencyHz;
	return setFrequencyResult;
}

void si5351_shutdown(void) {}
void si5351_shutdown_comms(void) {}
void si5351_start_comms(void) {}
void DAC0_init(void) {}
