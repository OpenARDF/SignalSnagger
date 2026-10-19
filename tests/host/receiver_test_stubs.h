#ifndef RECEIVER_TEST_STUBS_H_
#define RECEIVER_TEST_STUBS_H_

#include <stdint.h>

void resetReceiverHardwareStubs(void);
void setReceiverFrequencyResult(uint8_t result);
unsigned receiverSetFrequencyCalls(void);
unsigned receiverInitializeQuadratureCalls(void);
uint32_t receiverLastFrequency(void);

#endif /* RECEIVER_TEST_STUBS_H_ */
