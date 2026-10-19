#ifndef TEST_RECEIVER_H_
#define TEST_RECEIVER_H_

#include <stdint.h>

#include "receiver_frequency.h"

typedef uint32_t Frequency_Hz;
typedef uint8_t EC;
typedef uint8_t AntConnType;

enum {
	ERROR_CODE_NO_ERROR = 0x00,
	ERROR_CODE_SW_LOGIC_ERROR = 0xcf,
	ERROR_CODE_RF_OSCILLATOR_ERROR = 0xd2,
	ERROR_CODE_CLKGEN_NONRESPONSIVE = 0xfb
};

enum {
	SI5351_CRYSTAL_LOAD_6PF = 0,
	SI5351_CLK0 = 0,
	SI5351_CLK1 = 1,
	SI5351_DRIVE_8MA = 8,
	SI5351_CLK_DISABLED = 0
};

#define EEPROM_TX_80M_FREQUENCY_DEFAULT UINT32_C(3550000)
#define RX_MINIMUM_80M_FREQUENCY RECEIVER_FREQUENCY_MIN_HZ
#define RX_MAXIMUM_80M_FREQUENCY RECEIVER_FREQUENCY_MAX_HZ
#define NUMBER_OF_FREQUENCY_CHANNELS 20

bool si5351_init(uint8_t crystalLoad, int32_t correction);
EC si5351_drive_strength(uint8_t clock, uint8_t drive);
EC si5351_clock_enable(uint8_t clock, uint8_t enabled);
EC si5351_init_for_quad(Frequency_Hz frequencyHz);
EC si5351_set_quad_frequency(Frequency_Hz frequencyHz);
void si5351_shutdown(void);
void si5351_shutdown_comms(void);
void si5351_start_comms(void);
void DAC0_init(void);

void shutdown_receiver(void);
void restart_receiver(void);
EC init_receiver(void);
EC init_receiver(Frequency_Hz frequencyHz);
bool rxIsInitialized(void);
EC rxSetParameters(void);
bool rxSetFrequency(Frequency_Hz* frequencyHz, bool leaveClockOff);
bool rxTuneMemoryStep(Frequency_Hz* receiverFrequency,
	Frequency_Hz* memoryFrequency,
	int8_t direction,
	uint32_t stepHz);
Frequency_Hz rxGetFrequencty(void);

#endif /* TEST_RECEIVER_H_ */
