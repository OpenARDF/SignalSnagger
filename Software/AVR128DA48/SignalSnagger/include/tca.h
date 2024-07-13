/*
 * tca.h
 *
 * Created: 5/18/2024 11:14:44 AM
 *  Author: charl
 */ 


#ifndef TCA_H_
#define TCA_H_

#include <compiler.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MAX_PWM_SETTING (uint8_t)10

void TIMERA_init(void);
void setPWM(uint8_t duty);
uint8_t getPWM(void);

#ifdef __cplusplus
}
#endif
#endif /* TCA_H_ */