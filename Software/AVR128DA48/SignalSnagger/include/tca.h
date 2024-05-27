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

void TIMERA_init(void);
uint8_t setPWM(uint8_t duty);

#ifdef __cplusplus
}
#endif
#endif /* TCA_H_ */