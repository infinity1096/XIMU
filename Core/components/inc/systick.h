/*
 * systick.h
 *
 *  Created on: Jun 15, 2020
 *      Author: yuche
 */
#include <stdint.h>

#ifndef COMPONENTS_INC_SYSTICK_H_
#define COMPONENTS_INC_SYSTICK_H_

#define SYSTICK_FREQ 1000 //1kHz

void systickInit();

uint32_t millis();

void delay(uint32_t ms);

void systick_Inc(void);

#endif /* COMPONENTS_INC_SYSTICK_H_ */
