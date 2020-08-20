/*
 * led.h
 *
 *  Created on: Jul 1, 2020
 *      Author: yuche
 */

#include "stdint.h"

#ifndef COMPONENTS_INC_LED_H_
#define COMPONENTS_INC_LED_H_

#define LED_DISCONNECTED 	0b00000000
#define LED_IDLE 			0b11111111		//connected
#define LED_MEASURING 		0b11111110
#define LED_FAULT			0b10101010
#define LED_INIT			0b11001100

void led_update();
void led_set(uint8_t);

#endif /* COMPONENTS_INC_LED_H_ */
