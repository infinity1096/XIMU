/*
 * led.c
 *
 *  Created on: Jul 1, 2020
 *      Author: yuche
 */

#include "led.h"
#include "stm32f1xx_hal.h"

uint8_t led_pattern = LED_DISCONNECTED;
uint8_t led_state = 0;

void led_update(){
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13,  1 - (led_pattern>>led_state) & 0x01);
	led_state++;
	led_state = led_state % 8;
}

void led_set(uint8_t led_pattern_){
	led_pattern = led_pattern_;
}
