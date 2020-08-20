/*
 * systick.c
 *
 *  Created on: Jun 15, 2020
 *      Author: yuche
 */

#include "systick.h"
#include "stm32f1xx.h"


static volatile uint32_t ticks = 0;

void systickInit(){

	   if (SysTick_Config(SystemCoreClock / SYSTICK_FREQ)){
		   while (1){ /* Error in initializing Systick */ }
	   }
}

/**
 * return milliseconds
 * @return time since systickInit in milliseconds
 */
uint32_t millis(){
	return ticks;
}

/**
 * delay milliseconds
 * @param ms milliseconds to delay
 */
void delay(uint32_t ms){
	uint32_t start = millis();
	while(millis() - start < ms){
		//do nothing
	}
}

/**
 * add this function in stm32f1xx_it
 * called by the system to update systick accordingly
 */
void systick_Inc(void) {
  ticks++;
}
