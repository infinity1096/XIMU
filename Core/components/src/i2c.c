/*
 * i2c.c
 *
 *  Created on: Jun 16, 2020
 *      Author: yuche
 */

#include "i2c.h"

uint8_t I2C_read(I2C_HandleTypeDef* i2cx, uint8_t target_address, uint8_t register_address, uint8_t length, uint8_t* output){
	int state = HAL_I2C_Mem_Read(i2cx,target_address << 1,register_address,sizeof(register_address),output,length,20);//timeout is magic. Do not touch
	if (state != HAL_OK){
		__NOP();
	}
	return state;
}

uint8_t I2C_write(I2C_HandleTypeDef* i2cx, uint8_t target_address, uint8_t register_address, uint8_t length, uint8_t* input){
	int state =  HAL_I2C_Mem_Write(i2cx,target_address << 1,register_address,sizeof(register_address),input,length,20);//timeout is magic. Do not touch
	if (state != HAL_OK){
			__NOP();
		}
	return state;
}


