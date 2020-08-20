/*
 * i2c.h
 *
 *  Created on: Jun 16, 2020
 *      Author: yuche
 */
#include "stm32f1xx.h"

#ifndef COMPONENTS_INC_I2C_H_
#define COMPONENTS_INC_I2C_H_


uint8_t I2C_read(I2C_HandleTypeDef* i2cx, uint8_t target_address, uint8_t register_address, uint8_t length, uint8_t* output);
uint8_t I2C_write(I2C_HandleTypeDef* i2cx, uint8_t target_address, uint8_t register_address, uint8_t length, uint8_t* input);


#endif /* COMPONENTS_INC_I2C_H_ */
