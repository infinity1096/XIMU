/*
 * ms5611.h
 *
 *  Created on: 2020年6月17日
 *      Author: yuche
 */
#include <stdint.h>
#include "stm32f1xx.h"

#ifndef COMPONENTS_INC_MS5611_H_
#define COMPONENTS_INC_MS5611_H_

#define MS5611_I2C_ADDR			0x77	//when pin csb is low
#define MS5611_I2C_ADDR2		0x76	//when pin csb is high

#define MS5611_CMD_ADC_READ		0x00
#define MS5611_CMD_CONVERT_D1	0x40	//convert pressure
#define MS5611_CMD_CONVERT_D2	0x50	//convert temperature
#define MS5611_CMD_RESET		0x1E
#define MS5611_CMD_READ_PROM	0xA2	//6 calibration values are stored from 0xA2 to 0xAC with interval 2

enum MS5611_OSR {
	MS5611_OSR_256 = 0,
	MS5611_OSR_512,
	MS5611_OSR_1024,
	MS5611_OSR_2048,
	MS5611_OSR_4096,
};

I2C_HandleTypeDef* ms5611_i2cx;

void ms5611_set_i2c(I2C_HandleTypeDef* i2cx);

uint8_t ms5611_read_i2c(uint8_t register_address,uint8_t length,uint8_t* output);
uint8_t ms5611_write_i2c(uint8_t register_address,uint8_t length,uint8_t* input);

void ms5611_osr_select(enum MS5611_OSR osr);

void ms5611_init();

void ms5611_request_pressure();
void ms5611_retrieve_pressure();
void ms5611_request_temperature();
void ms5611_retrieve_temperature();

void ms5611_update_pressure();
void ms5611_update_temperature();

void ms5611_update();

double ms5611_get_temperature();
double ms5611_get_pressure();

#endif /* COMPONENTS_INC_MS5611_H_ */
