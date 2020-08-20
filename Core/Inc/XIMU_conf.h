/*
 * XIMU_conf.h
 *
 *  Created on: 2020年8月19日
 *      Author: yuche
 */

#ifndef INC_XIMU_CONF_H_
#define INC_XIMU_CONF_H_

typedef enum {
	//Normal states
	XIMU_STATE_STM_INIT,			//STM32 Initialization after powered on
	XIMU_STATE_DMP_INIT,			//Download & verify DMP firmware
	XIMU_STATE_GPS_INIT,			//Wait for GPS module to return valid data
	XIMU_STATE_FILTER_INIT,			//Collect enough sensor readings to initialize ESKF filter
	XIMU_STATE_FILTER_FUSE,			//Fusing data and updating state variables, normal working state

	//Error handeling states

	//external sensor error
	XIMU_STATE_ERROR_IMU_UNDERRUN,	//Accel / gyro 		data significantly below sample rate
	XIMU_STATE_ERROR_MAG_UNDERRUN,	//Mag 				data significantly below sample rate
	XIMU_STATE_ERROR_DMP_UNDERRUN,	//DMP(quaternion) 	data significantly below sample rate
	XIMU_STATE_ERROR_GPS_UNDERRUN,	//GPS 				data significantly below sample rate

	//communication error
	XIMU_STATE_ERROR_I2C,			//I2C bus error

} XIMU_STATE;

#endif /* INC_XIMU_CONF_H_ */
