/*
 * dataProcessing.c
 *
 *  Created on: 2020年7月6日
 *      Author: yuche
 */

#include "dataProcessing.h"



/**
 * Calibrate magnetic sensor reading and convert readings from magnetic sensor
 * frame to IMU(accel and gyroscope) frame.
 */
void calibrate_and_convert_mag_reading(double m_reading[3], double m[3]){

	double m_unbias[3]; //magnatic readings minus offset
	double m_mag[3];

	m_unbias[0] = m_reading[0] + MAG_X_OFFSET;
	m_unbias[1] = m_reading[1] + MAG_Y_OFFSET;
	m_unbias[2] = m_reading[2] + MAG_Z_OFFSET;

	m_mag[0] = MAG_TRANSF_11*m_unbias[0] + MAG_TRANSF_12*m_unbias[1] + MAG_TRANSF_13*m_unbias[2];
	m_mag[1] = MAG_TRANSF_21*m_unbias[0] + MAG_TRANSF_22*m_unbias[1] + MAG_TRANSF_23*m_unbias[2];
	m_mag[2] = MAG_TRANSF_31*m_unbias[0] + MAG_TRANSF_32*m_unbias[1] + MAG_TRANSF_33*m_unbias[2];

	/**
	 * Conversion between HMC5883L and MPU9250 axis:
	 *
	 * 		Axis in HMC5883L	|	Axis in MPU9250
	 *
	 * 		+X	-------------------------	+X
	 * 		+Y	-------------------------	-Y
	 * 		+Z	-------------------------	-Z
	 *
	 */

	m[0] = m_mag[0];
	m[1] = -m_mag[1];
	m[2] = -m_mag[2];
}



