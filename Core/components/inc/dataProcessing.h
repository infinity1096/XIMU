/*
 * dataProcessing.h
 *
 *  Created on: 2020年7月6日
 *      Author: yuche
 */


//#define ARM_MATH_CM3 - added to preprocessor symbol.

#ifndef COMPONENTS_INC_DATAPROCESSING_H_
#define COMPONENTS_INC_DATAPROCESSING_H_

#include "stdint.h"

#define DATA_ENTRY_SIZE 30

typedef struct{

	double q0;
	double q1;
	double q2;
	double q3;

	double ax;
	double ay;
	double az;

	double gx;
	double gy;
	double gz;
	uint32_t qag_ts;//time stamp of quaternion, acceleration and gyration

	double mx;
	double my;
	double mz;
	uint32_t m_ts;//time stamp of magnetic field

	double pressure;
	double temperature;
	uint32_t pt_ts;//time stamp of pressure
	//temperature is indeed sampled differently. However, we can assume the value doesn't change too fast.
	//for simplicity I'm going to combine their times tamp.

	double lat;
	double lon;
	uint32_t gps_ts;//time stamp for GPS data

} XIMU_sens_t;

XIMU_sens_t XIMU_sens;

//the following calibration is obtained using
//https://www.instructables.com/id/Easy-hard-and-soft-iron-magnetometer-calibration/

#define MAG_X_OFFSET 138.737
#define MAG_Y_OFFSET 262.501
#define MAG_Z_OFFSET -73.638

#define MAG_TRANSF_11 1.195
#define MAG_TRANSF_12 -0.053
#define MAG_TRANSF_13 0.054
#define MAG_TRANSF_21 -0.044
#define MAG_TRANSF_22 1.197
#define MAG_TRANSF_23 0.01
#define MAG_TRANSF_31 -0.026
#define MAG_TRANSF_32 -0.013
#define MAG_TRANSF_33 1.173

/**
 * magnatic field transformation matrix:
 *
 * 	1.636	-0.066	0.005
 * 	-0.052	1.616	0
 * 	-0.02	-0.007	1.67
 *
 * 	calibrated_data = R * (raw - offset)
 *
 */

//this number indicates the error between true north and magnetic north.
//negative number indicates magnetic north is west of true north
//Beijing: -7.18	San Diego: +11.31	Utah:+11.6
#define MAG_DECLINATION -7.18

typedef struct{

	double ax_abs;
	double ay_abs;
	double az_abs;

} XIMU_u_t;

XIMU_u_t XIMU_u;

typedef struct{

	double mag_ref_q0;
	double mag_ref_q1;
	double mag_ref_q2;
	double mag_ref_q3;

	double x;
	double y;
	double z;

} XIMU_observ_t;

XIMU_observ_t XIMU_observ;

typedef struct{

	//rotation from magnetic refrence to DMP output
	double dq0;
	double dq1;
	double dq2;
	double dq3;

	//translation from position origin to current position
	//in East-North-Up frame
	double x;
	double y;
	double z;

	//velocity in East-North-Up frame
	double vx;
	double vy;
	double vz;

} XIMU_state;

void calibrate_mag_reading();

void calc_absolute_acceleration();

void calc_magnetic_orientation();

void build_data_str(char*);

#endif /* COMPONENTS_INC_DATAPROCESSING_H_ */
