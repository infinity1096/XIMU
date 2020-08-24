/*
 * dataProcessing.h
 *
 *  Created on: 2020年7月6日
 *      Author: yuche
 */


//#define ARM_MATH_CM3 - added to preprocessor symbol.

#ifndef COMPONENTS_INC_DATAPROCESSING_H_
#define COMPONENTS_INC_DATAPROCESSING_H_


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
 * 	1.195	-0.053	0.054
 * 	-0.044	1.197	0.01
 * 	-0.026	-0.013	1.173
 *
 * 	calibrated_data = R * (raw - offset)
 *
 */

void calibrate_and_convert_mag_reading(double m_reading[3], double m[3]);

#endif /* COMPONENTS_INC_DATAPROCESSING_H_ */
