/*
 * dataProcessing.c
 *
 *  Created on: 2020年7月6日
 *      Author: yuche
 */

#include "dataProcessing.h"
#include "string.h"
#include "stdlib.h"
#include "math.h"

void calibrate_mag_reading(){

	double m[3]; //magnatic readings minus offset

	m[0] = XIMU_sens.mx - MAG_X_OFFSET;
	m[1] = XIMU_sens.my - MAG_Y_OFFSET;
	m[2] = XIMU_sens.mz - MAG_Z_OFFSET;

	XIMU_sens.mx = MAG_TRANSF_11*m[0] + MAG_TRANSF_12*m[1] + MAG_TRANSF_13*m[2];
	XIMU_sens.my = MAG_TRANSF_21*m[0] + MAG_TRANSF_22*m[1] + MAG_TRANSF_23*m[2];
	XIMU_sens.mz = MAG_TRANSF_31*m[0] + MAG_TRANSF_32*m[1] + MAG_TRANSF_33*m[2];
}

void calc_absolute_acceleration() {

	double ax = XIMU_sens.ax,	ay = XIMU_sens.ay,	az = XIMU_sens.az;
	double q0 = XIMU_sens.q0,	q1 = XIMU_sens.q1,	q2 = XIMU_sens.q2,	q3 = XIMU_sens.q3;
	double q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3, norm_2;
	q0q0 = q0 * q0;
	q0q1 = q0 * q1;
	q0q2 = q0 * q2;
	q0q3 = q0 * q3;
	q1q1 = q1 * q1;
	q1q2 = q1 * q2;
	q1q3 = q1 * q3;
	q2q2 = q2 * q2;
	q2q3 = q2 * q3;
	q3q3 = q3 * q3;
	norm_2 = q0q0 + q1q1 + q2q2 + q3q3;

	XIMU_u.ax_abs = (ax*q0q0 + 2*az*q0q2 - 2*ay*q0q3 + ax*q1q1 + 2*ay*q1q2 + 2*az*q1q3 - ax*q2q2 - ax*q3q3)/norm_2;
	XIMU_u.ay_abs = (ay*q0q0 - 2*az*q0q1 + 2*ax*q0q3 - ay*q1q1 + 2*ax*q1q2 + ay*q2q2 + 2*az*q2q3 - ay*q3q3)/norm_2;
	XIMU_u.az_abs = (az*q0q0 + 2*ay*q0q1 - 2*ax*q0q2 - az*q1q1 + 2*ax*q1q3 - az*q2q2 + 2*ay*q2q3 + az*q3q3)/norm_2;
}

void normalize2(double v[3]){
	double norm = sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
	v[0] /= norm;
	v[1] /= norm;
	v[2] /= norm;
}

void calc_magnetic_orientation(){

	double g[3];
	g[0] = -XIMU_sens.ax;
	g[1] = -XIMU_sens.ay;
	g[2] = -XIMU_sens.az;

	normalize(g);

	/**
	 * Conversion between AK8963 and MPU9250 axis:
	 *
	 * 		Axis in AK8963	|	Axis in MPU9250
	 *
	 * 		+X	-------------------------	+Y
	 * 		+Y	-------------------------	+X
	 * 		+Z	-------------------------	-Z
	 *
	 */

	double m[3];
	m[0] = XIMU_sens.my;
	m[1] = XIMU_sens.mx;
	m[2] = -XIMU_sens.mz;

	normalize2(m);

	//step1: cross gravity(negative acceleration) and magnetic field to get reference direction of East
	double East_ref[3] = {0};

	East_ref[0] = g[1]*m[2] - g[2]*m[1];
	East_ref[1] = g[2]*m[0] - g[0]*m[2];
	East_ref[2] = g[0]*m[1] - g[1]*m[0];

	normalize2(East_ref);

	//step2: cross East and gravity(negative acceleration) to get reference direction of North
	double North_ref[3] = {0};

	North_ref[0] = East_ref[1]*g[2] - East_ref[2]*g[1];
	North_ref[1] = East_ref[2]*g[0] - East_ref[0]*g[2];
	North_ref[2] = East_ref[0]*g[1] - East_ref[1]*g[0];

	normalize2(North_ref);

	//optional: rotate north and east according to magnetic field inclination

	//step3: calculate quaternion from East_ref, North_ref, minus gravity(UP)
	double m00,m10,m20,m01,m11,m21,m02,m12,m22;

	//the following is the TRANSPOSED rotation matrix.
	//to get the rotation matrix we normally use, the one that
	//store basis vectors as columns, do a transpose.
	m00 = East_ref[0];	m01 = East_ref[1];	m02 = East_ref[2];
	m10 = North_ref[0];	m11 = North_ref[1];	m12 = North_ref[2];
	m20 = -g[0];		m21 = -g[1];		m22 = -g[2];

	//conversion from transposed rotation matrix to quaternion
	double t;
	if (m22 < 0){
		if (m00 > m11){
			t = 1 + m00 - m11 - m22;

			XIMU_observ.mag_ref_q0 = m12-m21;
			XIMU_observ.mag_ref_q1 = t;
			XIMU_observ.mag_ref_q2 = m01+m10;
			XIMU_observ.mag_ref_q3 = m20+m02;
		}else{
			t = 1 - m00 + m11 - m22;

			XIMU_observ.mag_ref_q0 = m20-m02;
			XIMU_observ.mag_ref_q1 = m01+m10;
			XIMU_observ.mag_ref_q2 = t;
			XIMU_observ.mag_ref_q3 = m12+m21;
		}
	 }else{
		 if (m00 < -m11){
			 t = 1 - m00 - m11 + m22;

			 XIMU_observ.mag_ref_q0 = m01-m10;
			 XIMU_observ.mag_ref_q1 = m20+m02;
			 XIMU_observ.mag_ref_q2 = m12+m21;
			 XIMU_observ.mag_ref_q3 = t;
		 }else{
			 t = 1 + m00 + m11 + m22;

			 XIMU_observ.mag_ref_q0 = t;
			 XIMU_observ.mag_ref_q1 = m12-m21;
			 XIMU_observ.mag_ref_q2 = m20-m02;
			 XIMU_observ.mag_ref_q3 = m01-m10;
		 }
	 }

	//normalize
	XIMU_observ.mag_ref_q0 *= 0.5f / sqrt(t);
	XIMU_observ.mag_ref_q1 *= 0.5f / sqrt(t);
	XIMU_observ.mag_ref_q2 *= 0.5f / sqrt(t);
	XIMU_observ.mag_ref_q3 *= 0.5f / sqrt(t);
}

void calc_absolute_position();
void calc_absolute_altitude();

void build_data_str(char* str){

	char temp[30];
	char delim[2] = {'\t','\0'};
	char term[2] = {'\n','\0'};


	itoa((int)(XIMU_sens.q0),temp,10);
	strcpy(str,temp);
	strcat(str,delim);

	itoa((int)(XIMU_sens.q1),temp,10);
	strcat(str,temp);
	strcat(str,delim);

	itoa((int)(XIMU_sens.q2),temp,10);
	strcat(str,temp);
	strcat(str,delim);

	itoa((int)(XIMU_sens.q3),temp,10);
	strcat(str,temp);
	strcat(str,delim);

	itoa((int)(XIMU_sens.ax*1000),temp,10);
	strcat(str,temp);
	strcat(str,delim);

	itoa((int)(XIMU_sens.ay*1000),temp,10);
	strcat(str,temp);
	strcat(str,delim);

	itoa((int)(XIMU_sens.az*1000),temp,10);
	strcat(str,temp);
	strcat(str,delim);

	itoa((int)(XIMU_sens.gx*1000),temp,10);
	strcat(str,temp);
	strcat(str,delim);

	itoa((int)(XIMU_sens.gy*1000),temp,10);
	strcat(str,temp);
	strcat(str,delim);

	itoa((int)(XIMU_sens.gz*1000),temp,10);
	strcat(str,temp);
	strcat(str,delim);

	itoa((int)(XIMU_sens.mx*1000),temp,10);
	strcat(str,temp);
	strcat(str,delim);

	itoa((int)(XIMU_sens.my*1000),temp,10);
	strcat(str,temp);
	strcat(str,delim);

	itoa((int)(XIMU_sens.mz*1000),temp,10);
	strcat(str,temp);
	strcat(str,delim);

	itoa((int)(XIMU_sens.pressure*1000),temp,10);
	strcat(str,temp);
	strcat(str,delim);

	itoa((int)(XIMU_sens.temperature*1000),temp,10);
	strcat(str,temp);
	strcat(str,delim);

	itoa((int)(XIMU_sens.lat*1000000),temp,10);
	strcat(str,temp);
	strcat(str,delim);

	itoa((int)(XIMU_sens.lon*1000000),temp,10);
	strcat(str,temp);
	strcat(str,delim);

	itoa((int)(XIMU_sens.qag_ts),temp,10);
	strcat(str,temp);
	strcat(str,delim);

	itoa((int)(XIMU_sens.m_ts),temp,10);
	strcat(str,temp);
	strcat(str,delim);

	itoa((int)(XIMU_sens.pt_ts),temp,10);
	strcat(str,temp);
	strcat(str,delim);

	itoa((int)(XIMU_sens.gps_ts),temp,10);
	strcat(str,temp);
	strcat(str,term);
}
