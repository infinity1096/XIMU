/*
 * EKF.h
 *
 *  Created on: 2020年7月7日
 *      Author: yuche
 */

#ifndef COMPONENTS_INC_ESKF_H_
#define COMPONENTS_INC_ESKF_H_

#include "arm_math.h"

/**
 *                        STATE VARIABLE DEFINITION
 * 	State       true        nominal     error       dimension
 * 	position    p_t         p           del_p       3
 *  velocity    v_t         v           del_v       3
 *  rotation    q_t         q           del_q       4
 *  angles vec                          del_theta   3
 *  acc bias    ab_t        ab          del_ab      3
 *  gyro bias   wb_t        wb          del_wb      3
 *  gravity     g_t         g           del_g       3
*/

typedef struct{

	//initialization status
	int IMU_initialized;
	int MAG_initialized;
	int GPS_initialized;

	//nominal state
	arm_matrix_instance_f32 p;
	float32_t p_data[3*1];

	arm_matrix_instance_f32 v;
	float32_t v_data[3*1];

	arm_matrix_instance_f32 q;
	float32_t q_data[4*1];

	arm_matrix_instance_f32 R;
	float32_t R_data[3*3];

	arm_matrix_instance_f32 ab;
	float32_t ab_data[3*1];

	arm_matrix_instance_f32 wb;
	float32_t wb_data[3*1];

	//error state
	arm_matrix_instance_f32 del_p;
	float32_t del_p_data[3*1];

	arm_matrix_instance_f32 del_v;
	float32_t del_v_data[3*1];

	arm_matrix_instance_f32 del_theta;//note: this is not "del_q"!
	float32_t del_theta_data[3*1];

	arm_matrix_instance_f32 del_ab;
	float32_t del_ab_data[3*1];

	arm_matrix_instance_f32 del_wb;
	float32_t del_wb_data[3*1];

	//error state covariance
	arm_matrix_instance_f32 P;
	float32_t P_data[15*15];

	//process noise covariance
	arm_matrix_instance_f32 Q;
	float32_t Q_data[12*12];

	//observer noise covariance
	arm_matrix_instance_f32 V_GPS;
	float32_t V_GPS_data[3*3];

	arm_matrix_instance_f32 V_MAG;
	float32_t V_MAG_data[3*3];

	//state transfer matrix
	arm_matrix_instance_f32 Fx;
	float32_t Fx_data[15*15];

	arm_matrix_instance_f32 Fi;
	float32_t Fi_data[15*12];

	//observation matrix
	arm_matrix_instance_f32 H_GPS;
	float32_t H_GPS_data[3*15];

	arm_matrix_instance_f32 H_MAG;
	float32_t H_MAG_data[3*15];

	//Kalman gain matrix
	arm_matrix_instance_f32 K_GPS;
	float32_t K_GPS_data[15*3];

	arm_matrix_instance_f32 K_MAG;
	float32_t K_MAG_data[15*3];

	//measurements
	arm_matrix_instance_f32 am;
	float32_t am_data[3*1];

	arm_matrix_instance_f32 wm;
	float32_t wm_data[3*1];

	arm_matrix_instance_f32 mm;
	float32_t mm_data[3*1];


	//Utils - identity matrix
	arm_matrix_instance_f32 I3;
	float32_t I3_data[3*3];

	arm_matrix_instance_f32 I12;
	float32_t I12_data[12*12];

	arm_matrix_instance_f32 I15;
	float32_t I15_data[15*15];

	arm_matrix_instance_f32 g;
	float32_t g_data[3*1];


	//Initialization variables
	int IMU_init_count;
	int MAG_init_count;
	int GPS_init_count;

	arm_matrix_instance_f32 am_init;
	float32_t am_init_data[3*1];

	arm_matrix_instance_f32 mm_init;
	float32_t mm_init_data[3*1];

	float32_t lla_init[3];

	//Time
	double last_t;

	//temp variables necessary during computation
	arm_matrix_instance_f32 am_unbias;
	float32_t am_unbias_data[3*1];

	arm_matrix_instance_f32 wm_unbias;
	float32_t wm_unbias_data[3*1];

	arm_matrix_instance_f32 R_hat_am_unbias;// R * hat(am - ab) = R * hat(am_unbias)
	float32_t R_hat_am_unbias_data[3*3];

	arm_matrix_instance_f32 matexp2_wub_dt;
	float32_t matexp2_wub_dt_data[3*3];


}ESKF_filter;

#endif /* COMPONENTS_INC_ESKF_H_ */
