/*
 * EKF.c
 *
 *  Created on: 2020年7月7日
 *      Author: yuche
 */

#include "ESKF.h"
#include "math.h"
#include "quaternion.h"
#include "armMathUtils.h"


#define P_INITIAL_COV 100.0			//position cov: 10m
#define V_INITIAL_COV 100.0			//velocity cov: 10m/s
#define PR_INITIAL_COV 0.04			//roll&pitch cov: 0.2rad
#define Y_INITIAL_COV 2.0			//yaw cov: 1.414rad
#define AB_INITIAL_COV 0.0004		//accelerometer bias cov: 0.02
#define WB_INITIAL_COV 0.0004		//gyrometer bias cov: 0.02

#define GPS_XY_INITIAL_COV 1.81		//GPS observation horizontal cov
#define GPS_Z_INITIAL_COV 4.32		//GPS observation vertical cov
#define MAG_INITIAL_COV	0.5			//MAG observation cov (normalized value)

#define IMU_INITIALIZE_COUNT 10		//number of acceleration samples collected
									//to initialize roll and pitch. (initial orientation)

#define MAG_INITIALIZE_COUNT 10		//number of magnetometer samples collected
									//to initialize declination.This step requires initialize
									//being horizontal

#define GPS_INITIALIZE_COUNT 10		//number of GPS samples collected to initialize local lat/lon origin


void ESKF_new(ESKF_filter* eskf){

	eskf->IMU_initialized = 0;
	eskf->MAG_initialized = 0;
	eskf->GPS_initialized = 0;

	//for simplicity
	float32_t* P = eskf->P.pData;

	//Utils
	arm_mat_init_f32(&eskf->I3,3,3,eskf->I3_data);
	arm_mat_init_f32(&eskf->I12,12,12,eskf->I12_data);
	arm_mat_init_f32(&eskf->I15,15,15,eskf->I15_data);

	eye(&eskf->I3);
	eye(&eskf->I12);
	eye(&eskf->I15);

	//Nominal states
	arm_mat_init_f32(&eskf->p,3,1,eskf->p_data);
	arm_mat_init_f32(&eskf->v,3,1,eskf->v_data);
	arm_mat_init_f32(&eskf->q,4,1,eskf->q_data);
	arm_mat_init_f32(&eskf->ab,3,1,eskf->ab_data);
	arm_mat_init_f32(&eskf->wb,3,1,eskf->wb_data);

	zeros(&eskf->p);
	zeros(&eskf->v);
	zeros(&eskf->q);eskf->q.pData[0] = 1.0;//q = [1,0,0,0]'
	zeros(&eskf->ab);
	zeros(&eskf->wb);

	//Error states
	arm_mat_init_f32(&eskf->del_p,3,1,eskf->del_p_data);
	arm_mat_init_f32(&eskf->del_v,3,1,eskf->del_v_data);
	arm_mat_init_f32(&eskf->del_theta,3,1,eskf->del_theta_data);
	arm_mat_init_f32(&eskf->del_ab,3,1,eskf->del_ab_data);
	arm_mat_init_f32(&eskf->del_wb,3,1,eskf->del_wb_data);

	zeros(&eskf->del_p);
	zeros(&eskf->del_v);
	zeros(&eskf->del_theta);
	zeros(&eskf->del_ab);
	zeros(&eskf->del_wb);

	//covariance matrix
	arm_mat_init_f32(&eskf->P,15,15,eskf->P_data);
	arm_mat_init_f32(&eskf->Q,12,12,eskf->Q_data);

	zeros(&eskf->P);

	P[0] = P[16] = P[32] = P_INITIAL_COV;
	P[48] = P[64] = P[80] = V_INITIAL_COV;
	P[96] = P[112] = PR_INITIAL_COV;
	P[128] = Y_INITIAL_COV;
	P[144] = P[160] = P[176] = AB_INITIAL_COV;
	P[192] = P[208] = P[224] = WB_INITIAL_COV;

	zeros(&eskf->Q);//Value of Q depends on dt

	arm_mat_init_f32(&eskf->V_GPS,3,3,eskf->V_GPS_data);
	arm_mat_init_f32(&eskf->V_MAG,3,3,eskf->V_MAG_data);

	zeros(&eskf->V_GPS);
	eskf->V_GPS.pData[0] = eskf->V_GPS.pData[4] = GPS_XY_INITIAL_COV;
	eskf->V_GPS.pData[8] = GPS_Z_INITIAL_COV;

	zeros(&eskf->V_MAG);
	eskf->V_MAG.pData[0] = eskf->V_MAG.pData[4] = eskf->V_MAG.pData[8] = MAG_INITIAL_COV;

	//state transfer matrix
	arm_mat_init_f32(&eskf->Fx,15,15,eskf->Fx_data);
	arm_mat_init_f32(&eskf->Fi,15,12,eskf->Fi_data);

	eye(&eskf->Fx);//other part of Fx depends on dt

	zeros(&eskf->Fi);
	matcpy2(&eskf->Fi,&eskf->I12,3,0);

	//observation matrix
	arm_mat_init_f32(&eskf->H_GPS,3,15,eskf->H_GPS_data);
	arm_mat_init_f32(&eskf->H_MAG,3,15,eskf->H_MAG_data);

	zeros(&eskf->H_GPS);
	matcpy2(&eskf->H_GPS,&eskf->I3,0,0);

	zeros(&eskf->H_MAG);//H_MAG depends on q

	//Kalman gain matrix
	arm_mat_init_f32(&eskf->K_GPS,15,3,eskf->K_GPS_data);
	arm_mat_init_f32(&eskf->K_MAG,15,3,eskf->K_MAG_data);

	zeros(&eskf->K_GPS);
	zeros(&eskf->K_MAG);

	//measurements
	arm_mat_init_f32(&eskf->am,3,1,eskf->am_data);
	arm_mat_init_f32(&eskf->wm,3,1,eskf->wm_data);
	arm_mat_init_f32(&eskf->mm,3,1,eskf->mm_data);

	zeros(&eskf->am);
	zeros(&eskf->wm);
	zeros(&eskf->mm);

	//Initialization variables
	eskf->IMU_init_count = 0;
	eskf->MAG_init_count = 0;
	eskf->GPS_init_count = 0;

	arm_mat_init_f32(&eskf->am_init,3,1,eskf->am_init_data);
	arm_mat_init_f32(&eskf->mm_init,3,1,eskf->mm_init_data);

	zeros(&eskf->am_init);
	zeros(&eskf->mm_init);

	//time
	eskf->last_t = -1;
}

/**
 * a - acceleration measurement in XYZ, unit: m/s^2
 * w - gyration measurement in XYZ following right-hand rule, unit: rad/s
 * m - magnetic field measurement in XYZ, internally normalized
 * lla - latitude, longitude, altitude provided by GPS
 * info - indicates which type of information is passed to this function.
 * 		- 1 = IMU, 2 = MAG, 3 = GPS
 */
void ESKF_update(ESKF_filter* eskf, double t, double am[3], double wm[3], double mm[3], double lla[3], int info){

	double mm_norm = sqrt(mm[0]*mm[0] + mm[1]*mm[1] + mm[2]*mm[2]);
	if (mm_norm != 0){
		mm[0] = mm[0] / mm_norm;
		mm[1] = mm[1] / mm_norm;
		mm[2] = mm[2] / mm_norm;
	}

	//store values into measurement vectors
	memcpy(eskf->am.pData,am,3*sizeof(float32_t));
	memcpy(eskf->wm.pData,wm,3*sizeof(float32_t));
	memcpy(eskf->mm.pData,mm,3*sizeof(float32_t));

	//initialization
	if (eskf->IMU_initialized == 0 || eskf->MAG_initialized == 0 || eskf->GPS_initialized == 0){

		if (info == 1 && eskf->IMU_initialized == 0){

			if (eskf->IMU_init_count < IMU_INITIALIZE_COUNT){
				arm_mat_add_f32(&eskf->am,&eskf->am_init,&eskf->am_init);
				eskf->IMU_init_count++;
			}

			if (eskf->IMU_init_count == IMU_INITIALIZE_COUNT){
				arm_mat_scale_f32(&eskf->am_init,1/IMU_INITIALIZE_COUNT,&eskf->am_init);

				//calculate initial orientation



				eskf->IMU_initialized = 1;
			}
		}

		if (info == 2 && eskf->MAG_initialized == 0){




		}

		if (info == 3 && eskf->GPS_initialized == 0){




		}

		eskf->last_t = t;
	}


}













