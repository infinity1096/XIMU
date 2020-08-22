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

#define A_NOISE 5e-2
#define W_NOISE 5e-4
#define AB_NOISE 0.0001
#define WB_NOISE 0.0001

void ESKF_new(ESKF_filter* eskf){

	eskf->IMU_initialized = 0;
	eskf->MAG_initialized = 0;
	eskf->GPS_initialized = 0;

	//Utils
	arm_mat_init_f32(&eskf->I3,3,3,eskf->I3_data);
	arm_mat_init_f32(&eskf->I12,12,12,eskf->I12_data);
	arm_mat_init_f32(&eskf->I15,15,15,eskf->I15_data);
	arm_mat_init_f32(&eskf->g,3,1,eskf->g_data);

	eye(&eskf->I3);
	eye(&eskf->I12);
	eye(&eskf->I15);
	zeros(&eskf->g);	eskf->g_data[2] = -9.81;

	//Nominal states
	arm_mat_init_f32(&eskf->p,3,1,eskf->p_data);
	arm_mat_init_f32(&eskf->v,3,1,eskf->v_data);
	arm_mat_init_f32(&eskf->q,4,1,eskf->q_data);
	arm_mat_init_f32(&eskf->R,3,3,eskf->R_data);
	arm_mat_init_f32(&eskf->ab,3,1,eskf->ab_data);
	arm_mat_init_f32(&eskf->wb,3,1,eskf->wb_data);

	zeros(&eskf->p);
	zeros(&eskf->v);
	zeros(&eskf->q);eskf->q.pData[0] = 1.0;//q = [1,0,0,0]'
	eye(&eskf->R);
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

	//for simplicity
	float32_t* P = eskf->P.pData;

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

	//observation
	arm_mat_init_f32(&eskf->z_GPS,3,1,eskf->z_GPS_data);
	arm_mat_init_f32(&eskf->z_MAG,3,1,eskf->z_MAG_data);

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
	arm_mat_init_f32(&eskf->m_ref,3,1,eskf->m_ref_data);

	zeros(&eskf->am_init);
	zeros(&eskf->mm_init);
	zeros(&eskf->m_ref);

	//time
	eskf->last_t = 0;

	//AUX variables necessary during computation
	arm_mat_init_f32(&eskf->am_unbias,3,1,eskf->am_unbias_data);
	arm_mat_init_f32(&eskf->wm_unbias,3,1,eskf->wm_unbias_data);

	arm_mat_init_f32(&eskf->del_q,4,1,eskf->del_q_data);

	arm_mat_init_f32(&eskf->R_hat_am_unbias,3,3,eskf->R_hat_am_unbias_data);

	arm_mat_init_f32(&eskf->Fx_T,15,15,eskf->Fx_T_data);
	arm_mat_init_f32(&eskf->Fx_P,15,15,eskf->Fx_P_data);
	arm_mat_init_f32(&eskf->P_temp,15,15,eskf->P_temp_data);
	arm_mat_init_f32(&eskf->Fi_T,12,15,eskf->Fi_T_data);
	arm_mat_init_f32(&eskf->Fi_Q,15,12,eskf->Fi_Q_data);

	//MAG update related variables

	//GPS update related variables
	arm_mat_init_f32(&eskf->H_GPS_T,15,3,eskf->H_GPS_T_data);
	arm_mat_trans_f32(&eskf->H_GPS,&eskf->H_GPS_T);

	arm_mat_init_f32(&eskf->COV_GPS,3,3,eskf->COV_GPS_data);
	arm_mat_init_f32(&eskf->inv_COV_GPS,3,3,eskf->inv_COV_GPS_data);
	arm_mat_init_f32(&eskf->P_H_GPS_T,15,3,eskf->P_H_GPS_T_data);

	arm_mat_init_f32(&eskf->z_hx_GPS,3,1,eskf->z_hx_GPS_data);

	arm_mat_init_f32(&eskf->I_KH_GPS,15,15,eskf->I_KH_GPS_data);
	arm_mat_init_f32(&eskf->I_KH_GPS_T,15,15,eskf->I_KH_GPS_T_data);

	arm_mat_init_f32(&eskf->KV_GPS,15,3,eskf->KV_GPS_data);
	arm_mat_init_f32(&eskf->K_GPS_T,3,15,eskf->K_GPS_T_data);


	//shared between MAG and GPS update
	arm_mat_init_f32(&eskf->del_x,15,1,eskf->del_x_data);
}

/**
 * a - acceleration measurement in XYZ, unit: m/s^2
 * w - gyration measurement in XYZ following right-hand rule, unit: rad/s
 * m - magnetic field measurement in XYZ, internally normalized
 * lla - latitude, longitude, altitude provided by GPS
 * info - indicates which type of information is passed to this function.
 * 		- 1 = IMU, 2 = MAG, 3 = GPS
 */
void ESKF_update(ESKF_filter* eskf, double t, float32_t am[3], float32_t wm[3], float32_t mm[3], float32_t lla[3], int info){

	//for simplicity
	float32_t* Q = eskf->Q.pData;

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
				arm_mat_scale_f32(&eskf->am_init,1.0/IMU_INITIALIZE_COUNT,&eskf->am_init);

				//calculate initial orientation



				eskf->IMU_initialized = 1;
			}
		}

		if (info == 2 && eskf->MAG_initialized == 0){




		}

		if (info == 3 && eskf->GPS_initialized == 0){
			eskf->lla_init[0] += lla[0];
			eskf->lla_init[1] += lla[1];
			eskf->lla_init[2] += lla[2];

			eskf->GPS_init_count++;
			if (eskf->GPS_init_count == GPS_INITIALIZE_COUNT){
				eskf->lla_origin[0] = eskf->lla_init[0] / GPS_INITIALIZE_COUNT;
				eskf->lla_origin[1] = eskf->lla_init[1] / GPS_INITIALIZE_COUNT;
				eskf->lla_origin[2] = eskf->lla_init[2] / GPS_INITIALIZE_COUNT;

				eskf->GPS_initialized = 1;
			}
		}

		//eskf->last_t = t; TODO only for testing
		//return; TODO only for testing
	}

	//[IMU Information arrived]
	if (info == 1){
		double dt = t - eskf->last_t;
		double dt_2 = dt * dt;
		eskf->last_t = t;

		arm_matrix_instance_f32 tempvec;
		float32_t tempvec_data[3*1];
		arm_mat_init_f32(&tempvec,3,1,tempvec_data);
		arm_matrix_instance_f32 tempquat;
		float32_t tempquat_data[4*1];
		arm_mat_init_f32(&tempquat,4,1,tempquat_data);
		arm_matrix_instance_f32 tempmat;
		float32_t tempmat_data[3*3];
		arm_mat_init_f32(&tempmat,3,3,tempmat_data);


		quat2mat(&eskf->q,&eskf->R);//Get equivlent representation of orientation

		arm_mat_sub_f32(&eskf->am,&eskf->ab,&eskf->am_unbias);//subtract acc  bias
		arm_mat_sub_f32(&eskf->wm,&eskf->wb,&eskf->wm_unbias);//subtract gyro bias

		//Update nominal states -----------------------------------------------

		/* MATLAB code
		p = p + v * dt + 1/2*(R*(am - ab) + g) * dt_2;
        v = v + (R*(am - ab) + g) * dt;
        q = otimes(q,quatexp2((wm-wb)*dt));
        %ab = ab;
        %wb = wb;
		*/

		//update p
		arm_mat_scale_f32(&eskf->v,dt,&tempvec);
		arm_mat_add_f32(&eskf->p,&tempvec,&eskf->p);
		arm_mat_mult_f32(&eskf->R,&eskf->am_unbias,&tempvec);
		arm_mat_add_f32(&tempvec,&eskf->g,&tempvec);
		arm_mat_scale_f32(&tempvec,0.5*dt_2,&tempvec);
		arm_mat_add_f32(&eskf->p,&tempvec,&eskf->p);

		//update v
		arm_mat_mult_f32(&eskf->R,&eskf->am_unbias,&tempvec);
		arm_mat_add_f32(&tempvec,&eskf->g,&tempvec);
		arm_mat_scale_f32(&tempvec,dt,&tempvec);
		arm_mat_add_f32(&eskf->v,&tempvec,&eskf->v);

		//update q
		arm_mat_scale_f32(&eskf->wm_unbias,dt,&tempvec);
		quatexp2(&tempvec,&eskf->del_q);
		otimes(&eskf->q,&eskf->del_q,&tempquat);
		matcpy(&tempquat,&eskf->q);

		//update ab (unchanged)
		//update wb (unchanged)

		//Update error states -----------------------------------------------
		quat2mat(&eskf->q,&eskf->R);//Update R to our best estimation

		//Since we haven't observed the error state(It's reset after every observation),and
		//its mean is 0. Therefore our best estimate of the error state is 0. Then its mean
		//does not need to be updated. Its uncertainty, however, needs to be updated.

		/* MATLAB code
		Fx = eye(15);
        Fx(1:3,4:6) = eye(3) * dt;
        Fx(4:6,7:9) = -R * hat(am-ab)*dt;
        Fx(4:6,10:12) = -R*dt;
        Fx(7:9,7:9) = matexp2((wm-wb)*dt)'; <- Note this transpose!
        Fx(7:9,13:15) = -eye(3)*dt;
		 */

		//fill Fx
		eye(&eskf->Fx);
		//
		arm_mat_scale_f32(&eskf->I3,dt,&tempmat);
		matcpy2(&eskf->Fx,&tempmat,0,3);
		//
		hat(&eskf->am_unbias,&tempmat);
		arm_mat_mult_f32(&eskf->R,&tempmat,&eskf->R_hat_am_unbias);
		arm_mat_scale_f32(&eskf->R_hat_am_unbias,-dt,&eskf->R_hat_am_unbias);
		matcpy2(&eskf->Fx,&eskf->R_hat_am_unbias,3,6);
		//
		arm_mat_scale_f32(&eskf->R,-dt,&tempmat);
		matcpy2(&eskf->Fx,&tempmat,3,9);
		//
		arm_mat_scale_f32(&eskf->wm_unbias,-dt,&tempvec);//changed dt to -dt is equivalent to transpose
		matexp2(&tempvec,&tempmat);
		matcpy2(&eskf->Fx,&tempmat,6,6);
		//
		arm_mat_scale_f32(&eskf->I3,-dt,&tempmat);
		matcpy2(&eskf->Fx,&tempmat,6,12);

		//fill Fi : Fi does not change. Already done in "ESKF_new".

		//fill Q according to predefined an, wn, abn, wbn and dt.
		Q[0] = Q[13] = Q[26] = 1.0e-4;
		Q[39] = Q[52] = Q[65] = 1.0e-6;
		Q[78] = Q[91] = Q[104] = 1e-6;//Due to computation precision, can't use dt here
		Q[117] = Q[130] = Q[143] = 1e-6;

		//update P: P = Fx * P * Fx' + Fi * Q * Fi';
		arm_mat_trans_f32(&eskf->Fx,&eskf->Fx_T);
		arm_mat_mult_f32(&eskf->Fx,&eskf->P,&eskf->Fx_P);
		arm_mat_mult_f32(&eskf->Fx_P,&eskf->Fx_T,&eskf->P_temp);
		matcpy(&eskf->P_temp,&eskf->P);
		arm_mat_mult_f32(&eskf->Fi,&eskf->Q,&eskf->Fi_Q);
		arm_mat_trans_f32(&eskf->Fi,&eskf->Fi_T);
		arm_mat_mult_f32(&eskf->Fi_Q,&eskf->Fi_T,&eskf->P_temp);
		arm_mat_add_f32(&eskf->P,&eskf->P_temp,&eskf->P);
	}

	//[MAG Information arrived]
	if (info == 2){

	}

	//[GPS Information arrived]
	if (info == 3){

		lla2enu(eskf->lla_origin,lla,eskf->z_GPS_data);

		//H_GPS does not change (unless your IMU and GPS is sepreated by some distance).
		//Compute Kalman gain. MATLAB code:
		//      K = P * H' * inv(H * P * H' + V);
        //		del_x = K * (z - p);
		//
		//	since H = [I 0 0 0 0], H*P*H' = P(1:3,1:3)

		matslice(&eskf->P,&eskf->COV_GPS,0,0);
		arm_mat_add_f32(&eskf->COV_GPS,&eskf->V_GPS,&eskf->COV_GPS);
		arm_mat_inverse_f32(&eskf->COV_GPS,&eskf->inv_COV_GPS);

		arm_mat_mult_f32(&eskf->P,&eskf->H_GPS_T,&eskf->P_H_GPS_T);
		arm_mat_mult_f32(&eskf->P_H_GPS_T,&eskf->inv_COV_GPS,&eskf->K_GPS);

		arm_mat_sub_f32(&eskf->z_GPS,&eskf->p,&eskf->z_hx_GPS);
		arm_mat_mult_f32(&eskf->K_GPS,&eskf->z_hx_GPS,&eskf->del_x);

		//update covariance. MATLAB code:
		//P = (eye(15) - K * H) * P * (eye(15) - K * H)' + K * V * K';
		//since H [I 0 0 0 0], K * H = [K zeros(15,12)]

		zeros(&eskf->I_KH_GPS);
		matcpy2(&eskf->I_KH_GPS,&eskf->K_GPS,0,0);
		arm_mat_scale_f32(&eskf->I_KH_GPS,-1,&eskf->I_KH_GPS);//now &eskf->I_KH_GPS = -K*H
		arm_mat_add_f32(&eskf->I_KH_GPS,&eskf->I15,&eskf->I_KH_GPS);
		arm_mat_trans_f32(&eskf->I_KH_GPS,&eskf->I_KH_GPS_T);

		//update P
		arm_mat_mult_f32(&eskf->I_KH_GPS,&eskf->P,&eskf->P_temp);
		matcpy(&eskf->P_temp,&eskf->P);
		arm_mat_mult_f32(&eskf->P,&eskf->I_KH_GPS_T,&eskf->P_temp);
		matcpy(&eskf->P_temp,&eskf->P);

		arm_mat_trans_f32(&eskf->K_GPS,&eskf->K_GPS_T);
		arm_mat_mult_f32(&eskf->K_GPS,&eskf->V_GPS,&eskf->KV_GPS);
		arm_mat_mult_f32(&eskf->KV_GPS,&eskf->K_GPS_T,&eskf->P_temp);
		arm_mat_add_f32(&eskf->P,&eskf->P_temp,&eskf->P);

		inject_error_state(eskf);
	}
}

/**
 * inject error into nominal state, and then reset error state.
 */
void inject_error_state(ESKF_filter* eskf){

	arm_matrix_instance_f32 tempquat;
	float32_t tempquat_data[4*1];
	arm_mat_init_f32(&tempquat,4,1,tempquat_data);

	//copy data in del_x to corresponding error state
	matslice(&eskf->del_x,&eskf->del_p,0,0);
	matslice(&eskf->del_x,&eskf->del_v,3,0);
	matslice(&eskf->del_x,&eskf->del_theta,6,0);
	matslice(&eskf->del_x,&eskf->del_ab,9,0);
	matslice(&eskf->del_x,&eskf->del_wb,12,0);

	//inject error states into nominal states
	arm_mat_add_f32(&eskf->p,&eskf->del_p,&eskf->p);
	arm_mat_add_f32(&eskf->v,&eskf->del_v,&eskf->v);

	quatexp2(&eskf->del_theta,&eskf->del_q);
	otimes(&eskf->q,&eskf->del_q,&tempquat);
	matcpy(&tempquat,&eskf->q);

	arm_mat_add_f32(&eskf->ab,&eskf->del_ab,&eskf->ab);
	arm_mat_add_f32(&eskf->wb,&eskf->del_wb,&eskf->wb);

	//no need to reset del_p ~ del_wb to zero, since they always
	//get their value here.
}


/**
 * convert lla - latitude, longitude, altitude in WGS84 coordinate
 * into ENU - East, North, Up coordinate centered at lla_ref
 *
 * More details can be found here:
 * http://www.wiki.gis.com/wiki/index.php/Geodetic_system
 */
void lla2enu(float32_t lla_ref[3], float32_t lla[3], float32_t enu[3]){

	float32_t xyz_ref[3], xyz[3];
	lla2xyz(lla_ref,xyz_ref);
	lla2xyz(lla,xyz);

	float32_t lat_ref = lla_ref[0] / 180.0 * M_PI;
	float32_t lon_ref = lla_ref[1] / 180.0 * M_PI;

	float32_t sin_lat_ref = sin(lat_ref);
	float32_t cos_lat_ref = cos(lat_ref);
	float32_t sin_lon_ref = sin(lon_ref);
	float32_t cos_lon_ref = cos(lon_ref);

	float32_t dx = xyz[0] - xyz_ref[0];
	float32_t dy = xyz[1] - xyz_ref[1];
	float32_t dz = xyz[2] - xyz_ref[2];

	enu[0] = -sin_lon_ref * dx + cos_lon_ref * dy;
	enu[1] = -sin_lat_ref*cos_lon_ref*dx - sin_lat_ref*sin_lon_ref*dy + cos_lat_ref * dz;
	enu[2] = cos_lat_ref*cos_lon_ref*dx + cos_lat_ref*sin_lon_ref*dy + sin_lat_ref * dz;
}

/**
 * convert lla to ECEF xyz coordinate
 */
void lla2xyz(float32_t lla[3], float32_t xyz[3]){

	float32_t lat = lla[0] / 180.0 * M_PI;//DEG -> RAD
	float32_t lon = lla[1] / 180.0 * M_PI;

	float32_t a = 6378137.0;//earth semimajor axis in meters
	//float32_t f = 0.003352810664747;//reciprocal flattening
	float32_t e2 = 0.006694379990141;//eccentricity squared, e2 = 2*f - f^2

	float32_t sin_lat = sin(lat);
	float32_t cos_lat = cos(lat);
	float32_t sin_lon = sin(lon);
	float32_t cos_lon = cos(lon);

	float32_t chi = sqrt(1-e2 * sin_lat * sin_lat);

	xyz[0] = (a / chi + lla[2]) * cos_lat * cos_lon;
	xyz[1] = (a / chi + lla[2]) * cos_lat * sin_lon;
	xyz[2] = (a * (1-e2) / chi + lla[2]) * sin_lat;
}












