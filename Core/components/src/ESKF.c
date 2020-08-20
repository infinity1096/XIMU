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
	arm_mat_init_f32(&eskf->g,3,1,eskf->g_data);

	eye(&eskf->I3);
	eye(&eskf->I12);
	eye(&eskf->I15);
	zeros(&eskf->g);	eskf->g_data[2] = -9.81;

	//Nominal states
	arm_mat_init_f32(&eskf->p,3,1,eskf->p_data);
	arm_mat_init_f32(&eskf->v,3,1,eskf->v_data);
	arm_mat_init_f32(&eskf->q,4,1,eskf->q_data);
	arm_mat_init_f32(&eskf->R,4,1,eskf->R_data);
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

	//AUX variables necessary during computation
	arm_mat_init_f32(&eskf->am_unbias,3,1,eskf->am_unbias_data);
	arm_mat_init_f32(&eskf->wm_unbias,3,1,eskf->wm_unbias_data);

	arm_mat_init_f32(&eskf->R_hat_am_unbias,3,3,eskf->R_hat_am_unbias_data);
	arm_mat_init_f32(&eskf->matexp2_wub_dt,3,3,eskf->matexp2_wub_dt_data);


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

	//[IMU Information arrived]
	if (info == 1){
		double dt = t - eskf->last_t;
		double dt_2 = dt * dt;
		eskf->last_t = t;

		arm_matrix_instance_f32 tempvec;
		float32_t tempvec_data[3*1];
		arm_mat_init_f32(&tempvec,3,1,tempvec_data);
		arm_matrix_instance_f32 tempmat;
		float32_t tempmat_data[3*3];
		arm_mat_init_f32(&tempmat,3,3,tempmat_data);


		quat2mat(&eskf->q,&eskf->R);//Get equivlent representation of orientation

		arm_mat_sub_f32(&eskf->am,&eskf->ab,&eskf->am_unbias);//subtract bias

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

		//update q TODO

		//update ab (unchanged)
		//update wb (unchanged)

		//Update error states -----------------------------------------------
		quat2mat(&eskf->q,&eskf->R);//Update R to our best estimation

		//Since we haven't observed the error state(It's reset after every observation),and
		//its mean is 0. Therefore our best estimate of the error state is 0. Then its mean
		//does not need to be updated. However, its uncertainty needs to be updated.

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
		matcpy2(&eskf->Fx,&eskf->I3,0,3);
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

		//fill Q according to predefined an, wn, abn, wbn.


	}

	//[MAG Information arrived]
	if (info == 2){

	}

	//[GPS Information arrived]
	if (info == 3){

	}




}













