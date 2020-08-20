/*
 * quaternion.c
 *
 *  Created on: 2020年8月20日
 *      Author: yuche
 */


#include "quaternion.h"
#include "arm_math.h"



void quat2mat(arm_matrix_instance_f32* q_,arm_matrix_instance_f32* R_){

#ifdef QUATERNION_SIZE_CHECK
	if (q_->numCols != 1 || q_->numRows != 4 || R_->numCols != 3 || R_->numRows != 3){
		return;
	}
#endif
	float32_t* q = q_->pData;
	float32_t* R = R_->pData;

	float32_t q0_2 = q[0]*q[0];
	float32_t q1_2 = q[1]*q[1];
	float32_t q2_2 = q[2]*q[2];
	float32_t q3_2 = q[3]*q[3];
	float32_t q0q1_ = 2 * q[0]*q[1];
	float32_t q0q2_ = 2 * q[0]*q[2];
	float32_t q0q3_ = 2 * q[0]*q[3];
	float32_t q1q2_ = 2 * q[1]*q[2];
	float32_t q1q3_ = 2 * q[1]*q[3];
	float32_t q2q3_ = 2 * q[2]*q[3];

	R[0] = q0_2 + q1_2 - q2_2 - q3_2;
	R[1] = q1q2_ - q0q3_;
	R[2] = q1q3_ + q0q2_;
	R[3] = q1q2_ + q0q3_;
	R[4] = q0_2 - q1_2 + q2_2 - q3_2;
	R[5] = q2q3_ - q0q1_;
	R[6] = q1q3_ - q0q2_;
	R[7] = q2q3_ + q0q1_;
	R[8] = q0_2 - q1_2 - q2_2 + q3_2;
}

/*
 * Matrix equivalent to left quaternion multiplication
 */
void quat_L(arm_matrix_instance_f32* q_, arm_matrix_instance_f32* qL_){

#ifdef QUATERNION_SIZE_CHECK
	if (q_->numCols != 1 || q_->numRows != 4 || qL->numCols != 4 || qL->numRows != 4){
		return;
	}
#endif

	float32_t* q = q_->pData;
	float32_t* qL = qL_->pData;

	qL[0] = q[0];		qL[1] = -q[1];		qL[2] = -q[2];		qL[3] = -q[3];
	qL[4] = q[1];		qL[5] = q[0];		qL[6] = -q[3];		qL[7] = q[2];
	qL[8] = q[2];		qL[9] = q[3];		qL[10] = q[0];		qL[11] = -q[1];
	qL[12] = q[3];		qL[13] = -q[2];		qL[14] = q[1];		qL[15] = q[0];

}

void quat_R(arm_matrix_instance_f32* q_, arm_matrix_instance_f32* qR_){
#ifdef QUATERNION_SIZE_CHECK
	if (q_->numCols != 1 || q_->numRows != 4 || qL->numCols != 4 || qL->numRows != 4){
		return;
	}
#endif

	float32_t* q = q_->pData;
	float32_t* qR = qR_->pData;

	qR[0] = q[0];		qR[1] = -q[1];		qR[2] = -q[2];		qR[3] = -q[3];
	qR[4] = q[1];		qR[5] = q[0];		qR[6] = q[3];		qR[7] = -q[2];
	qR[8] = q[2];		qR[9] = -q[3];		qR[10] = q[0];		qR[11] = q[1];
	qR[12] = q[3];		qR[13] = q[2];		qR[14] = -q[1];		qR[15] = q[0];
}

/**
 * quaternion multiplication
 */
void otimes(arm_matrix_instance_f32* q1,arm_matrix_instance_f32* q2,arm_matrix_instance_f32* q_res){
	arm_matrix_instance_f32 qL;
	float32_t qL_data[4*4];
	arm_mat_init_f32(&qL,4,4,qL_data);

	quat_L(q1,&qL);
	arm_mat_mult_f32(&qL,q2,q_res);
}


/**
 * Jacobian w.r.t. rotation. Define vector v' as the result of rotating vector v by quaternion
 * q. i.e. v' = q \otimes v \otimes q*.
 */
void diff_qvq_diff_a(arm_matrix_instance_f32* q_, arm_matrix_instance_f32* jacob_){
	quat2mat(q_,jacob_);
}

void diff_qvq_diff_q(arm_matrix_instance_f32* q_,arm_matrix_instance_f32* v_, arm_matrix_instance_f32* jacob_){
	float32_t* q = q_->pData;
	float32_t* v = v_->pData;
	float32_t* jacob = jacob_->pData;

	jacob[0] = q[0]*v[0] - q[3]*v[1] + q[2]*v[2];
	jacob[4] = q[0]*v[1] + q[3]*v[0] - q[1]*v[2];
	jacob[8] = q[0]*v[2] - q[2]*v[0] + q[1]*v[1];

	double qv_T_v = q[1]*v[0] + q[2]*v[1] + q[3]*v[2];

	jacob[1] = qv_T_v;
	jacob[2] = q[1]*v[1] - q[2]*v[0] + q[0]*v[2];
	jacob[3] = q[1]*v[2] - q[3]*v[0] - q[0]*v[1];

	jacob[5] = q[2]*v[0] - q[1]*v[1] - q[0]*v[2];
	jacob[6] = qv_T_v;
	jacob[7] = q[2]*v[2] - q[3]*v[1] + q[0]*v[0];

	jacob[9] = q[3]*v[0] - q[1]*v[2] + q[0]*v[1];
	jacob[10] = q[3]*v[1] - q[2]*v[2] - q[0]*v[0];
	jacob[11] = qv_T_v;
}

/**
 * Jacobian w.r.t. quaternion perturbation. i.e. diff q \oplus \delta\theta diff \delta\theta
 */
void diff_q_diff_dtheta(arm_matrix_instance_f32* q_, arm_matrix_instance_f32* jacob_){
	float32_t* q = q_->pData;
	float32_t* jacob = jacob_->pData;

	jacob[0] = -q[1];	jacob[1] = -q[2];	jacob[2] = -q[3];
	jacob[3] = q[0];	jacob[4] = -q[3];	jacob[5] = q[2];
	jacob[6] = q[3];	jacob[7] = q[0];	jacob[8] = -q[1];
	jacob[9] = -q[2];	jacob[10] = q[1];	jacob[11] = q[0];

	arm_mat_scale_f32(jacob_,0.5,jacob_);
}

