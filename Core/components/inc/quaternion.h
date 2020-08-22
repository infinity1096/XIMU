/*
 * quaternion.h
 *
 *  Created on: 2020年8月20日
 *      Author: yuche
 */

#ifndef COMPONENTS_INC_QUATERNION_H_
#define COMPONENTS_INC_QUATERNION_H_

#include "arm_math.h"

void quat2mat(arm_matrix_instance_f32* q_,arm_matrix_instance_f32* R_);
void mat2quat(arm_matrix_instance_f32* R_,arm_matrix_instance_f32* q_);
void quat_L(arm_matrix_instance_f32* q_, arm_matrix_instance_f32* qL_);
void quat_R(arm_matrix_instance_f32* q_, arm_matrix_instance_f32* qR_);
void quatexp2(arm_matrix_instance_f32* phi_, arm_matrix_instance_f32* q_);
void otimes(arm_matrix_instance_f32* q1,arm_matrix_instance_f32* q2,arm_matrix_instance_f32* q_res);
void diff_qvq_diff_a(arm_matrix_instance_f32* q_, arm_matrix_instance_f32* jacob);
void diff_qvq_diff_q(arm_matrix_instance_f32* q_,arm_matrix_instance_f32* v_, arm_matrix_instance_f32* jacob_);
void diff_q_diff_dtheta(arm_matrix_instance_f32* q_, arm_matrix_instance_f32* jacob_);

#endif /* COMPONENTS_INC_QUATERNION_H_ */
