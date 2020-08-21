/*
 * armMathUtils.c
 *
 *  Created on: 2020年8月19日
 *      Author: yuche
 */

#include "armMathUtils.h"
#include "string.h"
#include "math.h"

#define MATH_UTILS_SIZE_CHECK

void zeros(arm_matrix_instance_f32* mat){
	memset(mat->pData,0,mat->numCols * mat->numRows * sizeof(float32_t));
}

void eye(arm_matrix_instance_f32* mat){

#ifdef MATH_UTILS_SIZE_CHECK
	if (mat->numCols != mat->numRows){
		return;
	}
#endif
	zeros(mat);
	for(int i = 0; i < (mat->numCols * mat->numRows); i += (mat->numCols+1)){
		mat->pData[i] = 1;
	}
}

void matcpy(arm_matrix_instance_f32* mat1,arm_matrix_instance_f32* mat2){

#ifdef MATH_UTILS_SIZE_CHECK
	if (mat1->numCols != mat2->numCols || mat1->numRows != mat2->numRows){
		return;
	}
#endif

	memcpy(mat2->pData,mat1->pData,mat1->numCols * mat1->numRows * sizeof(float32_t));
}


//copy mat2 to mat1 starting at i,j at mat1.
void matcpy2(arm_matrix_instance_f32* mat1,arm_matrix_instance_f32* mat2,int i, int j){

	int i2;//row ind in mat2
	int k = i * mat1->numCols + j;

	for(i2 = 0; i2 < mat2->numRows; i2++){
		memcpy(mat1->pData+k,mat2->pData + (i2 * mat2->numCols),mat2->numCols*sizeof(float32_t));
		k += mat1->numCols;
	}
}

void hat(arm_matrix_instance_f32* v_, arm_matrix_instance_f32* v_hat_){

	float32_t* v = v_->pData;
	float32_t* v_hat = v_hat_->pData;

	zeros(v_hat_);

	v_hat[1] = -v[2];
	v_hat[2] = v[1];
	v_hat[3] = v[2];
	v_hat[5] = -v[0];
	v_hat[6] = -v[1];
	v_hat[7] = v[0];
}

void vee(arm_matrix_instance_f32* v_hat_, arm_matrix_instance_f32* v_){

	float32_t* v = v_->pData;
	float32_t* v_hat = v_hat_->pData;

	v[0] = v_hat[7] - v_hat[5];
	v[1] = v_hat[2] - v_hat[6];
	v[2] = v_hat[3] - v_hat[1];

	arm_mat_scale_f32(v_,0.5,v_);
}

void matexp2(arm_matrix_instance_f32* phi_, arm_matrix_instance_f32* R_){

	float32_t* phi = phi_->pData;
	float32_t phi_norm = sqrt(phi[0]*phi[0] + phi[1]*phi[1] + phi[2]*phi[2]);

	arm_matrix_instance_f32 tempmat;
	float32_t tempmat_data[3*3];
	arm_matrix_instance_f32 u_hat;
	float32_t u_hat_data[3*3];


	arm_mat_init_f32(&tempmat,3,3,tempmat_data);
	arm_mat_init_f32(&u_hat,3,3,u_hat_data);

	//MATLAB code
	//mat = eye(3) + sin(phi) * phi_hat + (1 - cos(phi)) * (phi_hat^2);

	eye(R_);

	hat(phi_,&u_hat);

	if (fabs(phi_norm >= 1e-6)){
		arm_mat_scale_f32(&u_hat, 1/phi_norm, &u_hat);
		arm_mat_scale_f32(&u_hat,sin(phi_norm),&tempmat);
		arm_mat_add_f32(R_,&tempmat,R_);

		arm_mat_mult_f32(&u_hat,&u_hat,&tempmat);
		arm_mat_scale_f32(&tempmat,1-cos(phi_norm),&tempmat);
		arm_mat_add_f32(R_,&tempmat,R_);
	}else{
		//approximation: mat = eye(3) + phi^
		arm_mat_add_f32(R_,&u_hat,R_);
	}
}














