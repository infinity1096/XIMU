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

//3D rotation functions
















