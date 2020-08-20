/*
 * armmathutils.h
 *
 *  Created on: 2020年8月19日
 *      Author: yuche
 */

#ifndef COMPONENTS_ARMMATHUTILS_H_
#define COMPONENTS_ARMMATHUTILS_H_

#include "arm_math.h"

void zeros(arm_matrix_instance_f32* mat);
void eye(arm_matrix_instance_f32* mat);
void matcpy(arm_matrix_instance_f32* mat1,arm_matrix_instance_f32* mat2);
void matcpy2(arm_matrix_instance_f32* mat1,arm_matrix_instance_f32* mat2,int i, int j);



#endif /* COMPONENTS_ARMMATHUTILS_H_ */
