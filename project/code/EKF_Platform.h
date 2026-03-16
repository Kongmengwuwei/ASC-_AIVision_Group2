/*
 * EKF_Platform.h
 * Platform helpers for quaternion EKF.
 */
#ifndef CODE_EKF_PLATFORM_H_
#define CODE_EKF_PLATFORM_H_

#include <math.h>
#include "matrix.h"   // arm platform uses local matrix implementation
#define arm_cos_f32 cosf
#define arm_atan2_f32 atan2f

#define mat arm_matrix_instance_f32
#define Matrix_Init arm_mat_init_f32
#define Matrix_Add arm_mat_add_f32
#define Matrix_Subtract arm_mat_sub_f32
#define Matrix_Multiply arm_mat_mult_f32
#define Matrix_Transpose arm_mat_trans_f32
#define Matrix_Inverse arm_mat_inverse_f32

#endif /* CODE_EKF_PLATFORM_H_ */
