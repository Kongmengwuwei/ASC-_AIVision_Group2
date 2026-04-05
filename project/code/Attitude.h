/*
 * Attitude.h
 *
 *  Created on: 2024��3��9��
 *      Author: huawei
 */

#ifndef CODE_ATTITUDE_H_
#define CODE_ATTITUDE_H_
#include "zf_device_imu963ra.h"

typedef struct
{
    float roll;
    float pitch;
    float yaw;
    float roll_offset;
} attitude_euler_t;

typedef struct
{
    float gyro_x;
    float gyro_y;
    float gyro_z;
    float acc_x;
    float acc_y;
    float acc_z;
} imu963ra_data_t;

extern attitude_euler_t eulerAngle;
extern imu963ra_data_t icm_data;
void Attitude_Init(void); //attitude init
void Attitude_Calculate(void);  //attitude calculate

extern float gyroscope[3];
extern float accelerometer[3];
extern float gyroscopeOffset[3];
#endif /* CODE_ATTITUDE_H_ */
