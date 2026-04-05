/*
 * describe: Attitude calculate
 * 角度计算
 */
#include "Attitude.h"
#include <math.h>
#include <string.h>
#include "zf_common_headfile.h"
#include "QuaternionEKF.h"
#pragma section all "cpu0_dsram"
//==================================================================================================
#define DEG_TO_RAD 0.01745329252f
#define gyroscope_threshold 300
#define ATTITUDE_UPDATE_PERIOD_S 0.002f			 //解算周期
#define GYRO_CALIBRATION_SAMPLES 500U
#define GYRO_CALIBRATION_MAX_TRIES (GYRO_CALIBRATION_SAMPLES * 2U)
//--------------------------------------------------------------------------------------
attitude_euler_t eulerAngle = {0};
imu963ra_data_t icm_data = {0};
float gyroscopeOffset[3] = {0.0f, 0.0f, 0.0f};    //gyro
float gyroscope[3] ={0.0f,0.0f,0.0f};             //gyro
float accelerometer[3] = {0.0f, 0.0f, 0.0f};      //acc
//--------------------------------------------------------------------------------------
//===================================================================================================

static void attitude_calibrate_gyro(void)
{
    uint16_t collected = 0;
    uint16_t attempts = 0;
    memset(gyroscopeOffset, 0, sizeof(gyroscopeOffset));

    while ((collected < GYRO_CALIBRATION_SAMPLES) && (attempts < GYRO_CALIBRATION_MAX_TRIES))
    {
        imu963ra_get_gyro();
        float gx = (imu963ra_gyro_x);
        float gy = (imu963ra_gyro_y);
        float gz = (imu963ra_gyro_z);

        if (fabsf(gx) + fabsf(gy) + fabsf(gz) < gyroscope_threshold)
        {
            gyroscopeOffset[0] += gx;
            gyroscopeOffset[1] += gy;
            gyroscopeOffset[2] += gz;
            collected++;
        }
        attempts++;
        system_delay_ms(2);
    }

    if (collected > 0)
    {
        gyroscopeOffset[0] /= collected;
        gyroscopeOffset[1] /= collected;
        gyroscopeOffset[2] /= collected;
    }
}


/**
 * @brief 		姿态解算初始化
 * @param       无
 * @note 		包含了底层的IMU_QuaternionEKF_Init，imu要单独初始化
 */
void Attitude_Init(void)
{
    IMU_QuaternionEKF_Init(100, 0.00001f, 100000000, 0.9996f, ATTITUDE_UPDATE_PERIOD_S, 0);
    eulerAngle.roll_offset = 0;
    attitude_calibrate_gyro();

    imu963ra_get_acc();
    imu963ra_get_gyro();
    gyroscope[0]= ((imu963ra_gyro_x) - gyroscopeOffset[0])/14.3f * DEG_TO_RAD;
    gyroscope[1]= ((imu963ra_gyro_y) - gyroscopeOffset[1])/14.3f * DEG_TO_RAD;
    gyroscope[2]= ((imu963ra_gyro_z) - gyroscopeOffset[2])/14.3f * DEG_TO_RAD;
    accelerometer[0]=(imu963ra_acc_x)/4096.0f;
    accelerometer[1]=(imu963ra_acc_y)/4096.0f;
    accelerometer[2]=(imu963ra_acc_z)/4096.0f;
}

#define cheat_define 0.008 //0.0008
/**
 * @brief 		姿态解算
 * @param       无
 * @note 		调用了各种底层，初步处理了原始数据再调用姿态解算相关函数，调用后读取attitude_euler_t结构里的值即可
 * @note        解算周期当前为2ms，可以通过修改#define ATTITUDE_UPDATE_PERIOD_S 0.002f宏定义修改
 */
void Attitude_Calculate(void)
{
    imu963ra_get_acc();
    imu963ra_get_gyro();
    gyroscope[0]= ((imu963ra_gyro_x) - gyroscopeOffset[0])/14.3f * DEG_TO_RAD;
    gyroscope[1]= ((imu963ra_gyro_y) - gyroscopeOffset[1])/14.3f * DEG_TO_RAD;
    gyroscope[2]= ((imu963ra_gyro_z) - gyroscopeOffset[2])/14.3f * DEG_TO_RAD;
    accelerometer[0]=(imu963ra_acc_x)/4096.0f;
    accelerometer[1]=(imu963ra_acc_y)/4096.0f;
    accelerometer[2]=(imu963ra_acc_z)/4096.0f;

    if(fabsf(gyroscope[0])<cheat_define) gyroscope[0]=0;       
    if(fabsf(gyroscope[1])<cheat_define) gyroscope[1]=0;
    if(fabsf(gyroscope[2])<cheat_define) gyroscope[2]=0;

    IMU_QuaternionEKF_Update(gyroscope[0],gyroscope[1],gyroscope[2],accelerometer[0],accelerometer[1],accelerometer[2]);

    eulerAngle.pitch = -QEKF_INS.Pitch;
    eulerAngle.yaw = QEKF_INS.Yaw;
    eulerAngle.roll =-(-eulerAngle.roll_offset+ QEKF_INS.Roll);

    icm_data.gyro_x= QEKF_INS.Gyro[0];
    icm_data.gyro_y = QEKF_INS.Gyro[1];
    icm_data.gyro_z = QEKF_INS.Gyro[2];

    icm_data.acc_x =QEKF_INS.Accel[0];
    icm_data.acc_y =QEKF_INS.Accel[1];
    icm_data.acc_z =QEKF_INS.Accel[2];
}
#pragma section all restore
