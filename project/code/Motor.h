#ifndef __MOTOR_H_
#define __MOTOR_H_

#include "zf_common_typedef.h"

#define MOTOR1_DIR              (C7)                           //上左
#define MOTOR1_PWM              (PWM2_MODULE0_CHA_C6)

#define MOTOR2_DIR              (C9)                          //上右
#define MOTOR2_PWM              (PWM2_MODULE1_CHA_C8)

#define MOTOR3_DIR              (C11)                          //下左
#define MOTOR3_PWM              (PWM2_MODULE2_CHA_C10)

#define MOTOR4_DIR              (D3)                           //下右
#define MOTOR4_PWM              (PWM2_MODULE3_CHA_D2)

// Motor board selection:
// 0: old board pin/direction configuration.
// 1: new board wiring adaptation, enabled by default.
#define MOTOR_BOARD_USE_NEW          (1)

#if MOTOR_BOARD_USE_NEW
// Measured order was 2,3,4,1 when commands were sent as 1,2,3,4.
#define MOTOR_BOARD_REMAP_ORDER_2341 (1)
#define MOTOR_BOARD_REVERSE_ALL_DIR  (1)
// Extra logical-wheel direction correction for the new board.
#define MOTOR_BOARD_REVERSE_UL_DIR   (1)
#define MOTOR_BOARD_REVERSE_UR_DIR   (0)
#define MOTOR_BOARD_REVERSE_DL_DIR   (0)
#define MOTOR_BOARD_REVERSE_DR_DIR   (1)
#define MOTOR_BOARD_REVERSE_ENCODER_ALL_DIR (1)
#else
#define MOTOR_BOARD_REMAP_ORDER_2341 (0)
#define MOTOR_BOARD_REVERSE_ALL_DIR  (0)
#define MOTOR_BOARD_REVERSE_UL_DIR   (0)
#define MOTOR_BOARD_REVERSE_UR_DIR   (0)
#define MOTOR_BOARD_REVERSE_DL_DIR   (0)
#define MOTOR_BOARD_REVERSE_DR_DIR   (0)
#define MOTOR_BOARD_REVERSE_ENCODER_ALL_DIR (0)
#endif


#define ENCODER_1                   (QTIMER1_ENCODER2)
#define ENCODER_1_A                 (QTIMER1_ENCODER2_CH1_C2)
#define ENCODER_1_B                 (QTIMER1_ENCODER2_CH2_C24)

#define ENCODER_2                   (QTIMER2_ENCODER2)
#define ENCODER_2_A                 (QTIMER2_ENCODER2_CH1_C5)
#define ENCODER_2_B                 (QTIMER2_ENCODER2_CH2_C25)

#define ENCODER_3                   (QTIMER2_ENCODER1)
#define ENCODER_3_A                 (QTIMER2_ENCODER1_CH1_C3)
#define ENCODER_3_B                 (QTIMER2_ENCODER1_CH2_C4)

#define ENCODER_4                   (QTIMER1_ENCODER1)
#define ENCODER_4_A                 (QTIMER1_ENCODER1_CH1_C0)
#define ENCODER_4_B                 (QTIMER1_ENCODER1_CH2_C1)

//参数宏定义
#define ENCODER_RESOLUTION      2390.0   //编码器分辨率, 轮子转一圈，编码器产生的脉冲数
#define WHEEL_DIAMETER          0.06239  //轮子直径,单位：米@
#define LINEAR_DISTANCE_CORRECTION_FACTOR 1.009406f  // 上位机地图实测线性距离修正，约 12080 pulse/m
#define LATERAL_CORRECTION_FACTOR 0.940000f  //实际横移距离 / 计划横移距离
#define LATERAL_TO_LONGITUDINAL_COUPLING_FACTOR (-0.029000f)  // signed dx drift / dy travel
#define LATERAL_TO_LONGITUDINAL_BIAS_FACTOR 0.000000f  // direction-independent dx drift / abs(dy)
#define D_X                     0.176     //底盘Y轴上两轮中心的间距
#define D_Y                     0.20     //底盘X轴上两轮中心的间距
#define PID_RATE                100       //PID调节PWM值的频率

#define LIMIT_PWM_MIN              -6000
#define LIMIT_PWM_MAX               6000

/* Measured no-load start thresholds, using 85% as conservative feedforward. */
#define MOTOR_DEADZONE_TARGET_MIN_COUNTS  2
#define MOTOR_UL_DEADZONE_FWD             410
#define MOTOR_UL_DEADZONE_REV             410
#define MOTOR_UR_DEADZONE_FWD             490
#define MOTOR_UR_DEADZONE_REV             510
#define MOTOR_DL_DEADZONE_FWD             340
#define MOTOR_DL_DEADZONE_REV             340
#define MOTOR_DR_DEADZONE_FWD             600
#define MOTOR_DR_DEADZONE_REV             680

#define LIMIT_ENCODER_MIN          -500
#define LIMIT_ENCODER_MAX           500
#define ENCODER_FILTER_ALPHA       0.35f


extern int all;
extern int16 up_L_all;
extern int16 down_L_all;
extern int16 up_R_all;
extern int16 down_R_all;

extern int32 encoder_all;
extern int16 encoders_average;
extern int16 encoder_data_quaddec1;
extern int16 encoder_data_quaddec2;
extern int16 encoder_data_quaddec3;
extern int16 encoder_data_quaddec4;
extern double pulse_per_meter;
extern float rx_plus_ry_cali;
extern float motor_lateral_correction_factor;
extern float motor_lateral_to_longitudinal_coupling_factor;
extern float motor_lateral_to_longitudinal_bias_factor;
extern float motor_rotation_radius_m;

extern float speed_three_array[3];
extern int speed_encoder[4];
extern int car_stop_array[4];

void motor_init(void);
void encoder_init(void);
void encoder_get(void);
int Limit_int(int left_limit, int target_num, int right_limit);
void motor_pwm(int up_left_speed,int up_right_speed,int down_left_speed,int down_right_speed);
void motor_control(int* input_speed_encoder);
//void encoder_read_filtered(int *enc1, int *enc2, int *enc3, int *enc4);
int16 Lowpass(int16 X_last,int16 X_new);
void Kinematics_Init(void);
void Kinematics_Inverse(float* input, int* output);

#endif

