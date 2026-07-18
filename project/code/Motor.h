#ifndef __MOTOR_H_
#define __MOTOR_H_

#include "zf_common_typedef.h"

#define MOTOR1_DIR              (C7)                           //����
#define MOTOR1_PWM              (PWM2_MODULE0_CHA_C6)

#define MOTOR2_DIR              (C9)                          //����
#define MOTOR2_PWM              (PWM2_MODULE1_CHA_C8)

#define MOTOR3_DIR              (C11)                          //����
#define MOTOR3_PWM              (PWM2_MODULE2_CHA_C10)

#define MOTOR4_DIR              (D3)                           //����
#define MOTOR4_PWM              (PWM2_MODULE3_CHA_D2)

// Motor board selection:
// 0: old board pin/direction configuration.
// 1: new board wiring adaptation, enabled by default.
#define MOTOR_BOARD_USE_NEW          (0)

#if MOTOR_BOARD_USE_NEW
// Remap logical wheels UL, UR, DL, DR to the measured physical motor channels.
#define MOTOR_BOARD_REMAP_LOGICAL_WHEELS (1)
#define MOTOR_BOARD_REVERSE_ALL_DIR  (1)
// Extra logical-wheel direction correction for the new board.
#define MOTOR_BOARD_REVERSE_UL_DIR   (0)
#define MOTOR_BOARD_REVERSE_UR_DIR   (0)
#define MOTOR_BOARD_REVERSE_DL_DIR   (1)
#define MOTOR_BOARD_REVERSE_DR_DIR   (1)
#define MOTOR_BOARD_REVERSE_ENCODER_ALL_DIR (1)
#define MOTOR_BOARD_REVERSE_LEFT_ENCODERS (0)
#else
#define MOTOR_BOARD_REMAP_LOGICAL_WHEELS (0)
#define MOTOR_BOARD_REVERSE_ALL_DIR  (0)
#define MOTOR_BOARD_REVERSE_UL_DIR   (0)
#define MOTOR_BOARD_REVERSE_UR_DIR   (0)
#define MOTOR_BOARD_REVERSE_DL_DIR   (0)
#define MOTOR_BOARD_REVERSE_DR_DIR   (0)
#define MOTOR_BOARD_REVERSE_ENCODER_ALL_DIR (0)
#define MOTOR_BOARD_REVERSE_LEFT_ENCODERS (1)
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

//�����궨��
#define ENCODER_RESOLUTION      2390.0   //�������ֱ���, ����תһȦ��������������������
#define WHEEL_DIAMETER          0.06239  //����ֱ��,��λ����@
#define LATERAL_CORRECTION_FACTOR 0.901589f  //ʵ�ʺ��ƾ��� / �ƻ����ƾ���
#define LATERAL_TO_LONGITUDINAL_COUPLING_FACTOR 0.0f  // dx drift / dy travel
#define D_X                     0.176     //����Y�����������ĵļ��
#define D_Y                     0.20     //����X�����������ĵļ��
#define PID_RATE                100       //PID����PWMֵ��Ƶ��

#define LIMIT_PWM_MIN              -6000
#define LIMIT_PWM_MAX               6000

/* Closed-loop tuned dead-zone feedforward for the final motor driver board. */
#define MOTOR_DEADZONE_TARGET_MIN_COUNTS  2
#define MOTOR_UL_DEADZONE_FWD             420
#define MOTOR_UL_DEADZONE_REV             390
#define MOTOR_UR_DEADZONE_FWD             495
#define MOTOR_UR_DEADZONE_REV             429
#define MOTOR_DL_DEADZONE_FWD             435
#define MOTOR_DL_DEADZONE_REV             390
#define MOTOR_DR_DEADZONE_FWD             550
#define MOTOR_DR_DEADZONE_REV             637

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
