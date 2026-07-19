/*
 * UART8 蓝牙串口调参和遥控模块。
 *
 * 兼容的蓝牙帧格式：
 *   [slider,name,value]  滑条调参，例如 [slider,speed,40]
 *   [slider,line.kp,value] / [slider,line.min,value] 调节运行段法向纠偏。
 *   [slider,posf.kp,value] / [slider,posb.kp,value] 分别调前进/后退位置环。
 *   [button_name]        按钮命令，例如 [forward]、[start]、[stop]
 *   [button,name]        兼容部分小程序发送的 button 前缀格式
 *
 * 直线遥控的使用顺序：
 *   [forward] / [backward] / [left] / [right] 选择下一次运动方向；
 *   [slider,speed,value] 和 [slider,position,value] 设置速度和距离；
 *   [start]（也兼容 run/apply/move）提交一次按目标位置停止的 S 曲线相对位移动作；
 *   [speed]（也兼容 cruise/speed.start）按所选方向和目标速度持续匀速行驶，直到 stop。
 * 中断适配原则：
 *   UART8 ISR 只负责收字节、拼帧、入队；字符串解析、printf、路径控制、电机状态切换
 *   都在主循环或 10ms 控制任务里完成，避免串口中断阻塞 PIT 控制中断。
 */

#include "zf_common_headfile.h"
#include "zf_driver_uart.h"
#include "data_handle.h"
#include "path.h"
#include "path_follow.h"
#include "Motor.h"
#include "PID_config.h"
#include "Mymenu.h"
#include "Attitude.h"
#include "Control.h"
#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.1415926f
#endif

#define BLUESERIAL_UART                         UART_8
#define BLUESERIAL_BAUDRATE                     115200U
#define BLUESERIAL_TX_PIN                       UART8_TX_D16
#define BLUESERIAL_RX_PIN                       UART8_RX_D17
#define BLUESERIAL_IRQN                         LPUART8_IRQn
/* UART8 优先级低于 PIT 控制中断，蓝牙突发数据不会抢占底盘闭环。 */
#define BLUESERIAL_IRQ_PRIORITY                 8U
#define BLUESERIAL_RX_FRAME_LEN                 80U
#define BLUESERIAL_RX_QUEUE_LEN                 16U
#define BLUESERIAL_TELEMETRY_PERIOD_TICKS       10U
#define BLUESERIAL_MAX_TARGET_SPEED_CMPS        200.0f
#define BLUESERIAL_MAX_TARGET_POSITION_CM       500.0f
#define BLUESERIAL_MAX_YAW_KP                   100.0f
#define BLUESERIAL_MAX_YAW_KI                   10.0f
#define BLUESERIAL_MAX_YAW_KD                   200.0f
#define BLUESERIAL_MAX_YAW_FF_DEGPS             120.0f
#define BLUESERIAL_MAX_Y_CROSSTALK_ABS          0.100f
#define BLUESERIAL_MAX_LINE_GUIDE_KP             4.0f
#define BLUESERIAL_MAX_LINE_GUIDE_MIN_CMPS       3.0f
#define BLUESERIAL_MAX_WHEEL_KP                  30.0f
#define BLUESERIAL_MAX_WHEEL_KI                   3.0f
#define BLUESERIAL_MAX_WHEEL_KD                  60.0f
#define BLUESERIAL_MAX_DEADZONE_PWM            1200
#define BLUESERIAL_MAX_DEADZONE_MIN_COUNTS       20
#define BLUESERIAL_SPEED_LOG_CAPACITY            600U
#define BLUESERIAL_SPEED_LOG_HEAD_COUNT           200U
#define BLUESERIAL_SPEED_LOG_TAIL_COUNT           (BLUESERIAL_SPEED_LOG_CAPACITY - BLUESERIAL_SPEED_LOG_HEAD_COUNT)
#define BLUESERIAL_MAX_POSITION_KP              20.0f
#define BLUESERIAL_MAX_POSITION_KI              10.0f
#define BLUESERIAL_MAX_POSITION_KD              50.0f
#define BLUESERIAL_MIN_POSITION_ENVELOPE        0.10f
#define BLUESERIAL_MAX_POSITION_ENVELOPE        1.00f
#define BLUESERIAL_RELATIVE_GRID_M              0.01f
#define BLUESERIAL_RELATIVE_CENTER_CELL         127
#define BLUESERIAL_RELATIVE_MAX_OFFSET_CELL     120
#define BLUESERIAL_POSITION_RETRY_LOOPS         200U
#define BLUESERIAL_POSITION_MAX_RETRY           3U
#define BLUESERIAL_VISION_TIMEOUT_LOOPS          1000U
#define BLUESERIAL_POINT_TARGET_GRID_M          0.01f
#define BLUESERIAL_POINT_TARGET_MAX_CELL        250
#define BLUESERIAL_POINT_TARGET_MAX_M           20.0f

typedef enum
{
    BLUESERIAL_MODE_STOP = 0,      /* 蓝牙未接管底盘输出。 */
    BLUESERIAL_MODE_RAW_PWM,       /* 蓝牙直接下发四轮 PWM，用于电机方向/接线检查。 */
    BLUESERIAL_MODE_PATH_MOTION,   /* 目标位置模式：由 path_follow 完成 S 曲线启停。 */
    BLUESERIAL_MODE_SPEED_CRUISE   /* 目标速度模式：按所选方向持续输出速度，不创建位置目标。 */
} blueserial_control_mode_t;

typedef enum
{
    BLUESERIAL_MOVE_FORWARD = 0,
    BLUESERIAL_MOVE_BACKWARD,
    BLUESERIAL_MOVE_LEFT,
    BLUESERIAL_MOVE_RIGHT
} blueserial_move_direction_t;

typedef struct
{
    uint16 tick;
    int16 target[MOTOR_WHEEL_COUNT];
    int16 actual[MOTOR_WHEEL_COUNT];
    int16 raw[MOTOR_WHEEL_COUNT];
    int16 pid_pwm[MOTOR_WHEEL_COUNT];
    int16 final_pwm[MOTOR_WHEEL_COUNT];
    int16 yaw_delta_cdeg;
    int16 gyro_z_cdegps;
} blueserial_speed_sample_t;

static volatile uint8 g_rx_collecting = 0U;
static volatile uint8 g_enabled = 1U;
static uint8 g_initialized = 0U;
static volatile uint8 g_rx_work_len = 0U;
static char g_rx_work[BLUESERIAL_RX_FRAME_LEN];
static char g_rx_queue[BLUESERIAL_RX_QUEUE_LEN][BLUESERIAL_RX_FRAME_LEN];
static volatile uint8 g_rx_queue_write = 0U;
static volatile uint8 g_rx_queue_read = 0U;
static volatile uint32 g_rx_drop_count = 0U;
static char g_last_rx_frame[BLUESERIAL_RX_FRAME_LEN] = "";
static volatile uint8 g_last_rx_frame_len = 0U;

static volatile blueserial_control_mode_t g_control_mode = BLUESERIAL_MODE_STOP;
/* 逻辑轮序与 Motor.c 的 motor_pwm() 参数一致：UL, UR, DL, DR。 */
static volatile int g_raw_pwm[4] = {0, 0, 0, 0};
static float g_target_speed_cmps = 40.0f;
static float g_target_position_cm = 50.0f;
static blueserial_move_direction_t g_next_move_direction = BLUESERIAL_MOVE_FORWARD;
static blueserial_move_direction_t g_cruise_move_direction = BLUESERIAL_MOVE_FORWARD;
static uint8 g_turn_quarters_remaining = 0U;
static float g_turn_step_deg = 90.0f;
static volatile uint8 g_yaw_hold_enabled = 0U;
static Position g_blueserial_relative_path[3];
static uint8 g_position_request_pending = 0U;
static uint8 g_position_request_start_frame = 0U;
static uint8 g_position_request_retry_count = 0U;
static uint16 g_position_request_wait_loops = 0U;
static uint8 g_vision_request_pending = 0U;
static uint16 g_vision_request_wait_loops = 0U;
static VisionRecognitionType g_vision_request_type = VISION_RECOGNITION_NONE;
static uint8 g_point_target_valid = 0U;
static float g_point_target_x_m = 0.0f;
static float g_point_target_y_m = 0.0f;
static Position g_blueserial_point_target_path[3];
static volatile uint8 g_telemetry_tick_count = 0U;
static volatile uint8 g_telemetry_report_pending = 0U;
static volatile uint32 g_telemetry_sequence = 0U;
static uint8 g_telemetry_was_active = 0U;
static blueserial_speed_sample_t g_speed_log[BLUESERIAL_SPEED_LOG_CAPACITY];
static motor_speed_debug_snapshot_t g_speed_log_final_motor;
static volatile uint16 g_speed_log_count = 0U;
static uint16 g_speed_log_tail_write = 0U;
static uint16 g_speed_log_dump_index = 0U;
static volatile uint32 g_speed_log_total_ticks = 0U;
static volatile uint8 g_speed_log_recording = 0U;
static uint8 g_speed_log_ready = 0U;
static uint8 g_speed_log_dumping = 0U;
static volatile uint8 g_speed_log_overflow = 0U;
static uint8 g_speed_log_enabled = 0U;
static uint8 g_speed_summary_pending = 0U;
static uint8 g_speed_log_aborted = 0U;
static uint8 g_speed_log_settle_timeout = 0U;
static uint8 g_speed_log_profile_fault = 0U;
static uint8 g_speed_test_profile_enabled = 0U;
static volatile uint8 g_speed_test_yaw_enabled = 0U;
static uint32 g_speed_run_id = 0U;
static uint32 g_speed_log_run_id = 0U;
static uint32 g_speed_cfg_id = 0U;
static uint32 g_speed_log_cfg_id = 0U;
static uint32 g_speed_log_abs_error_sum[MOTOR_WHEEL_COUNT];
static uint32 g_speed_log_abs_target_sum[MOTOR_WHEEL_COUNT];
static uint32 g_speed_log_saturation_ticks[MOTOR_WHEEL_COUNT];
static int g_speed_log_peak_error[MOTOR_WHEEL_COUNT];
static float g_speed_log_peak_axis_cm = 0.0f;
static float g_speed_log_profile_total_s = 0.0f;
static float g_speed_log_start_yaw_deg = 0.0f;
static float g_speed_log_end_yaw_deg = 0.0f;
static float g_speed_log_peak_yaw_abs_deg = 0.0f;
static float g_speed_log_requested_cm = 0.0f;
static float g_speed_log_requested_speed_cmps = 0.0f;
static blueserial_move_direction_t g_speed_log_direction = BLUESERIAL_MOVE_FORWARD;

static void BlueSerial_CountsToBodyCm(const int32 counts[MOTOR_WHEEL_COUNT],
                                      float *x_cm,
                                      float *y_cm);
static void BlueSerial_ProjectBodyCm(blueserial_move_direction_t direction,
                                     float x_cm,
                                     float y_cm,
                                     float *axis_cm,
                                     float *cross_cm);
static tagPID_T *BlueSerial_GetWheelPid(uint8 wheel_index);

static float BlueSerial_ClampFloat(float value, float min_value, float max_value)
{
    if (value < min_value)
    {
        return min_value;
    }
    if (value > max_value)
    {
        return max_value;
    }
    return value;
}

static int BlueSerial_ClampInt(int value, int min_value, int max_value)
{
    if (value < min_value)
    {
        return min_value;
    }
    if (value > max_value)
    {
        return max_value;
    }
    return value;
}

static int BlueSerial_RoundToInt(float value)
{
    return (int)(value + ((value >= 0.0f) ? 0.5f : -0.5f));
}

static int16 BlueSerial_ClampInt16(int value)
{
    if (value > 32767)
    {
        return 32767;
    }
    if (value < -32768)
    {
        return -32768;
    }
    return (int16)value;
}

static float BlueSerial_WrapDeg(float angle_deg)
{
    while (angle_deg > 180.0f)
    {
        angle_deg -= 360.0f;
    }
    while (angle_deg < -180.0f)
    {
        angle_deg += 360.0f;
    }
    return angle_deg;
}

static void BlueSerial_SpeedLogResetLocked(void)
{
    uint8 i;

    g_speed_log_count = 0U;
    g_speed_log_tail_write = 0U;
    g_speed_log_dump_index = 0U;
    g_speed_log_total_ticks = 0U;
    g_speed_log_recording = 0U;
    g_speed_log_ready = 0U;
    g_speed_log_dumping = 0U;
    g_speed_log_overflow = 0U;
    g_speed_summary_pending = 0U;
    g_speed_log_aborted = 0U;
    g_speed_log_settle_timeout = 0U;
    g_speed_log_profile_fault = 0U;
    g_speed_log_peak_yaw_abs_deg = 0.0f;
    g_speed_log_peak_axis_cm = 0.0f;
    g_speed_log_profile_total_s = 0.0f;
    for (i = 0U; i < MOTOR_WHEEL_COUNT; ++i)
    {
        g_speed_log_abs_error_sum[i] = 0U;
        g_speed_log_abs_target_sum[i] = 0U;
        g_speed_log_saturation_ticks[i] = 0U;
        g_speed_log_peak_error[i] = 0;
    }
    memset(&g_speed_log_final_motor, 0, sizeof(g_speed_log_final_motor));
    motor_speed_debug_reset();
}

static void BlueSerial_SpeedLogBeginLocked(blueserial_move_direction_t direction,
                                           float requested_cm,
                                           float start_yaw_deg)
{
    BlueSerial_SpeedLogResetLocked();
    g_speed_run_id++;
    g_speed_log_run_id = g_speed_run_id;
    g_speed_log_cfg_id = g_speed_cfg_id;
    g_speed_log_direction = direction;
    g_speed_log_requested_cm = requested_cm;
    g_speed_log_requested_speed_cmps = g_target_speed_cmps;
    g_speed_log_start_yaw_deg = start_yaw_deg;
    g_speed_log_end_yaw_deg = start_yaw_deg;
    if (!g_speed_log_enabled)
    {
        return;
    }

    g_speed_log_recording = 1U;
}

static void BlueSerial_SpeedLogCapture10ms(void)
{
    motor_speed_debug_snapshot_t motor = {0};
    path_follow_status_t status = {0};
    blueserial_speed_sample_t *sample;
    float yaw_delta_deg;
    float x_cm;
    float y_cm;
    float axis_cm;
    float unused_cross_cm;
    uint16 slot;
    uint8 i;

    if (!g_speed_log_recording)
    {
        return;
    }
    motor_speed_debug_get_snapshot(&motor);
    path_follow_get_status(&status);
    g_speed_log_total_ticks++;
    for (i = 0U; i < MOTOR_WHEEL_COUNT; ++i)
    {
        int error = motor.target_counts[i] - motor.filtered_counts[i];
        int abs_error = (error >= 0) ? error : -error;
        int abs_target = (motor.target_counts[i] >= 0) ?
                         motor.target_counts[i] : -motor.target_counts[i];
        int abs_pwm = (motor.final_pwm[i] >= 0) ?
                      motor.final_pwm[i] : -motor.final_pwm[i];

        g_speed_log_abs_error_sum[i] += (uint32)abs_error;
        g_speed_log_abs_target_sum[i] += (uint32)abs_target;
        if (abs_error > g_speed_log_peak_error[i])
        {
            g_speed_log_peak_error[i] = abs_error;
        }
        if (abs_pwm >= LIMIT_PWM_MAX)
        {
            g_speed_log_saturation_ticks[i]++;
        }
    }
    yaw_delta_deg = BlueSerial_WrapDeg(eulerAngle.yaw - g_speed_log_start_yaw_deg);
    g_speed_log_peak_yaw_abs_deg =
        fmaxf(g_speed_log_peak_yaw_abs_deg, fabsf(yaw_delta_deg));
    if (status.speed_test_profile_total_s > 0.0f)
    {
        g_speed_log_profile_total_s = status.speed_test_profile_total_s;
    }
    BlueSerial_CountsToBodyCm(motor.cumulative_raw_counts, &x_cm, &y_cm);
    BlueSerial_ProjectBodyCm(g_speed_log_direction, x_cm, y_cm,
                             &axis_cm, &unused_cross_cm);
    g_speed_log_peak_axis_cm = fmaxf(g_speed_log_peak_axis_cm, axis_cm);

    if (g_speed_log_total_ticks <= BLUESERIAL_SPEED_LOG_HEAD_COUNT)
    {
        slot = (uint16)(g_speed_log_total_ticks - 1U);
    }
    else
    {
        slot = (uint16)(BLUESERIAL_SPEED_LOG_HEAD_COUNT + g_speed_log_tail_write);
        g_speed_log_tail_write = (uint16)((g_speed_log_tail_write + 1U) %
                                           BLUESERIAL_SPEED_LOG_TAIL_COUNT);
    }
    if (g_speed_log_count < BLUESERIAL_SPEED_LOG_CAPACITY)
    {
        g_speed_log_count++;
    }
    else
    {
        g_speed_log_overflow = 1U;
    }

    sample = &g_speed_log[slot];
    sample->tick = (g_speed_log_total_ticks <= 65535U) ?
                   (uint16)g_speed_log_total_ticks : 65535U;
    for (i = 0U; i < MOTOR_WHEEL_COUNT; ++i)
    {
        sample->target[i] = BlueSerial_ClampInt16(motor.target_counts[i]);
        sample->actual[i] = BlueSerial_ClampInt16(motor.filtered_counts[i]);
        sample->raw[i] = BlueSerial_ClampInt16(motor.raw_counts[i]);
        sample->pid_pwm[i] = BlueSerial_ClampInt16(motor.pid_pwm[i]);
        sample->final_pwm[i] = BlueSerial_ClampInt16(motor.final_pwm[i]);
    }
    sample->yaw_delta_cdeg = BlueSerial_ClampInt16(
        BlueSerial_RoundToInt(yaw_delta_deg * 100.0f));
    sample->gyro_z_cdegps = BlueSerial_ClampInt16(
        BlueSerial_RoundToInt(gyroscope[2] * (180.0f / (float)M_PI) * 100.0f));
}

static void BlueSerial_SpeedLogFinishLocked(uint8 aborted)
{
    path_follow_status_t status = {0};

    if (!g_speed_log_recording)
    {
        return;
    }

    motor_speed_debug_get_snapshot(&g_speed_log_final_motor);
    path_follow_get_status(&status);
    g_speed_log_end_yaw_deg = eulerAngle.yaw;
    g_speed_log_aborted = aborted ? 1U : 0U;
    g_speed_log_settle_timeout = status.speed_test_settle_timeout;
    g_speed_log_profile_fault = status.speed_test_profile_fault;
    g_speed_log_recording = 0U;
    g_speed_log_ready = (g_speed_log_count > 0U) ? 1U : 0U;
    g_speed_summary_pending = g_speed_log_ready;
}

static void BlueSerial_ClearWheelPid(void)
{
    PID_Clear(&ULpid);
    PID_Clear(&URpid);
    PID_Clear(&DLpid);
    PID_Clear(&DRpid);
#if !MOTOR_BOARD_USE_NEW
    motor_control_reset_state();
#endif
}

static void BlueSerial_ZeroSpeedCommand(void)
{
    speed_three_array[0] = 0.0f;
    speed_three_array[1] = 0.0f;
    speed_three_array[2] = 0.0f;
    speed_encoder[0] = 0;
    speed_encoder[1] = 0;
    speed_encoder[2] = 0;
    speed_encoder[3] = 0;
}

/*
 * 蓝牙侧保持“逻辑轮序”不变。新旧电机板的物理通道重映射、方向反向和编码器修正
 * 全部集中在 Motor.c 内部处理，避免蓝牙模块再维护一套容易跑偏的映射关系。
 */
static void BlueSerial_ApplyRawPwm(void)
{
    motor_pwm(g_raw_pwm[0], g_raw_pwm[1], g_raw_pwm[2], g_raw_pwm[3]);
}

/* 调用者需要在关中断状态下进入，避免 10ms 控制中断读到半更新状态。 */
static void BlueSerial_StopControlLocked(void)
{
    g_control_mode = BLUESERIAL_MODE_STOP;
    g_turn_quarters_remaining = 0U;
    g_yaw_hold_enabled = 0U;
    path_follow_set_path(NULL, 0U);
    path_follow_set_speed_test_mode(0U, 0U);
    path_follow_set_stationary_yaw_hold_enabled(0U);
    car_go_flag = 0U;
    car_stop_flag = 0U;
    BlueSerial_ZeroSpeedCommand();
    BlueSerial_ClearWheelPid();
    PID_Clear(&pid_yaw);
    g_raw_pwm[0] = 0;
    g_raw_pwm[1] = 0;
    g_raw_pwm[2] = 0;
    g_raw_pwm[3] = 0;
    motor_pwm(0, 0, 0, 0);
}

static void BlueSerial_StopControl(void)
{
    uint32 primask = interrupt_global_disable();

    BlueSerial_SpeedLogFinishLocked(1U);
    BlueSerial_StopControlLocked();
    interrupt_global_enable(primask);
}

uint8 BlueSerial_IsControlActive(void)
{
    uint32 primask;
    uint8 active;

    primask = interrupt_global_disable();
    active = (g_enabled &&
              (g_control_mode != BLUESERIAL_MODE_STOP || g_yaw_hold_enabled)) ? 1U : 0U;
    interrupt_global_enable(primask);
    return active;
}

static void BlueSerial_ToggleYawHold(void)
{
    uint32 primask;
    uint8 enabled;
    float hold_yaw_deg = 0.0f;
    path_follow_status_t status = {0};

    primask = interrupt_global_disable();
    if (!g_yaw_hold_enabled && g_control_mode != BLUESERIAL_MODE_STOP)
    {
        interrupt_global_enable(primask);
        BlueSerial_Printf("ERR stop_first yawhold\r\n");
        return;
    }
    if (g_yaw_hold_enabled)
    {
        BlueSerial_StopControlLocked();
        enabled = 0U;
    }
    else
    {
        /* Keep the existing cardinal target; measured yaw must not become a
         * new heading command. */
        path_follow_get_status(&status);
        hold_yaw_deg = status.target_yaw_deg;
        path_follow_set_path(NULL, 0U);
        g_control_mode = BLUESERIAL_MODE_STOP;
        g_turn_quarters_remaining = 0U;
        BlueSerial_ZeroSpeedCommand();
        BlueSerial_ClearWheelPid();
        PID_Clear(&pid_yaw);
        path_follow_set_target_yaw(hold_yaw_deg);
        path_follow_set_stationary_yaw_hold_enabled(1U);
        g_yaw_hold_enabled = 1U;
        car_go_flag = 1U;
        car_stop_flag = 0U;
        enabled = 1U;
    }
    interrupt_global_enable(primask);

    if (enabled)
    {
        BlueSerial_Printf("OK yawhold=on target=%.2fdeg\r\n", hold_yaw_deg);
    }
    else
    {
        BlueSerial_Printf("OK yawhold=off\r\n");
    }
}

static void BlueSerial_ApplyConfiguredPathSpeedLocked(void)
{
    uint8 i;
    float speed_mps = g_target_speed_cmps * 0.01f;

    if (speed_mps <= 0.0f)
    {
        return;
    }

    for (i = 0U; i < PATH_FOLLOW_SCURVE_BAND_COUNT; ++i)
    {
        g_path_follow_scurve_band_cfg[i].vmax_mps = speed_mps;
    }
}

static void BlueSerial_EnterPathMode(void)
{
    if (!g_yaw_hold_enabled)
    {
        path_follow_set_stationary_yaw_hold_enabled(0U);
    }
    g_control_mode = BLUESERIAL_MODE_PATH_MOTION;
    car_go_flag = 1U;
    car_stop_flag = 0U;
    BlueSerial_ZeroSpeedCommand();
    BlueSerial_ClearWheelPid();
}

/*
 * 目标速度模式不创建路径，也不调用 S 曲线规划。
 * 四轮速度 PID 会把实际车速拉到 g_target_speed_cmps；启动时锁定此前选择的方向，path_follow 在这里
 * 仅提供航向保持角速度，避免直行过程中逐渐偏航。
 */
static uint8 BlueSerial_StartSpeedCruise(void)
{
    uint32 primask;
    float hold_yaw_deg;
    path_follow_status_t status = {0};

    if (g_target_speed_cmps <= 0.0f)
    {
        return 0U;
    }

    primask = interrupt_global_disable();
    path_follow_get_status(&status);
    hold_yaw_deg = status.target_yaw_deg;
    g_cruise_move_direction = g_next_move_direction;
    g_turn_quarters_remaining = 0U;
    g_yaw_hold_enabled = 0U;
    path_follow_set_path(NULL, 0U);
    path_follow_set_speed_test_mode(0U, 0U);
    path_follow_set_stationary_yaw_hold_enabled(0U);
    BlueSerial_ZeroSpeedCommand();
    BlueSerial_ClearWheelPid();
    PID_Clear(&pid_yaw);
    BlueSerial_SpeedLogBeginLocked(g_cruise_move_direction, 0.0f, hold_yaw_deg);
    path_follow_set_target_yaw(hold_yaw_deg);
    path_follow_set_stationary_yaw_hold_enabled(g_speed_test_yaw_enabled);
    g_control_mode = BLUESERIAL_MODE_SPEED_CRUISE;
    car_go_flag = 1U;
    car_stop_flag = 0U;
    interrupt_global_enable(primask);
    return 1U;
}

static const char *BlueSerial_MoveDirectionName(blueserial_move_direction_t direction)
{
    switch (direction)
    {
        case BLUESERIAL_MOVE_FORWARD:
            return "forward";
        case BLUESERIAL_MOVE_BACKWARD:
            return "backward";
        case BLUESERIAL_MOVE_LEFT:
            return "left";
        case BLUESERIAL_MOVE_RIGHT:
            return "right";
        default:
            return "unknown";
    }
}

static void BlueSerial_SelectMoveDirection(blueserial_move_direction_t direction)
{
    uint32 primask = interrupt_global_disable();

    g_next_move_direction = direction;
    g_point_target_valid = 0U;
    interrupt_global_enable(primask);
    BlueSerial_Printf("OK direction=%s next_speed=%.1fcmps next_position=%.1fcm\r\n",
                      BlueSerial_MoveDirectionName(direction),
                      g_target_speed_cmps,
                      g_target_position_cm);
}

static uint8 BlueSerial_ConvertVisionPoseToMeter(const CarPose *pose, float *x_m, float *y_m)
{
    float row_f;
    float col_f;

    if (pose == NULL || x_m == NULL || y_m == NULL)
    {
        return 0U;
    }

    /*
     * 视觉端的车姿坐标和 path_follow 的执行坐标不是同一套轴定义。
     * 这里复用 path.h 里的补偿开关，保持蓝牙上报值和控制状态机里的视觉定位值一致。
     */
#if PATH_COORD_TRANSPOSE_COMPENSATE
    row_f = pose->x;
    col_f = pose->y;
#else
    row_f = pose->y;
    col_f = pose->x;
#endif

#if PATH_COORD_FLIP_VERTICAL
    col_f = (float)(MAP_ROWS - 1) - col_f;
#endif

    *x_m = row_f * GRID_SIZE_M;
    *y_m = col_f * GRID_SIZE_M;
    return 1U;
}

static void BlueSerial_PrintPositionReport(uint8 vision_valid)
{
    uint32 primask;
    path_follow_status_t odom = {0};
    CarPose vision_pose = {0};
    float vision_x_m = 0.0f;
    float vision_y_m = 0.0f;

    primask = interrupt_global_disable();
    path_follow_get_status(&odom);
    interrupt_global_enable(primask);

    vision_pose = car_pose;
    vision_valid = (vision_valid && car_pose_ready) ? 1U : 0U;
    if (vision_valid)
    {
        vision_valid = BlueSerial_ConvertVisionPoseToMeter(&vision_pose, &vision_x_m, &vision_y_m);
    }

    if (vision_valid)
    {
        BlueSerial_Printf("POSITION vision_grid=%.2f,%.2f,%.2f vision_m=%.3f,%.3f "
                          "odom_m=%.3f,%.3f,%.2f\r\n",
                          vision_pose.x,
                          vision_pose.y,
                          vision_pose.yaw,
                          vision_x_m,
                          vision_y_m,
                          odom.x_m,
                          odom.y_m,
                          odom.yaw_deg);
    }
    else
    {
        BlueSerial_Printf("POSITION vision=NA odom_m=%.3f,%.3f,%.2f\r\n",
                          odom.x_m,
                          odom.y_m,
                          odom.yaw_deg);
    }
}

/*
 * Reuse the same POSITION payload as the [position] command for autonomous
 * visual-localization updates. This only reports the cached valid pose; it
 * never sends CARPOS or changes chassis control state.
 */
void BlueSerial_ReportPosition(void)
{
    BlueSerial_PrintPositionReport(car_pose_ready ? 1U : 0U);
}

static void BlueSerial_StartPositionRequest(void)
{
    /*
     * 先消费 UART1 已到达的旧数据，再记录帧计数并发送新请求。
     * 后续只认 car_frame_count 增量，避免把上一次缓存的视觉定位误当成新结果。
     */
    process_blob_data();
    g_position_request_start_frame = car_frame_count;
    g_position_request_retry_count = 0U;
    g_position_request_wait_loops = 0U;
    g_position_request_pending = 1U;
    uart_send_car_request();
    BlueSerial_Printf("OK position_request\r\n");
}

static void BlueSerial_ServicePositionRequest(void)
{
    if (!g_position_request_pending)
    {
        return;
    }

    process_blob_data();
    if (car_pose_ready && car_frame_count != g_position_request_start_frame)
    {
        g_position_request_pending = 0U;
        BlueSerial_PrintPositionReport(1U);
        return;
    }

    if (g_position_request_wait_loops < BLUESERIAL_POSITION_RETRY_LOOPS)
    {
        g_position_request_wait_loops++;
        return;
    }

    g_position_request_wait_loops = 0U;
    if (g_position_request_retry_count < BLUESERIAL_POSITION_MAX_RETRY)
    {
        g_position_request_retry_count++;
        g_position_request_start_frame = car_frame_count;
        uart_send_car_request();
        return;
    }

    g_position_request_pending = 0U;
    BlueSerial_Printf("ERR position_timeout\r\n");
    BlueSerial_PrintPositionReport(0U);
}

static const char *BlueSerial_VisionTypeName(VisionRecognitionType type)
{
    return (type == VISION_RECOGNITION_NUM) ? "NUM" : "IMG";
}

static uint8 BlueSerial_VisionDistanceGridCount(VisionRecognitionDistance distance)
{
    return (distance == VISION_RECOGNITION_DISTANCE_THREE_GRID) ? 3U : 1U;
}

static void BlueSerial_PrintVisionResult(VisionRecognitionType type,
                                         const VisionRecognitionResult *result)
{
    if (result == NULL)
    {
        BlueSerial_Printf("VISION type=%s success=0 label=? score=-1 mode=0\r\n",
                          BlueSerial_VisionTypeName(type));
        return;
    }

    BlueSerial_Printf("VISION type=%s success=%u label=%s score=%d mode=%u\r\n",
                      BlueSerial_VisionTypeName(type),
                      result->success ? 1U : 0U,
                      result->label,
                      (int)result->score,
                      result->mode_marker ? 1U : 0U);
}

static void BlueSerial_StartVisionRequest(VisionRecognitionType type,
                                          VisionRecognitionDistance distance)
{
    if (g_vision_request_pending)
    {
        BlueSerial_Printf("ERR vision_busy\r\n");
        return;
    }

    if (control_get_stage() == CONTROL_STAGE_IDENTIFY_EXECUTE_PATH)
    {
        BlueSerial_Printf("ERR vision_busy_auto\r\n");
        return;
    }

    if (!uart_send_vision_request(type, distance))
    {
        BlueSerial_Printf("ERR vision_request_failed\r\n");
        return;
    }

    g_vision_request_pending = 1U;
    g_vision_request_wait_loops = 0U;
    g_vision_request_type = type;
    BlueSerial_Printf("OK vision_request type=%s distance=%ugrid\r\n",
                      BlueSerial_VisionTypeName(type),
                      (unsigned int)BlueSerial_VisionDistanceGridCount(distance));
}

static void BlueSerial_ServiceVisionRequest(void)
{
    VisionRecognitionResult result = {0};

    if (!g_vision_request_pending)
    {
        return;
    }

    process_vision_data();
    if ((g_vision_request_type == VISION_RECOGNITION_IMG &&
         vision_take_img_result(&result)) ||
        (g_vision_request_type == VISION_RECOGNITION_NUM &&
         vision_take_num_result(&result)))
    {
        g_vision_request_pending = 0U;
        BlueSerial_PrintVisionResult(g_vision_request_type, &result);
        g_vision_request_type = VISION_RECOGNITION_NONE;
        return;
    }

    if (g_vision_request_wait_loops < BLUESERIAL_VISION_TIMEOUT_LOOPS)
    {
        g_vision_request_wait_loops++;
        return;
    }

    g_vision_request_pending = 0U;
    BlueSerial_Printf("ERR vision_timeout type=%s\r\n",
                      BlueSerial_VisionTypeName(g_vision_request_type));
    g_vision_request_type = VISION_RECOGNITION_NONE;
}

static uint8 BlueSerial_HasPointTarget(void)
{
    uint32 primask;
    uint8 valid;

    primask = interrupt_global_disable();
    valid = g_point_target_valid;
    interrupt_global_enable(primask);
    return valid;
}

static uint8 BlueSerial_SetPointTarget(float x_m, float y_m)
{
    uint32 primask;

    if (!isfinite(x_m) || !isfinite(y_m))
    {
        BlueSerial_Printf("ERR invalid_target\r\n");
        return 0U;
    }
    if (x_m < 0.0f || y_m < 0.0f ||
        x_m > BLUESERIAL_POINT_TARGET_MAX_M ||
        y_m > BLUESERIAL_POINT_TARGET_MAX_M)
    {
        BlueSerial_Printf("ERR target_range x=0..%.1fm y=0..%.1fm\r\n",
                          BLUESERIAL_POINT_TARGET_MAX_M,
                          BLUESERIAL_POINT_TARGET_MAX_M);
        return 0U;
    }

    primask = interrupt_global_disable();
    g_point_target_x_m = x_m;
    g_point_target_y_m = y_m;
    g_point_target_valid = 1U;
    interrupt_global_enable(primask);

    BlueSerial_Printf("OK target_m=%.3f,%.3f send_start_to_go\r\n", x_m, y_m);
    return 1U;
}

static uint8 BlueSerial_BuildPointTargetPath(float start_x_m,
                                             float start_y_m,
                                             float target_x_m,
                                             float target_y_m)
{
    const float eps_m = 0.001f;
    float grid_m = BLUESERIAL_POINT_TARGET_GRID_M;
    float max_coord_m;
    float delta_x_m = target_x_m - start_x_m;
    float delta_y_m = target_y_m - start_y_m;
    int start_row;
    int start_col;
    int target_row;
    int target_col;

    if (start_x_m < 0.0f || start_y_m < 0.0f ||
        target_x_m < 0.0f || target_y_m < 0.0f)
    {
        return 0U;
    }

    max_coord_m = fmaxf(fmaxf(start_x_m, start_y_m),
                        fmaxf(target_x_m, target_y_m));
    if (max_coord_m > ((float)BLUESERIAL_POINT_TARGET_MAX_CELL * grid_m))
    {
        grid_m = max_coord_m / (float)BLUESERIAL_POINT_TARGET_MAX_CELL;
    }

    start_row = BlueSerial_ClampInt(BlueSerial_RoundToInt(start_x_m / grid_m), 0, 255);
    start_col = BlueSerial_ClampInt(BlueSerial_RoundToInt(start_y_m / grid_m), 0, 255);
    target_row = BlueSerial_ClampInt(BlueSerial_RoundToInt(target_x_m / grid_m), 0, 255);
    target_col = BlueSerial_ClampInt(BlueSerial_RoundToInt(target_y_m / grid_m), 0, 255);

    if (fabsf(delta_x_m) <= eps_m && fabsf(delta_y_m) <= eps_m)
    {
        path_follow_set_path(NULL, 0U);
        return 1U;
    }

    g_blueserial_point_target_path[0].row = (uint8)start_row;
    g_blueserial_point_target_path[0].col = (uint8)start_col;
    g_blueserial_point_target_path[0].id = 0U;
    /* A point relocation is one world-frame straight segment.  Splitting it
     * into X/Y legs introduces an artificial stop and restart, and a small
     * coordinate/yaw mismatch then appears as a visible tail segment. */
    g_blueserial_point_target_path[1].row = (uint8)target_row;
    g_blueserial_point_target_path[1].col = (uint8)target_col;
    g_blueserial_point_target_path[1].id = 0U;

    path_follow_set_path_with_grid(g_blueserial_point_target_path, 2U, grid_m, 0U);
    return 1U;
}

static uint8 BlueSerial_StartPointTargetMove(void)
{
    uint32 primask;
    path_follow_status_t status = {0};
    float target_x_m;
    float target_y_m;
    uint8 started;

    primask = interrupt_global_disable();
    if (!g_point_target_valid)
    {
        interrupt_global_enable(primask);
        return 0U;
    }

    target_x_m = g_point_target_x_m;
    target_y_m = g_point_target_y_m;
    path_follow_get_status(&status);
    path_follow_set_speed_test_mode(0U, 0U);
    started = BlueSerial_BuildPointTargetPath(status.x_m,
                                             status.y_m,
                                             target_x_m,
                                             target_y_m);
    if (started)
    {
        g_point_target_valid = 0U;
        g_turn_quarters_remaining = 0U;
        BlueSerial_ApplyConfiguredPathSpeedLocked();
        BlueSerial_EnterPathMode();
        path_follow_set_target_yaw(status.target_yaw_deg);
    }
    interrupt_global_enable(primask);

    if (started)
    {
        BlueSerial_Printf("OK start target_m=%.3f,%.3f from=%.3f,%.3f\r\n",
                          target_x_m,
                          target_y_m,
                          status.x_m,
                          status.y_m);
    }
    return started;
}

static void BlueSerial_StartCenteredRelativeMove(float delta_x_m,
                                                 float delta_y_m,
                                                 float pose_yaw_deg,
                                                 float target_yaw_deg,
                                                 uint8 segment_axis)
{
    const float eps_m = 0.001f;
    float grid_m = BLUESERIAL_RELATIVE_GRID_M;
    float max_delta_m = fmaxf(fabsf(delta_x_m), fabsf(delta_y_m));
    int center_cell = BLUESERIAL_RELATIVE_CENTER_CELL;
    int target_row;
    int target_col;

    if (max_delta_m <= eps_m)
    {
        path_follow_set_path(NULL, 0U);
        return;
    }

    /*
     * Position.row/col 是 uint8，不能保存负格点。蓝牙相对移动使用虚拟中心格点
     * 作为临时起点，避免 backward/right 从 0 附近出发时把 -1 写成 255。
     */
    if (max_delta_m > ((float)BLUESERIAL_RELATIVE_MAX_OFFSET_CELL * grid_m))
    {
        grid_m = max_delta_m / (float)BLUESERIAL_RELATIVE_MAX_OFFSET_CELL;
    }

    target_row = center_cell + BlueSerial_RoundToInt(delta_x_m / grid_m);
    target_col = center_cell + BlueSerial_RoundToInt(delta_y_m / grid_m);
    target_row = BlueSerial_ClampInt(target_row, 0, 255);
    target_col = BlueSerial_ClampInt(target_col, 0, 255);

    g_blueserial_relative_path[0].row = (uint8)center_cell;
    g_blueserial_relative_path[0].col = (uint8)center_cell;
    g_blueserial_relative_path[0].id = 0U;

    /* Keep Bluetooth relative motion as one world-frame straight segment.
     * Splitting the yaw-induced sub-centimetre lateral component into a
     * second axis leg caused a visible stop, restart and TPOS jump after the
     * main leg had already crossed its target plane. */
    g_blueserial_relative_path[1].row = (uint8)target_row;
    g_blueserial_relative_path[1].col = (uint8)target_col;
    g_blueserial_relative_path[1].id = 0U;

    path_follow_reset_pose((float)center_cell * grid_m,
                           (float)center_cell * grid_m,
                           pose_yaw_deg);
    path_follow_set_target_yaw(target_yaw_deg);
    path_follow_set_path_with_grid_axis(g_blueserial_relative_path,
                                        2U,
                                        grid_m,
                                        0U,
                                        segment_axis);
}

static uint8 BlueSerial_StartConfiguredMove(void)
{
    uint32 primask;
    blueserial_move_direction_t direction;
    float yaw_rad;
    float position_m;
    float delta_x_m = 0.0f;
    float delta_y_m = 0.0f;
    uint8 segment_axis = PATH_FOLLOW_AXIS_X;
    path_follow_status_t status = {0};

    if (g_target_speed_cmps <= 0.0f || g_target_position_cm <= 0.0f)
    {
        return 0U;
    }

    primask = interrupt_global_disable();
    path_follow_get_status(&status);
    direction = g_next_move_direction;
    yaw_rad = eulerAngle.yaw * ((float)M_PI / 180.0f);
    position_m = g_target_position_cm * 0.01f;

    /*
     * 手机端的前后左右按“当前车头朝向”解释：
     * forward/backward 沿车头方向，left/right 沿车体横向方向。
     */
    switch (direction)
    {
        case BLUESERIAL_MOVE_FORWARD:
            delta_x_m = cosf(yaw_rad) * position_m;
            delta_y_m = sinf(yaw_rad) * position_m;
            segment_axis = PATH_FOLLOW_AXIS_X;
            break;

        case BLUESERIAL_MOVE_BACKWARD:
            delta_x_m = -cosf(yaw_rad) * position_m;
            delta_y_m = -sinf(yaw_rad) * position_m;
            segment_axis = PATH_FOLLOW_AXIS_X;
            break;

        case BLUESERIAL_MOVE_LEFT:
            delta_x_m = -sinf(yaw_rad) * position_m;
            delta_y_m = cosf(yaw_rad) * position_m;
            segment_axis = PATH_FOLLOW_AXIS_Y;
            break;

        case BLUESERIAL_MOVE_RIGHT:
            delta_x_m = sinf(yaw_rad) * position_m;
            delta_y_m = -cosf(yaw_rad) * position_m;
            segment_axis = PATH_FOLLOW_AXIS_Y;
            break;

        default:
            interrupt_global_enable(primask);
            return 0U;
    }

    g_turn_quarters_remaining = 0U;
    BlueSerial_ApplyConfiguredPathSpeedLocked();
    path_follow_set_speed_test_mode(g_speed_test_profile_enabled,
                                    g_speed_test_yaw_enabled);
    BlueSerial_SpeedLogBeginLocked(direction,
                                   g_target_position_cm,
                                   eulerAngle.yaw);
    BlueSerial_EnterPathMode();
    BlueSerial_StartCenteredRelativeMove(delta_x_m,
                                         delta_y_m,
                                         eulerAngle.yaw,
                                         status.target_yaw_deg,
                                         segment_axis);
    interrupt_global_enable(primask);
    return 1U;
}

static void BlueSerial_StartNextQuarterTurn(void)
{
    path_follow_status_t status = {0};

    path_follow_get_status(&status);
    path_follow_start_rotate_to_yaw(status.target_yaw_deg + g_turn_step_deg);
}

static uint8 BlueSerial_StartTurn(uint8 quarter_count, float step_deg)
{
    uint32 primask;

    if (quarter_count == 0U)
    {
        return 0U;
    }

    primask = interrupt_global_disable();
    g_turn_quarters_remaining = quarter_count;
    g_turn_step_deg = step_deg;
    path_follow_set_speed_test_mode(0U, 0U);
    BlueSerial_EnterPathMode();
    BlueSerial_StartNextQuarterTurn();
    interrupt_global_enable(primask);
    return 1U;
}

static void BlueSerial_ServiceMotionCompletion(void)
{
    uint32 primask;
    uint8 completed = 0U;
    uint8 speed_test_timeout = 0U;
    uint8 speed_test_fault = 0U;
    path_follow_status_t status = {0};

    primask = interrupt_global_disable();
    if (g_control_mode != BLUESERIAL_MODE_PATH_MOTION)
    {
        interrupt_global_enable(primask);
        return;
    }

    path_follow_get_status(&status);
    if (status.active)
    {
        interrupt_global_enable(primask);
        return;
    }

    if (g_turn_quarters_remaining > 0U)
    {
        g_turn_quarters_remaining--;
        if (g_turn_quarters_remaining > 0U)
        {
            BlueSerial_StartNextQuarterTurn();
            interrupt_global_enable(primask);
            return;
        }
    }

    /*
     * 一次路径动作完成后释放蓝牙路径接管；如果之前打开过 yawhold，
     * 则继续保持航向，否则停止电机输出。
     */
    BlueSerial_SpeedLogFinishLocked(0U);
    speed_test_timeout = status.speed_test_settle_timeout;
    speed_test_fault = status.speed_test_profile_fault;
    path_follow_set_speed_test_mode(0U, 0U);
    g_control_mode = BLUESERIAL_MODE_STOP;
    car_go_flag = g_yaw_hold_enabled ? 1U : 0U;
    car_stop_flag = 0U;
    BlueSerial_ZeroSpeedCommand();
    BlueSerial_ClearWheelPid();
    motor_pwm(0, 0, 0, 0);
    completed = 1U;
    interrupt_global_enable(primask);

    if (completed)
    {
        if (speed_test_fault)
        {
            BlueSerial_Printf("FAULT speed_profile\r\n");
        }
        else if (speed_test_timeout)
        {
            BlueSerial_Printf("DONE settle_timeout\r\n");
        }
        else
        {
            BlueSerial_Printf("DONE\r\n");
        }
    }
}

static void BlueSerial_EnterRawPwmMode(void)
{
    BlueSerial_SpeedLogFinishLocked(1U);
    g_turn_quarters_remaining = 0U;
    g_yaw_hold_enabled = 0U;
    path_follow_set_path(NULL, 0U);
    path_follow_set_speed_test_mode(0U, 0U);
    path_follow_set_stationary_yaw_hold_enabled(0U);
    g_control_mode = BLUESERIAL_MODE_RAW_PWM;
    car_go_flag = 0U;
    car_stop_flag = 0U;
    BlueSerial_ZeroSpeedCommand();
    BlueSerial_ClearWheelPid();
    BlueSerial_ApplyRawPwm();
}

void Blue_Serial_Init(void)
{
#if !MOTOR_BOARD_USE_NEW && !MOTOR_OLD_BOARD_UART8_USE_BLUETOOTH
    /* Old-board competition firmware reserves UART8/D16/D17 for recognition. */
    g_enabled = 0U;
    g_initialized = 0U;
    return;
#endif
#if !MOTOR_BOARD_USE_NEW && MOTOR_OLD_BOARD_UART8_USE_BLUETOOTH
    /* The dedicated old-board tuning build always enables its UART owner. */
    g_enabled = 1U;
#endif
    g_rx_collecting = 0U;
    g_rx_work_len = 0U;
    g_rx_queue_write = 0U;
    g_rx_queue_read = 0U;
    g_rx_drop_count = 0U;
    g_last_rx_frame[0] = '\0';
    g_last_rx_frame_len = 0U;
    g_telemetry_tick_count = 0U;
    g_telemetry_report_pending = 0U;
    g_telemetry_sequence = 0U;
    g_telemetry_was_active = 0U;
    g_speed_log_enabled = 0U;
    g_speed_test_profile_enabled = 0U;
    g_speed_test_yaw_enabled = 0U;
    g_speed_run_id = 0U;
    g_speed_cfg_id = 0U;
    BlueSerial_SpeedLogResetLocked();
    uart_init(BLUESERIAL_UART, BLUESERIAL_BAUDRATE, BLUESERIAL_TX_PIN, BLUESERIAL_RX_PIN);
    interrupt_set_priority(BLUESERIAL_IRQN, BLUESERIAL_IRQ_PRIORITY);
    uart_rx_interrupt(BLUESERIAL_UART, g_enabled ? ZF_ENABLE : ZF_DISABLE);
    g_initialized = 1U;
}

void BlueSerial_SetEnabled(uint8 enabled)
{
    uint32 primask;
    uint8 new_enabled = enabled ? 1U : 0U;

#if !MOTOR_BOARD_USE_NEW && !MOTOR_OLD_BOARD_UART8_USE_BLUETOOTH
    /* Do not let menu state steal the recognition camera's UART8 pins. */
    new_enabled = 0U;
#endif

    primask = interrupt_global_disable();
    if (g_initialized && g_enabled && !new_enabled)
    {
        BlueSerial_SpeedLogFinishLocked(1U);
        BlueSerial_StopControlLocked();
    }
    g_enabled = new_enabled;
    g_rx_collecting = 0U;
    g_rx_work_len = 0U;
    g_rx_queue_read = g_rx_queue_write;
    if (g_initialized)
    {
        uart_rx_interrupt(BLUESERIAL_UART, g_enabled ? ZF_ENABLE : ZF_DISABLE);
    }
    interrupt_global_enable(primask);
}

uint8 BlueSerial_GetEnabled(void)
{
    return g_enabled ? 1U : 0U;
}

void BlueSerial_SendByte(uint8 Byte)
{
    uart_write_byte(BLUESERIAL_UART, Byte);
}

void BlueSerial_SendArray(uint8 *Array, uint16 Length)
{
    uint16 i;

    for (i = 0U; i < Length; ++i)
    {
        BlueSerial_SendByte(Array[i]);
    }
}

void BlueSerial_SendString(char *String)
{
    uint16 i;

    for (i = 0U; String[i] != '\0'; ++i)
    {
        BlueSerial_SendByte((uint8)String[i]);
    }
}

uint32 BlueSerial_Pow(uint32 X, uint32 Y)
{
    uint32 Result = 1U;

    while (Y--)
    {
        Result *= X;
    }
    return Result;
}

void BlueSerial_SendNumber(uint32 Number, uint8 Length)
{
    uint8 i;

    for (i = 0U; i < Length; ++i)
    {
        BlueSerial_SendByte((uint8)(Number / BlueSerial_Pow(10U, Length - i - 1U) % 10U + '0'));
    }
}

void BlueSerial_Printf(char *format, ...)
{
    char String[220];
    va_list arg;

    va_start(arg, format);
    (void)vsnprintf(String, sizeof(String), format, arg);
    va_end(arg);
    String[sizeof(String) - 1U] = '\0';
    BlueSerial_SendString(String);
}

static int BlueSerial_DirectionWheelSign(blueserial_move_direction_t direction,
                                         uint8 wheel_index)
{
    static const int8 signs[4][MOTOR_WHEEL_COUNT] =
    {
        { 1,  1,  1,  1},
        {-1, -1, -1, -1},
        {-1,  1,  1, -1},
        { 1, -1, -1,  1}
    };

    if ((uint8)direction >= 4U || wheel_index >= MOTOR_WHEEL_COUNT)
    {
        return 0;
    }
    return signs[(uint8)direction][wheel_index];
}

static void BlueSerial_CountsToBodyCm(const int32 counts[MOTOR_WHEEL_COUNT],
                                      float *x_cm,
                                      float *y_cm)
{
    float x_counts;
    float y_counts;
    float count_to_cm;

    if (x_cm == NULL || y_cm == NULL || pulse_per_meter <= 0.0)
    {
        if (x_cm != NULL)
        {
            *x_cm = 0.0f;
        }
        if (y_cm != NULL)
        {
            *y_cm = 0.0f;
        }
        return;
    }
    x_counts = 0.25f * (float)(counts[MOTOR_WHEEL_UL] +
                                      counts[MOTOR_WHEEL_UR] +
                                      counts[MOTOR_WHEEL_DL] +
                                      counts[MOTOR_WHEEL_DR]);
    y_counts = 0.25f * (float)(-counts[MOTOR_WHEEL_UL] +
                                      counts[MOTOR_WHEEL_UR] +
                                      counts[MOTOR_WHEEL_DL] -
                                      counts[MOTOR_WHEEL_DR]);
    count_to_cm = 100.0f / (float)pulse_per_meter;
    *y_cm = y_counts * count_to_cm * LATERAL_CORRECTION_FACTOR;
    *x_cm = x_counts * count_to_cm +
            LATERAL_TO_LONGITUDINAL_COUPLING_FACTOR * (*y_cm);
}

static void BlueSerial_ProjectBodyCm(blueserial_move_direction_t direction,
                                     float x_cm,
                                     float y_cm,
                                     float *axis_cm,
                                     float *cross_cm)
{
    if (axis_cm == NULL || cross_cm == NULL)
    {
        return;
    }
    switch (direction)
    {
        case BLUESERIAL_MOVE_FORWARD:
            *axis_cm = x_cm;
            *cross_cm = y_cm;
            break;
        case BLUESERIAL_MOVE_BACKWARD:
            *axis_cm = -x_cm;
            *cross_cm = y_cm;
            break;
        case BLUESERIAL_MOVE_LEFT:
            *axis_cm = y_cm;
            *cross_cm = x_cm;
            break;
        case BLUESERIAL_MOVE_RIGHT:
            *axis_cm = -y_cm;
            *cross_cm = x_cm;
            break;
        default:
            *axis_cm = 0.0f;
            *cross_cm = 0.0f;
            break;
    }
}

static uint16 BlueSerial_SpeedLogSlotFromDumpIndex(uint16 dump_index)
{
    if (!g_speed_log_overflow || dump_index < BLUESERIAL_SPEED_LOG_HEAD_COUNT)
    {
        return dump_index;
    }
    return (uint16)(BLUESERIAL_SPEED_LOG_HEAD_COUNT +
                    ((g_speed_log_tail_write + dump_index -
                      BLUESERIAL_SPEED_LOG_HEAD_COUNT) %
                     BLUESERIAL_SPEED_LOG_TAIL_COUNT));
}

static void BlueSerial_PrintSpeedTuneStatus(void)
{
    uint32 primask;
    path_follow_status_t status = {0};
    float kp[MOTOR_WHEEL_COUNT];
    float ki[MOTOR_WHEEL_COUNT];
    float kd[MOTOR_WHEEL_COUNT];
    float alpha[MOTOR_WHEEL_COUNT];
    int deadzone_fwd[MOTOR_WHEEL_COUNT];
    int deadzone_rev[MOTOR_WHEEL_COUNT];
    int deadzone_min;
    uint8 i;

    primask = interrupt_global_disable();
    path_follow_get_status(&status);
    for (i = 0U; i < MOTOR_WHEEL_COUNT; ++i)
    {
        tagPID_T *pid = BlueSerial_GetWheelPid(i);
        kp[i] = pid->fKp;
        ki[i] = pid->fKi;
        kd[i] = pid->fKd;
        alpha[i] = pid->alpha;
        deadzone_fwd[i] = motor_deadzone_fwd[i];
        deadzone_rev[i] = motor_deadzone_rev[i];
    }
    deadzone_min = motor_deadzone_target_min_counts;
    interrupt_global_enable(primask);

    BlueSerial_Printf("SPDCTL run=%lu cfg=%lu mode=%u arm=%u test=%u yaw=%u "
                      "pos=%u line=%u log=%u rec=%u ready=%u n=%u total=%lu ovf=%u\r\n",
                      (unsigned long)g_speed_run_id,
                      (unsigned long)g_speed_cfg_id,
                      (unsigned int)g_control_mode,
                      (unsigned int)g_speed_test_profile_enabled,
                      (unsigned int)status.speed_test_enabled,
                      (unsigned int)g_speed_test_yaw_enabled,
                      (unsigned int)status.position_loop_active,
                      (unsigned int)status.line_guidance_active,
                      (unsigned int)g_speed_log_enabled,
                      (unsigned int)g_speed_log_recording,
                      (unsigned int)g_speed_log_ready,
                      (unsigned int)g_speed_log_count,
                      (unsigned long)g_speed_log_total_ticks,
                      (unsigned int)g_speed_log_overflow);
    BlueSerial_Printf("SPDPID UL=%.3f,%.3f,%.3f,%.2f UR=%.3f,%.3f,%.3f,%.2f\r\n",
                      kp[0], ki[0], kd[0], alpha[0],
                      kp[1], ki[1], kd[1], alpha[1]);
    BlueSerial_Printf("SPDPID DL=%.3f,%.3f,%.3f,%.2f DR=%.3f,%.3f,%.3f,%.2f\r\n",
                      kp[2], ki[2], kd[2], alpha[2],
                      kp[3], ki[3], kd[3], alpha[3]);
    BlueSerial_Printf("SPDDZ UL=%d,%d UR=%d,%d min=%d\r\n",
                      deadzone_fwd[0], deadzone_rev[0],
                      deadzone_fwd[1], deadzone_rev[1], deadzone_min);
    BlueSerial_Printf("SPDDZ DL=%d,%d DR=%d,%d min=%d\r\n",
                      deadzone_fwd[2], deadzone_rev[2],
                      deadzone_fwd[3], deadzone_rev[3], deadzone_min);
}

static char BlueSerial_DirectionLetter(blueserial_move_direction_t direction)
{
    static const char letters[4] = {'F', 'B', 'L', 'R'};

    return ((uint8)direction < 4U) ? letters[(uint8)direction] : '?';
}

static void BlueSerial_PrintSpeedSummary(void)
{
    int32 target_counts[MOTOR_WHEEL_COUNT];
    int32 actual_counts[MOTOR_WHEEL_COUNT];
    int32 error_counts[MOTOR_WHEEL_COUNT];
    int32 t50_target_counts[MOTOR_WHEEL_COUNT] = {0};
    int32 t50_actual_counts[MOTOR_WHEEL_COUNT] = {0};
    int32 start_200ms_counts[MOTOR_WHEEL_COUNT] = {0};
    float gain[MOTOR_WHEEL_COUNT];
    float nmae[MOTOR_WHEEL_COUNT];
    float target_x_cm;
    float target_y_cm;
    float actual_x_cm;
    float actual_y_cm;
    float target_axis_cm;
    float target_cross_cm;
    float actual_axis_cm;
    float actual_cross_cm;
    float cross_error_cm;
    float cross_ratio_pct;
    float yaw_error_deg;
    float encoder_yaw_deg = 0.0f;
    float max_gain = -1000000.0f;
    float min_gain = 1000000.0f;
    float wheel_spread_pct;
    float t50_target_x_cm;
    float t50_target_y_cm;
    float t50_actual_x_cm;
    float t50_actual_y_cm;
    float t50_target_axis_cm;
    float t50_target_cross_cm;
    float t50_actual_axis_cm;
    float t50_actual_cross_cm;
    float t50_cross_error_cm = 0.0f;
    float t50_yaw_deg = 0.0f;
    float peak_target_average = 0.0f;
    float overshoot_cm;
    float compensated_back_cm;
    int peak_target[MOTOR_WHEEL_COUNT] = {0};
    int target_start_tick[MOTOR_WHEEL_COUNT] = {-1, -1, -1, -1};
    int actual_start_tick[MOTOR_WHEEL_COUNT] = {-1, -1, -1, -1};
    int lag_ms[MOTOR_WHEEL_COUNT] = {-1, -1, -1, -1};
    int first_target_tick = -1;
    int t50_tick = -1;
    int lag_spread_ms = 0;
    int min_lag_ms = 0;
    int max_lag_ms = 0;
    uint32 settle_ms = 0U;
    uint16 sample_count;
    uint16 startup_sample_count;
    uint16 j;
    uint8 i;
    uint8 lags_valid = 1U;

    if (!g_speed_log_ready || g_speed_log_recording)
    {
        BlueSerial_Printf("ERR speed_log_not_ready rec=%u\r\n",
                          (unsigned int)g_speed_log_recording);
        return;
    }

    sample_count = g_speed_log_count;
    startup_sample_count = (sample_count < BLUESERIAL_SPEED_LOG_HEAD_COUNT) ?
                           sample_count : BLUESERIAL_SPEED_LOG_HEAD_COUNT;
    for (i = 0U; i < MOTOR_WHEEL_COUNT; ++i)
    {
        int direction_sign = BlueSerial_DirectionWheelSign(g_speed_log_direction, i);
        float signed_target;

        target_counts[i] = g_speed_log_final_motor.cumulative_target_counts[i];
        actual_counts[i] = g_speed_log_final_motor.cumulative_raw_counts[i];
        error_counts[i] = target_counts[i] - actual_counts[i];
        signed_target = (float)(direction_sign * target_counts[i]);
        gain[i] = (fabsf(signed_target) > 0.5f) ?
                  (100.0f * (float)(direction_sign * actual_counts[i]) /
                   signed_target) : 0.0f;
        nmae[i] = (g_speed_log_abs_target_sum[i] > 0U) ?
                  (100.0f * (float)g_speed_log_abs_error_sum[i] /
                   (float)g_speed_log_abs_target_sum[i]) : 0.0f;
        max_gain = fmaxf(max_gain, gain[i]);
        min_gain = fminf(min_gain, gain[i]);
    }
    wheel_spread_pct = max_gain - min_gain;

    for (j = 0U; j < startup_sample_count; ++j)
    {
        const blueserial_speed_sample_t *sample =
            &g_speed_log[BlueSerial_SpeedLogSlotFromDumpIndex(j)];
        float average_target = 0.0f;

        for (i = 0U; i < MOTOR_WHEEL_COUNT; ++i)
        {
            int direction_sign = BlueSerial_DirectionWheelSign(g_speed_log_direction, i);
            int signed_target = direction_sign * sample->target[i];

            if (signed_target > peak_target[i])
            {
                peak_target[i] = signed_target;
            }
            average_target += 0.25f * (float)signed_target;
        }
        peak_target_average = fmaxf(peak_target_average, average_target);
    }

    for (j = 0U; j < startup_sample_count; ++j)
    {
        const blueserial_speed_sample_t *sample =
            &g_speed_log[BlueSerial_SpeedLogSlotFromDumpIndex(j)];

        for (i = 0U; i < MOTOR_WHEEL_COUNT; ++i)
        {
            int direction_sign = BlueSerial_DirectionWheelSign(g_speed_log_direction, i);
            int threshold = (peak_target[i] + 9) / 10;
            int signed_target = direction_sign * sample->target[i];
            int signed_actual = direction_sign * sample->actual[i];

            if (threshold < 1)
            {
                threshold = 1;
            }
            if (target_start_tick[i] < 0 && signed_target >= threshold)
            {
                target_start_tick[i] = (int)sample->tick;
                if (first_target_tick < 0 || target_start_tick[i] < first_target_tick)
                {
                    first_target_tick = target_start_tick[i];
                }
            }
            if (target_start_tick[i] >= 0 &&
                actual_start_tick[i] < 0 && j + 1U < startup_sample_count &&
                (int)sample->tick >= target_start_tick[i])
            {
                const blueserial_speed_sample_t *next_sample =
                    &g_speed_log[BlueSerial_SpeedLogSlotFromDumpIndex((uint16)(j + 1U))];
                int next_actual = direction_sign * next_sample->actual[i];

                if (next_sample->tick == (uint16)(sample->tick + 1U) &&
                    signed_actual >= threshold && next_actual >= threshold)
                {
                    actual_start_tick[i] = (int)sample->tick;
                }
            }
        }
    }

    if (first_target_tick >= 0)
    {
        for (j = 0U; j < startup_sample_count; ++j)
        {
            const blueserial_speed_sample_t *sample =
                &g_speed_log[BlueSerial_SpeedLogSlotFromDumpIndex(j)];

            if ((int)sample->tick >= first_target_tick &&
                (int)sample->tick < first_target_tick + 20)
            {
                for (i = 0U; i < MOTOR_WHEEL_COUNT; ++i)
                {
                    start_200ms_counts[i] +=
                        BlueSerial_DirectionWheelSign(g_speed_log_direction, i) *
                        sample->raw[i];
                }
            }
        }
    }

    for (i = 0U; i < MOTOR_WHEEL_COUNT; ++i)
    {
        if (target_start_tick[i] >= 0 && actual_start_tick[i] >= 0)
        {
            lag_ms[i] = (actual_start_tick[i] - target_start_tick[i]) * 10;
        }
        else
        {
            lags_valid = 0U;
        }
        if (i == 0U || lag_ms[i] < min_lag_ms)
        {
            min_lag_ms = lag_ms[i];
        }
        if (i == 0U || lag_ms[i] > max_lag_ms)
        {
            max_lag_ms = lag_ms[i];
        }
    }
    lag_spread_ms = lags_valid ? (max_lag_ms - min_lag_ms) : -1;

    if (peak_target_average > 0.0f)
    {
        for (j = 0U; j < startup_sample_count; ++j)
        {
            const blueserial_speed_sample_t *sample =
                &g_speed_log[BlueSerial_SpeedLogSlotFromDumpIndex(j)];
            float average_target = 0.0f;

            for (i = 0U; i < MOTOR_WHEEL_COUNT; ++i)
            {
                int direction_sign = BlueSerial_DirectionWheelSign(g_speed_log_direction, i);
                average_target += 0.25f * (float)(direction_sign * sample->target[i]);
                t50_target_counts[i] += sample->target[i];
                t50_actual_counts[i] += sample->raw[i];
            }
            if (average_target >= 0.5f * peak_target_average)
            {
                t50_tick = (int)sample->tick;
                t50_yaw_deg = (float)sample->yaw_delta_cdeg * 0.01f;
                break;
            }
        }
    }

    BlueSerial_CountsToBodyCm(target_counts, &target_x_cm, &target_y_cm);
    BlueSerial_CountsToBodyCm(actual_counts, &actual_x_cm, &actual_y_cm);
    BlueSerial_ProjectBodyCm(g_speed_log_direction, target_x_cm, target_y_cm,
                             &target_axis_cm, &target_cross_cm);
    BlueSerial_ProjectBodyCm(g_speed_log_direction, actual_x_cm, actual_y_cm,
                             &actual_axis_cm, &actual_cross_cm);
    cross_error_cm = actual_cross_cm - target_cross_cm;
    cross_ratio_pct = 100.0f * cross_error_cm /
                      fmaxf(fabsf(actual_axis_cm), 1.0f);
    yaw_error_deg = BlueSerial_WrapDeg(g_speed_log_end_yaw_deg -
                                       g_speed_log_start_yaw_deg);
    if (pulse_per_meter > 0.0 && rx_plus_ry_cali > 0.0f)
    {
        float rotation_counts = 0.25f *
            (float)(-actual_counts[0] + actual_counts[1] -
                    actual_counts[2] + actual_counts[3]);
        encoder_yaw_deg = rotation_counts /
                          ((float)pulse_per_meter * rx_plus_ry_cali) *
                          (180.0f / (float)M_PI);
    }
    if (t50_tick >= 0)
    {
        BlueSerial_CountsToBodyCm(t50_target_counts,
                                  &t50_target_x_cm, &t50_target_y_cm);
        BlueSerial_CountsToBodyCm(t50_actual_counts,
                                  &t50_actual_x_cm, &t50_actual_y_cm);
        BlueSerial_ProjectBodyCm(g_speed_log_direction,
                                 t50_target_x_cm, t50_target_y_cm,
                                 &t50_target_axis_cm, &t50_target_cross_cm);
        BlueSerial_ProjectBodyCm(g_speed_log_direction,
                                 t50_actual_x_cm, t50_actual_y_cm,
                                 &t50_actual_axis_cm, &t50_actual_cross_cm);
        t50_cross_error_cm = t50_actual_cross_cm - t50_target_cross_cm;
    }
    overshoot_cm = fmaxf(0.0f, g_speed_log_peak_axis_cm - target_axis_cm);
    compensated_back_cm = fmaxf(0.0f, g_speed_log_peak_axis_cm - actual_axis_cm);
    if (g_speed_log_profile_total_s > 0.0f)
    {
        int settle_ms_signed = (int)(g_speed_log_total_ticks * 10U) -
                               BlueSerial_RoundToInt(g_speed_log_profile_total_s * 1000.0f);
        settle_ms = (settle_ms_signed > 0) ? (uint32)settle_ms_signed : 0U;
    }

    BlueSerial_Printf("SPDSUM ID=%lu CFG=%lu D=%c V=%.1f REQ=%.2f T=%.2f A=%.2f SH=%.2f "
                      "RG=%.2f X=%.2f XR=%.2f YE=%.2f YP=%.2f YENC=%.2f "
                      "AB=%u TO=%u PF=%u OVF=%u\r\n",
                       (unsigned long)g_speed_log_run_id,
                      (unsigned long)g_speed_log_cfg_id,
                      BlueSerial_DirectionLetter(g_speed_log_direction),
                      g_speed_log_requested_speed_cmps,
                      g_speed_log_requested_cm, target_axis_cm, actual_axis_cm,
                      target_axis_cm - actual_axis_cm,
                      g_speed_log_requested_cm - actual_axis_cm,
                      cross_error_cm, cross_ratio_pct, yaw_error_deg,
                      g_speed_log_peak_yaw_abs_deg, encoder_yaw_deg,
                      (unsigned int)g_speed_log_aborted,
                      (unsigned int)g_speed_log_settle_timeout,
                      (unsigned int)g_speed_log_profile_fault,
                      (unsigned int)g_speed_log_overflow);
    BlueSerial_Printf("SPDWHL ID=%lu G=%.1f,%.1f,%.1f,%.1f GSP=%.1f "
                      "NM=%.1f,%.1f,%.1f,%.1f PE=%d,%d,%d,%d SATN=%lu,%lu,%lu,%lu\r\n",
                       (unsigned long)g_speed_log_run_id,
                      gain[0], gain[1], gain[2], gain[3], wheel_spread_pct,
                      nmae[0], nmae[1], nmae[2], nmae[3],
                      g_speed_log_peak_error[0], g_speed_log_peak_error[1],
                      g_speed_log_peak_error[2], g_speed_log_peak_error[3],
                      (unsigned long)g_speed_log_saturation_ticks[0],
                      (unsigned long)g_speed_log_saturation_ticks[1],
                      (unsigned long)g_speed_log_saturation_ticks[2],
                      (unsigned long)g_speed_log_saturation_ticks[3]);
    BlueSerial_Printf("SPDSTART ID=%lu L10=%d,%d,%d,%d DSP=%d X0=%.2f Y0=%.2f T50=%d "
                      "D200=%ld,%ld,%ld,%ld\r\n",
                       (unsigned long)g_speed_log_run_id,
                       lag_ms[0], lag_ms[1], lag_ms[2], lag_ms[3], lag_spread_ms,
                      t50_cross_error_cm, t50_yaw_deg,
                      (t50_tick >= 0) ? t50_tick * 10 : -1,
                      (long)start_200ms_counts[0], (long)start_200ms_counts[1],
                      (long)start_200ms_counts[2], (long)start_200ms_counts[3]);
    BlueSerial_Printf("SPDTAIL ID=%lu PK=%.2f OV=%.2f BK=%.2f SET=%lums TOTAL=%lums\r\n",
                       (unsigned long)g_speed_log_run_id,
                       g_speed_log_peak_axis_cm, overshoot_cm, compensated_back_cm,
                      (unsigned long)settle_ms,
                      (unsigned long)(g_speed_log_total_ticks * 10U));
    BlueSerial_Printf("SPDCUM ID=%lu T=%ld,%ld,%ld,%ld A=%ld,%ld,%ld,%ld "
                      "E=%ld,%ld,%ld,%ld\r\n",
                       (unsigned long)g_speed_log_run_id,
                       (long)target_counts[0], (long)target_counts[1],
                      (long)target_counts[2], (long)target_counts[3],
                      (long)actual_counts[0], (long)actual_counts[1],
                      (long)actual_counts[2], (long)actual_counts[3],
                      (long)error_counts[0], (long)error_counts[1],
                      (long)error_counts[2], (long)error_counts[3]);
}

static void BlueSerial_ServiceSpeedSummary(void)
{
    if (!g_speed_summary_pending)
    {
        return;
    }
    g_speed_summary_pending = 0U;
    BlueSerial_PrintSpeedSummary();
}

static void BlueSerial_ServiceSpeedDump(void)
{
    const blueserial_speed_sample_t *sample;
    uint16 slot;

    if (!g_speed_log_dumping || g_speed_log_recording)
    {
        return;
    }
    if (g_speed_log_dump_index >= g_speed_log_count)
    {
        g_speed_log_dumping = 0U;
        BlueSerial_Printf("SPDEND ID=%lu rows=%u total=%lu ovf=%u\r\n",
                           (unsigned long)g_speed_log_run_id,
                           (unsigned int)g_speed_log_count,
                          (unsigned long)g_speed_log_total_ticks,
                          (unsigned int)g_speed_log_overflow);
        return;
    }

    slot = BlueSerial_SpeedLogSlotFromDumpIndex(g_speed_log_dump_index);
    sample = &g_speed_log[slot];
    BlueSerial_Printf("SPD10 ID=%lu K=%u T=%d,%d,%d,%d F=%d,%d,%d,%d A=%d,%d,%d,%d "
                      "U=%d,%d,%d,%d P=%d,%d,%d,%d Y=%.2f G=%.2f\r\n",
                       (unsigned long)g_speed_log_run_id,
                       (unsigned int)sample->tick,
                      sample->target[0], sample->target[1],
                      sample->target[2], sample->target[3],
                      sample->actual[0], sample->actual[1],
                      sample->actual[2], sample->actual[3],
                      sample->raw[0], sample->raw[1],
                      sample->raw[2], sample->raw[3],
                      sample->pid_pwm[0], sample->pid_pwm[1],
                      sample->pid_pwm[2], sample->pid_pwm[3],
                      sample->final_pwm[0], sample->final_pwm[1],
                      sample->final_pwm[2], sample->final_pwm[3],
                      (float)sample->yaw_delta_cdeg * 0.01f,
                      (float)sample->gyro_z_cdegps * 0.01f);
    g_speed_log_dump_index++;
}

void BlueSerial_GetLastRxFrame(char *buffer, uint16 length)
{
    uint32 primask;

    if (buffer == NULL || length == 0U)
    {
        return;
    }

    primask = interrupt_global_disable();
    if (g_last_rx_frame_len == 0U)
    {
        (void)strncpy(buffer, "NONE", length - 1U);
    }
    else
    {
        (void)strncpy(buffer, g_last_rx_frame, length - 1U);
    }
    buffer[length - 1U] = '\0';
    interrupt_global_enable(primask);
}

static void BlueSerial_ProcessRxByte(uint8 ch)
{
    uint8 next_write;
    uint8 i;

    if (ch == '[')
    {
        g_rx_collecting = 1U;
        g_rx_work_len = 0U;
        return;
    }

    if (!g_rx_collecting)
    {
        return;
    }

    if (ch == ']')
    {
        if (g_rx_work_len > 0U)
        {
            for (i = 0U; i < g_rx_work_len; ++i)
            {
                g_last_rx_frame[i] = g_rx_work[i];
            }
            g_last_rx_frame[g_rx_work_len] = '\0';
            g_last_rx_frame_len = g_rx_work_len;

            next_write = (uint8)((g_rx_queue_write + 1U) % BLUESERIAL_RX_QUEUE_LEN);
            if (next_write != g_rx_queue_read)
            {
                for (i = 0U; i < g_rx_work_len; ++i)
                {
                    g_rx_queue[g_rx_queue_write][i] = g_rx_work[i];
                }
                g_rx_queue[g_rx_queue_write][g_rx_work_len] = '\0';
                g_rx_queue_write = next_write;
            }
            else
            {
                g_rx_drop_count++;
            }
        }
        g_rx_collecting = 0U;
        g_rx_work_len = 0U;
        return;
    }

    if (g_rx_work_len < (BLUESERIAL_RX_FRAME_LEN - 1U))
    {
        g_rx_work[g_rx_work_len++] = (char)ch;
    }
    else
    {
        g_rx_collecting = 0U;
        g_rx_work_len = 0U;
        g_rx_drop_count++;
    }
}

void BlueSerial_RxIrqHandler(void)
{
    uint8 ch = 0U;

    /*
     * LPUART 可能一次中断里已经积累多个字节，这里尽量读空 RX FIFO。
     * 注意：仍然只做轻量收帧状态机，不在 ISR 里 sscanf/printf/控制电机。
     */
    while (uart_query_byte(BLUESERIAL_UART, &ch))
    {
        if (g_enabled)
        {
            BlueSerial_ProcessRxByte(ch);
        }
    }
}

static char *BlueSerial_TrimCommand(char *command)
{
    char *end;

    while (*command == ' ' || *command == '\t' || *command == '\r' || *command == '\n')
    {
        command++;
    }

    end = command + strlen(command);
    while (end > command)
    {
        char ch = end[-1];
        if (ch != ' ' && ch != '\t' && ch != '\r' && ch != '\n')
        {
            break;
        }
        *--end = '\0';
    }
    return command;
}

static const char *BlueSerial_LogicalWheelName(uint8 wheel_index)
{
    static const char *const names[4] = {"ul", "ur", "dl", "dr"};

    return (wheel_index < 4U) ? names[wheel_index] : "unknown";
}

static const char *BlueSerial_PhysicalMotorName(uint8 logical_wheel_index)
{
#if MOTOR_BOARD_REMAP_LOGICAL_WHEELS
    static const char *const names[4] = {"m1", "m2", "m4", "m3"};
#else
    static const char *const names[4] = {"m1", "m2", "m3", "m4"};
#endif

    return (logical_wheel_index < 4U) ? names[logical_wheel_index] : "unknown";
}

static int BlueSerial_GetLogicalWheelIndex(const char *name)
{
    if (strcmp(name, "pwm.ul") == 0)
    {
        return 0;
    }
    if (strcmp(name, "pwm.ur") == 0)
    {
        return 1;
    }
    if (strcmp(name, "pwm.dl") == 0)
    {
        return 2;
    }
    if (strcmp(name, "pwm.dr") == 0)
    {
        return 3;
    }

#if MOTOR_BOARD_REMAP_LOGICAL_WHEELS
    if (strcmp(name, "pwm.m1") == 0)
    {
        return 0;
    }
    if (strcmp(name, "pwm.m2") == 0)
    {
        return 1;
    }
    if (strcmp(name, "pwm.m3") == 0)
    {
        return 3;
    }
    if (strcmp(name, "pwm.m4") == 0)
    {
        return 2;
    }
#else
    if (strcmp(name, "pwm.m1") == 0)
    {
        return 0;
    }
    if (strcmp(name, "pwm.m2") == 0)
    {
        return 1;
    }
    if (strcmp(name, "pwm.m3") == 0)
    {
        return 2;
    }
    if (strcmp(name, "pwm.m4") == 0)
    {
        return 3;
    }
#endif

    return -1;
}

static tagPID_T *BlueSerial_GetWheelPid(uint8 wheel_index)
{
    static tagPID_T *const pids[MOTOR_WHEEL_COUNT] =
    {
        &ULpid, &URpid, &DLpid, &DRpid
    };

    return (wheel_index < MOTOR_WHEEL_COUNT) ? pids[wheel_index] : NULL;
}

static PIDInitStruct *BlueSerial_GetWheelPidInit(uint8 wheel_index)
{
    static PIDInitStruct *const init[MOTOR_WHEEL_COUNT] =
    {
        &ULPidInitStruct, &URPidInitStruct, &DLPidInitStruct, &DRPidInitStruct
    };

    return (wheel_index < MOTOR_WHEEL_COUNT) ? init[wheel_index] : NULL;
}

static int BlueSerial_ParseWheelToken(const char *token)
{
    if (strcmp(token, "ul") == 0)
    {
        return MOTOR_WHEEL_UL;
    }
    if (strcmp(token, "ur") == 0)
    {
        return MOTOR_WHEEL_UR;
    }
    if (strcmp(token, "dl") == 0)
    {
        return MOTOR_WHEEL_DL;
    }
    if (strcmp(token, "dr") == 0)
    {
        return MOTOR_WHEEL_DR;
    }
    if (strcmp(token, "all") == 0)
    {
        return MOTOR_WHEEL_COUNT;
    }
    return -1;
}

static uint8 BlueSerial_TuningIsIdle(void)
{
    uint32 primask;
    uint8 idle;
    path_follow_status_t status = {0};

    primask = interrupt_global_disable();
    idle = (g_control_mode == BLUESERIAL_MODE_STOP && !g_yaw_hold_enabled) ? 1U : 0U;
    path_follow_get_status(&status);
    interrupt_global_enable(primask);
    return (idle && !status.active) ? 1U : 0U;
}

static uint8 BlueSerial_SplitTuneName(const char *name,
                                      const char *prefix,
                                      char *wheel,
                                      uint16 wheel_len,
                                      const char **field)
{
    const char *wheel_begin;
    const char *dot;
    size_t prefix_len = strlen(prefix);
    size_t token_len;

    if (strncmp(name, prefix, prefix_len) != 0)
    {
        return 0U;
    }
    wheel_begin = name + prefix_len;
    dot = strchr(wheel_begin, '.');
    if (dot == NULL || dot == wheel_begin || dot[1] == '\0' || strchr(dot + 1, '.') != NULL)
    {
        return 0U;
    }
    token_len = (size_t)(dot - wheel_begin);
    if (token_len >= wheel_len)
    {
        return 0U;
    }
    (void)memcpy(wheel, wheel_begin, token_len);
    wheel[token_len] = '\0';
    *field = dot + 1;
    return 1U;
}

static uint8 BlueSerial_TrySetWheelPid(const char *name, float value)
{
    char wheel_name[8];
    const char *field;
    int parsed_wheel;
    uint8 first_wheel;
    uint8 last_wheel;
    uint8 i;
    float applied_value;
    uint32 primask;

    if (!BlueSerial_SplitTuneName(name, "spd.", wheel_name,
                                  (uint16)sizeof(wheel_name), &field))
    {
        return 0U;
    }
    parsed_wheel = BlueSerial_ParseWheelToken(wheel_name);
    if (parsed_wheel < 0 ||
        (strcmp(field, "kp") != 0 && strcmp(field, "ki") != 0 &&
         strcmp(field, "kd") != 0 && strcmp(field, "alpha") != 0))
    {
        return 0U;
    }
    if (!BlueSerial_TuningIsIdle())
    {
        BlueSerial_Printf("ERR stop_first %s\r\n", name);
        return 1U;
    }

    if (strcmp(field, "kp") == 0)
    {
        applied_value = BlueSerial_ClampFloat(value, 0.0f, BLUESERIAL_MAX_WHEEL_KP);
    }
    else if (strcmp(field, "ki") == 0)
    {
        applied_value = BlueSerial_ClampFloat(value, 0.0f, BLUESERIAL_MAX_WHEEL_KI);
    }
    else if (strcmp(field, "kd") == 0)
    {
        applied_value = BlueSerial_ClampFloat(value, 0.0f, BLUESERIAL_MAX_WHEEL_KD);
    }
    else
    {
        applied_value = BlueSerial_ClampFloat(value, 0.0f, 1.0f);
    }

    first_wheel = (parsed_wheel == MOTOR_WHEEL_COUNT) ? 0U : (uint8)parsed_wheel;
    last_wheel = (parsed_wheel == MOTOR_WHEEL_COUNT) ?
                 (MOTOR_WHEEL_COUNT - 1U) : (uint8)parsed_wheel;
    primask = interrupt_global_disable();
    for (i = first_wheel; i <= last_wheel; ++i)
    {
        tagPID_T *pid = BlueSerial_GetWheelPid(i);
        PIDInitStruct *init = BlueSerial_GetWheelPidInit(i);

        if (strcmp(field, "kp") == 0)
        {
            init->fKp = applied_value;
        }
        else if (strcmp(field, "ki") == 0)
        {
            init->fKi = applied_value;
        }
        else if (strcmp(field, "kd") == 0)
        {
            init->fKd = applied_value;
        }
        else
        {
            init->alpha = applied_value;
        }
        PID_Update(pid, init);
        PID_Clear(pid);
    }
    g_speed_cfg_id++;
    interrupt_global_enable(primask);
    BlueSerial_Printf("OK spd.%s.%s=%.4f cfg=%lu\r\n",
                      wheel_name, field, applied_value,
                      (unsigned long)g_speed_cfg_id);
    return 1U;
}

static uint8 BlueSerial_TrySetDeadzone(const char *name, float value)
{
    char wheel_name[8];
    const char *field;
    int parsed_wheel;
    int applied_value;
    uint8 first_wheel;
    uint8 last_wheel;
    uint8 i;
    uint32 primask;

    if (strcmp(name, "dz.min") == 0)
    {
        if (!BlueSerial_TuningIsIdle())
        {
            BlueSerial_Printf("ERR stop_first %s\r\n", name);
            return 1U;
        }
        applied_value = BlueSerial_ClampInt(BlueSerial_RoundToInt(value),
                                            1, BLUESERIAL_MAX_DEADZONE_MIN_COUNTS);
        primask = interrupt_global_disable();
        motor_deadzone_target_min_counts = applied_value;
        g_speed_cfg_id++;
        interrupt_global_enable(primask);
        BlueSerial_Printf("OK dz.min=%dcount cfg=%lu\r\n",
                          applied_value, (unsigned long)g_speed_cfg_id);
        return 1U;
    }
    if (!BlueSerial_SplitTuneName(name, "dz.", wheel_name,
                                  (uint16)sizeof(wheel_name), &field))
    {
        return 0U;
    }
    parsed_wheel = BlueSerial_ParseWheelToken(wheel_name);
    if (parsed_wheel < 0 ||
        (strcmp(field, "fwd") != 0 && strcmp(field, "rev") != 0))
    {
        return 0U;
    }
    if (!BlueSerial_TuningIsIdle())
    {
        BlueSerial_Printf("ERR stop_first %s\r\n", name);
        return 1U;
    }

    applied_value = BlueSerial_ClampInt(BlueSerial_RoundToInt(value),
                                        0, BLUESERIAL_MAX_DEADZONE_PWM);
    first_wheel = (parsed_wheel == MOTOR_WHEEL_COUNT) ? 0U : (uint8)parsed_wheel;
    last_wheel = (parsed_wheel == MOTOR_WHEEL_COUNT) ?
                 (MOTOR_WHEEL_COUNT - 1U) : (uint8)parsed_wheel;
    primask = interrupt_global_disable();
    for (i = first_wheel; i <= last_wheel; ++i)
    {
        if (strcmp(field, "fwd") == 0)
        {
            motor_deadzone_fwd[i] = applied_value;
        }
        else
        {
            motor_deadzone_rev[i] = applied_value;
        }
    }
    g_speed_cfg_id++;
    interrupt_global_enable(primask);
    BlueSerial_Printf("OK dz.%s.%s=%d cfg=%lu\r\n",
                      wheel_name, field, applied_value,
                      (unsigned long)g_speed_cfg_id);
    return 1U;
}

static uint8 BlueSerial_SetSlider(const char *name, float value)
{
    uint32 primask;
    int wheel_index;
    int pwm_value;
    float applied_value;

    if (BlueSerial_TrySetWheelPid(name, value) ||
        BlueSerial_TrySetDeadzone(name, value))
    {
        return 1U;
    }
    if (strcmp(name, "test.profile") == 0 || strcmp(name, "test.yaw") == 0 ||
        strcmp(name, "spd.log") == 0)
    {
        uint8 enabled;

        if (!BlueSerial_TuningIsIdle())
        {
            BlueSerial_Printf("ERR stop_first %s\r\n", name);
            return 1U;
        }
        enabled = (value >= 0.5f) ? 1U : 0U;
        primask = interrupt_global_disable();
        if (strcmp(name, "test.profile") == 0)
        {
            g_speed_test_profile_enabled = enabled;
        }
        else if (strcmp(name, "test.yaw") == 0)
        {
            g_speed_test_yaw_enabled = enabled;
        }
        else
        {
            g_speed_log_enabled = enabled;
            if (!enabled)
            {
                BlueSerial_SpeedLogResetLocked();
            }
        }
        g_speed_cfg_id++;
        interrupt_global_enable(primask);
        BlueSerial_Printf("OK %s=%u cfg=%lu\r\n",
                          name, (unsigned int)enabled,
                          (unsigned long)g_speed_cfg_id);
        return 1U;
    }

    wheel_index = BlueSerial_GetLogicalWheelIndex(name);
    if (wheel_index >= 0)
    {
        if (g_control_mode != BLUESERIAL_MODE_RAW_PWM &&
            !BlueSerial_TuningIsIdle())
        {
            BlueSerial_Printf("ERR stop_first %s\r\n", name);
            return 1U;
        }
        pwm_value = BlueSerial_ClampInt(BlueSerial_RoundToInt(value), LIMIT_PWM_MIN, LIMIT_PWM_MAX);
        primask = interrupt_global_disable();
        g_raw_pwm[wheel_index] = pwm_value;
        BlueSerial_EnterRawPwmMode();
        interrupt_global_enable(primask);
        BlueSerial_Printf("OK pwm.%s=%d physical=%s\r\n",
                          BlueSerial_LogicalWheelName((uint8)wheel_index),
                          pwm_value,
                          BlueSerial_PhysicalMotorName((uint8)wheel_index));
        return 1U;
    }
    if (strcmp(name, "speed") == 0)
    {
        applied_value = BlueSerial_ClampFloat(value, 0.0f, BLUESERIAL_MAX_TARGET_SPEED_CMPS);
        primask = interrupt_global_disable();
        g_target_speed_cmps = applied_value;
        interrupt_global_enable(primask);
        BlueSerial_Printf("OK next_speed=%.1fcmps\r\n", applied_value);
        return 1U;
    }
    if (strcmp(name, "position") == 0 || strcmp(name, "distance") == 0)
    {
        applied_value = BlueSerial_ClampFloat(value, 0.0f, BLUESERIAL_MAX_TARGET_POSITION_CM);
        primask = interrupt_global_disable();
        g_target_position_cm = applied_value;
        interrupt_global_enable(primask);
        BlueSerial_Printf("OK next_position=%.1fcm\r\n", applied_value);
        return 1U;
    }
    if (strcmp(name, "cross.left") == 0 || strcmp(name, "ycross.left") == 0)
    {
        applied_value = BlueSerial_ClampFloat(value,
                                              -BLUESERIAL_MAX_Y_CROSSTALK_ABS,
                                              BLUESERIAL_MAX_Y_CROSSTALK_ABS);
        primask = interrupt_global_disable();
        path_y_crosstalk_left_x_comp_k = applied_value;
        interrupt_global_enable(primask);
        BlueSerial_Printf("OK cross.left=%.4f\r\n", applied_value);
        return 1U;
    }
    if (strcmp(name, "cross.right") == 0 || strcmp(name, "ycross.right") == 0)
    {
        applied_value = BlueSerial_ClampFloat(value,
                                              -BLUESERIAL_MAX_Y_CROSSTALK_ABS,
                                              BLUESERIAL_MAX_Y_CROSSTALK_ABS);
        primask = interrupt_global_disable();
        path_y_crosstalk_right_x_comp_k = applied_value;
        interrupt_global_enable(primask);
        BlueSerial_Printf("OK cross.right=%.4f\r\n", applied_value);
        return 1U;
    }
    if (strcmp(name, "line.kp") == 0 || strcmp(name, "guide.kp") == 0)
    {
        applied_value = BlueSerial_ClampFloat(value, 0.0f, BLUESERIAL_MAX_LINE_GUIDE_KP);
        primask = interrupt_global_disable();
        path_line_guide_kp = applied_value;
        interrupt_global_enable(primask);
        BlueSerial_Printf("OK line.kp=%.4f\r\n", applied_value);
        return 1U;
    }
    if (strcmp(name, "line.min") == 0 || strcmp(name, "guide.min") == 0)
    {
        applied_value = BlueSerial_ClampFloat(value,
                                              0.0f,
                                              BLUESERIAL_MAX_LINE_GUIDE_MIN_CMPS);
        primask = interrupt_global_disable();
        path_line_guide_min_cmps = applied_value;
        interrupt_global_enable(primask);
        BlueSerial_Printf("OK line.min=%.3fcmps\r\n", applied_value);
        return 1U;
    }
    if (strcmp(name, "pos.kp") == 0 || strcmp(name, "position.kp") == 0)
    {
        applied_value = BlueSerial_ClampFloat(value, 0.0f, BLUESERIAL_MAX_POSITION_KP);
        primask = interrupt_global_disable();
        path_follow_set_position_pid_gains_x(applied_value, pid_stay.fKi, pid_stay.fKd);
        path_follow_set_position_pid_gains_y(applied_value, pid_stay_y.fKi, pid_stay_y.fKd);
        path_follow_set_position_pid_gains_backward(
            applied_value, pid_stay_backward.fKi, pid_stay_backward.fKd);
        interrupt_global_enable(primask);
        BlueSerial_Printf("OK pos.kp=%.4f\r\n", applied_value);
        return 1U;
    }
    if (strcmp(name, "pos.ki") == 0 || strcmp(name, "position.ki") == 0)
    {
        applied_value = BlueSerial_ClampFloat(value, 0.0f, BLUESERIAL_MAX_POSITION_KI);
        primask = interrupt_global_disable();
        path_follow_set_position_pid_gains_x(pid_stay.fKp, applied_value, pid_stay.fKd);
        path_follow_set_position_pid_gains_y(pid_stay_y.fKp, applied_value, pid_stay_y.fKd);
        path_follow_set_position_pid_gains_backward(
            pid_stay_backward.fKp, applied_value, pid_stay_backward.fKd);
        interrupt_global_enable(primask);
        BlueSerial_Printf("OK pos.ki=%.4f\r\n", applied_value);
        return 1U;
    }
    if (strcmp(name, "pos.kd") == 0 || strcmp(name, "position.kd") == 0)
    {
        applied_value = BlueSerial_ClampFloat(value, 0.0f, BLUESERIAL_MAX_POSITION_KD);
        primask = interrupt_global_disable();
        path_follow_set_position_pid_gains_x(pid_stay.fKp, pid_stay.fKi, applied_value);
        path_follow_set_position_pid_gains_y(pid_stay_y.fKp, pid_stay_y.fKi, applied_value);
        path_follow_set_position_pid_gains_backward(
            pid_stay_backward.fKp, pid_stay_backward.fKi, applied_value);
        interrupt_global_enable(primask);
        BlueSerial_Printf("OK pos.kd=%.4f\r\n", applied_value);
        return 1U;
    }
    if (strcmp(name, "posf.kp") == 0 ||
        strcmp(name, "position.forward.kp") == 0)
    {
        applied_value = BlueSerial_ClampFloat(value, 0.0f, BLUESERIAL_MAX_POSITION_KP);
        primask = interrupt_global_disable();
        path_follow_set_position_pid_gains_forward(
            applied_value, pid_stay.fKi, pid_stay.fKd);
        interrupt_global_enable(primask);
        BlueSerial_Printf("OK posf.kp=%.4f\r\n", applied_value);
        return 1U;
    }
    if (strcmp(name, "posf.ki") == 0 ||
        strcmp(name, "position.forward.ki") == 0)
    {
        applied_value = BlueSerial_ClampFloat(value, 0.0f, BLUESERIAL_MAX_POSITION_KI);
        primask = interrupt_global_disable();
        path_follow_set_position_pid_gains_forward(
            pid_stay.fKp, applied_value, pid_stay.fKd);
        interrupt_global_enable(primask);
        BlueSerial_Printf("OK posf.ki=%.4f\r\n", applied_value);
        return 1U;
    }
    if (strcmp(name, "posf.kd") == 0 ||
        strcmp(name, "position.forward.kd") == 0)
    {
        applied_value = BlueSerial_ClampFloat(value, 0.0f, BLUESERIAL_MAX_POSITION_KD);
        primask = interrupt_global_disable();
        path_follow_set_position_pid_gains_forward(
            pid_stay.fKp, pid_stay.fKi, applied_value);
        interrupt_global_enable(primask);
        BlueSerial_Printf("OK posf.kd=%.4f\r\n", applied_value);
        return 1U;
    }
    if (strcmp(name, "posb.kp") == 0 ||
        strcmp(name, "position.backward.kp") == 0)
    {
        applied_value = BlueSerial_ClampFloat(value, 0.0f, BLUESERIAL_MAX_POSITION_KP);
        primask = interrupt_global_disable();
        path_follow_set_position_pid_gains_backward(
            applied_value, pid_stay_backward.fKi, pid_stay_backward.fKd);
        interrupt_global_enable(primask);
        BlueSerial_Printf("OK posb.kp=%.4f\r\n", applied_value);
        return 1U;
    }
    if (strcmp(name, "posb.ki") == 0 ||
        strcmp(name, "position.backward.ki") == 0)
    {
        applied_value = BlueSerial_ClampFloat(value, 0.0f, BLUESERIAL_MAX_POSITION_KI);
        primask = interrupt_global_disable();
        path_follow_set_position_pid_gains_backward(
            pid_stay_backward.fKp, applied_value, pid_stay_backward.fKd);
        interrupt_global_enable(primask);
        BlueSerial_Printf("OK posb.ki=%.4f\r\n", applied_value);
        return 1U;
    }
    if (strcmp(name, "posb.kd") == 0 ||
        strcmp(name, "position.backward.kd") == 0)
    {
        applied_value = BlueSerial_ClampFloat(value, 0.0f, BLUESERIAL_MAX_POSITION_KD);
        primask = interrupt_global_disable();
        path_follow_set_position_pid_gains_backward(
            pid_stay_backward.fKp, pid_stay_backward.fKi, applied_value);
        interrupt_global_enable(primask);
        BlueSerial_Printf("OK posb.kd=%.4f\r\n", applied_value);
        return 1U;
    }
    if (strcmp(name, "pos.envelope") == 0 ||
        strcmp(name, "position.envelope") == 0 ||
        strcmp(name, "envelope.factor") == 0)
    {
        applied_value = BlueSerial_ClampFloat(value,
                                              BLUESERIAL_MIN_POSITION_ENVELOPE,
                                              BLUESERIAL_MAX_POSITION_ENVELOPE);
        primask = interrupt_global_disable();
        path_follow_set_position_speed_factor(applied_value);
        interrupt_global_enable(primask);
        BlueSerial_Printf("OK pos.envelope=%.3f\r\n", applied_value);
        return 1U;
    }
    if (strcmp(name, "posx.kp") == 0 || strcmp(name, "position.x.kp") == 0)
    {
        applied_value = BlueSerial_ClampFloat(value, 0.0f, BLUESERIAL_MAX_POSITION_KP);
        primask = interrupt_global_disable();
        path_follow_set_position_pid_gains_x(applied_value, pid_stay.fKi, pid_stay.fKd);
        interrupt_global_enable(primask);
        BlueSerial_Printf("OK posx.kp=%.4f\r\n", applied_value);
        return 1U;
    }
    if (strcmp(name, "posx.ki") == 0 || strcmp(name, "position.x.ki") == 0)
    {
        applied_value = BlueSerial_ClampFloat(value, 0.0f, BLUESERIAL_MAX_POSITION_KI);
        primask = interrupt_global_disable();
        path_follow_set_position_pid_gains_x(pid_stay.fKp, applied_value, pid_stay.fKd);
        interrupt_global_enable(primask);
        BlueSerial_Printf("OK posx.ki=%.4f\r\n", applied_value);
        return 1U;
    }
    if (strcmp(name, "posx.kd") == 0 || strcmp(name, "position.x.kd") == 0)
    {
        applied_value = BlueSerial_ClampFloat(value, 0.0f, BLUESERIAL_MAX_POSITION_KD);
        primask = interrupt_global_disable();
        path_follow_set_position_pid_gains_x(pid_stay.fKp, pid_stay.fKi, applied_value);
        interrupt_global_enable(primask);
        BlueSerial_Printf("OK posx.kd=%.4f\r\n", applied_value);
        return 1U;
    }
    if (strcmp(name, "posy.kp") == 0 || strcmp(name, "position.y.kp") == 0)
    {
        applied_value = BlueSerial_ClampFloat(value, 0.0f, BLUESERIAL_MAX_POSITION_KP);
        primask = interrupt_global_disable();
        path_follow_set_position_pid_gains_y(applied_value, pid_stay_y.fKi, pid_stay_y.fKd);
        interrupt_global_enable(primask);
        BlueSerial_Printf("OK posy.kp=%.4f\r\n", applied_value);
        return 1U;
    }
    if (strcmp(name, "posy.ki") == 0 || strcmp(name, "position.y.ki") == 0)
    {
        applied_value = BlueSerial_ClampFloat(value, 0.0f, BLUESERIAL_MAX_POSITION_KI);
        primask = interrupt_global_disable();
        path_follow_set_position_pid_gains_y(pid_stay_y.fKp, applied_value, pid_stay_y.fKd);
        interrupt_global_enable(primask);
        BlueSerial_Printf("OK posy.ki=%.4f\r\n", applied_value);
        return 1U;
    }
    if (strcmp(name, "posy.kd") == 0 || strcmp(name, "position.y.kd") == 0)
    {
        applied_value = BlueSerial_ClampFloat(value, 0.0f, BLUESERIAL_MAX_POSITION_KD);
        primask = interrupt_global_disable();
        path_follow_set_position_pid_gains_y(pid_stay_y.fKp, pid_stay_y.fKi, applied_value);
        interrupt_global_enable(primask);
        BlueSerial_Printf("OK posy.kd=%.4f\r\n", applied_value);
        return 1U;
    }
    if (strcmp(name, "posx.envelope") == 0 ||
        strcmp(name, "position.x.envelope") == 0)
    {
        applied_value = BlueSerial_ClampFloat(value,
                                              BLUESERIAL_MIN_POSITION_ENVELOPE,
                                              BLUESERIAL_MAX_POSITION_ENVELOPE);
        primask = interrupt_global_disable();
        path_follow_set_position_speed_factor_x(applied_value);
        interrupt_global_enable(primask);
        BlueSerial_Printf("OK posx.envelope=%.3f\r\n", applied_value);
        return 1U;
    }
    if (strcmp(name, "posy.envelope") == 0 ||
        strcmp(name, "position.y.envelope") == 0)
    {
        applied_value = BlueSerial_ClampFloat(value,
                                              BLUESERIAL_MIN_POSITION_ENVELOPE,
                                              BLUESERIAL_MAX_POSITION_ENVELOPE);
        primask = interrupt_global_disable();
        path_follow_set_position_speed_factor_y(applied_value);
        interrupt_global_enable(primask);
        BlueSerial_Printf("OK posy.envelope=%.3f\r\n", applied_value);
        return 1U;
    }
    if (strcmp(name, "yaw.kp") == 0)
    {
        applied_value = BlueSerial_ClampFloat(value, 0.0f, BLUESERIAL_MAX_YAW_KP);
        primask = interrupt_global_disable();
        pid_yaw.fKp = applied_value;
        PID_Clear(&pid_yaw);
        interrupt_global_enable(primask);
        BlueSerial_Printf("OK yaw.kp=%.4f\r\n", applied_value);
        return 1U;
    }
    if (strcmp(name, "yaw.ki") == 0)
    {
        applied_value = BlueSerial_ClampFloat(value, 0.0f, BLUESERIAL_MAX_YAW_KI);
        primask = interrupt_global_disable();
        pid_yaw.fKi = applied_value;
        PID_Clear(&pid_yaw);
        interrupt_global_enable(primask);
        BlueSerial_Printf("OK yaw.ki=%.4f\r\n", applied_value);
        return 1U;
    }
    if (strcmp(name, "yaw.kd") == 0)
    {
        applied_value = BlueSerial_ClampFloat(value, 0.0f, BLUESERIAL_MAX_YAW_KD);
        primask = interrupt_global_disable();
        pid_yaw.fKd = applied_value;
        PID_Clear(&pid_yaw);
        interrupt_global_enable(primask);
        BlueSerial_Printf("OK yaw.kd=%.4f\r\n", applied_value);
        return 1U;
    }
    if (strcmp(name, "yaw.ff") == 0)
    {
        applied_value = BlueSerial_ClampFloat(value, 0.0f, BLUESERIAL_MAX_YAW_FF_DEGPS);
        primask = interrupt_global_disable();
        path_yaw_feedforward_min_degps = applied_value;
        interrupt_global_enable(primask);
        BlueSerial_Printf("OK yaw.ff=%.2fdegps\r\n", applied_value);
        return 1U;
    }
    if (strcmp(name, "rotate.kp") == 0 || strcmp(name, "turn.kp") == 0)
    {
        float rotate_ki;
        float rotate_kd;

        applied_value = BlueSerial_ClampFloat(value, 0.0f, BLUESERIAL_MAX_YAW_KP);
        primask = interrupt_global_disable();
        path_follow_get_rotate_yaw_pid_gains(NULL, &rotate_ki, &rotate_kd);
        path_follow_set_rotate_yaw_pid_gains(applied_value, rotate_ki, rotate_kd);
        interrupt_global_enable(primask);
        BlueSerial_Printf("OK rotate.kp=%.4f\r\n", applied_value);
        return 1U;
    }
    if (strcmp(name, "rotate.ki") == 0 || strcmp(name, "turn.ki") == 0)
    {
        float rotate_kp;
        float rotate_kd;

        applied_value = BlueSerial_ClampFloat(value, 0.0f, BLUESERIAL_MAX_YAW_KI);
        primask = interrupt_global_disable();
        path_follow_get_rotate_yaw_pid_gains(&rotate_kp, NULL, &rotate_kd);
        path_follow_set_rotate_yaw_pid_gains(rotate_kp, applied_value, rotate_kd);
        interrupt_global_enable(primask);
        BlueSerial_Printf("OK rotate.ki=%.4f\r\n", applied_value);
        return 1U;
    }
    if (strcmp(name, "rotate.kd") == 0 || strcmp(name, "turn.kd") == 0)
    {
        float rotate_kp;
        float rotate_ki;

        applied_value = BlueSerial_ClampFloat(value, 0.0f, BLUESERIAL_MAX_YAW_KD);
        primask = interrupt_global_disable();
        path_follow_get_rotate_yaw_pid_gains(&rotate_kp, &rotate_ki, NULL);
        path_follow_set_rotate_yaw_pid_gains(rotate_kp, rotate_ki, applied_value);
        interrupt_global_enable(primask);
        BlueSerial_Printf("OK rotate.kd=%.4f\r\n", applied_value);
        return 1U;
    }
    if (strcmp(name, "rotate.ff") == 0 || strcmp(name, "turn.ff") == 0)
    {
        applied_value = BlueSerial_ClampFloat(value, 0.0f, BLUESERIAL_MAX_YAW_FF_DEGPS);
        primask = interrupt_global_disable();
        path_follow_set_rotate_yaw_feedforward(applied_value);
        interrupt_global_enable(primask);
        BlueSerial_Printf("OK rotate.ff=%.2fdegps\r\n", applied_value);
        return 1U;
    }

    return 0U;
}

static void BlueSerial_PrintStatus(void)
{
    uint32 primask;
    uint32 drop_count;
    blueserial_move_direction_t direction;
    unsigned int mode;
    unsigned int yaw_hold;
    int pwm[4];
    float speed_cmps;
    float position_cm;
    float yaw_kp;
    float yaw_ki;
    float yaw_kd;
    float yaw_ff;
    float rotate_kp;
    float rotate_ki;
    float rotate_kd;
    float rotate_ff;
    float cross_left;
    float cross_right;
    float line_guide_kp;
    float line_guide_min_cmps;
    float position_x_kp;
    float position_x_ki;
    float position_x_kd;
    float position_y_kp;
    float position_y_ki;
    float position_y_kd;
    float position_backward_x_kp;
    float position_backward_x_ki;
    float position_backward_x_kd;
    float position_x_envelope;
    float position_y_envelope;
    uint8 point_target_valid;
    float point_target_x_m;
    float point_target_y_m;

    primask = interrupt_global_disable();
    mode = (unsigned int)g_control_mode;
    yaw_hold = (unsigned int)g_yaw_hold_enabled;
    direction = g_next_move_direction;
    speed_cmps = g_target_speed_cmps;
    position_cm = g_target_position_cm;
    pwm[0] = g_raw_pwm[0];
    pwm[1] = g_raw_pwm[1];
    pwm[2] = g_raw_pwm[2];
    pwm[3] = g_raw_pwm[3];
    yaw_kp = pid_yaw.fKp;
    yaw_ki = pid_yaw.fKi;
    yaw_kd = pid_yaw.fKd;
    yaw_ff = path_yaw_feedforward_min_degps;
    path_follow_get_rotate_yaw_pid_gains(&rotate_kp, &rotate_ki, &rotate_kd);
    rotate_ff = path_follow_get_rotate_yaw_feedforward();
    cross_left = path_y_crosstalk_left_x_comp_k;
    cross_right = path_y_crosstalk_right_x_comp_k;
    line_guide_kp = path_line_guide_kp;
    line_guide_min_cmps = path_line_guide_min_cmps;
    position_x_kp = pid_stay.fKp;
    position_x_ki = pid_stay.fKi;
    position_x_kd = pid_stay.fKd;
    position_y_kp = pid_stay_y.fKp;
    position_y_ki = pid_stay_y.fKi;
    position_y_kd = pid_stay_y.fKd;
    position_backward_x_kp = pid_stay_backward.fKp;
    position_backward_x_ki = pid_stay_backward.fKi;
    position_backward_x_kd = pid_stay_backward.fKd;
    position_x_envelope = path_follow_get_position_speed_factor_x();
    position_y_envelope = path_follow_get_position_speed_factor_y();
    point_target_valid = g_point_target_valid;
    point_target_x_m = g_point_target_x_m;
    point_target_y_m = g_point_target_y_m;
    drop_count = g_rx_drop_count;
    interrupt_global_enable(primask);

    BlueSerial_Printf("STATUS mode=%u yawhold=%u direction=%s next_speed=%.1fcmps next_position=%.1fcm "
                      "pwm.ulurdlr=%d,%d,%d,%d "
                      "yawpid=%.4f,%.4f,%.4f ff=%.2f drop=%lu\r\n",
                      mode,
                      yaw_hold,
                      BlueSerial_MoveDirectionName(direction),
                      speed_cmps,
                      position_cm,
                      pwm[0], pwm[1], pwm[2], pwm[3],
                      yaw_kp, yaw_ki, yaw_kd,
                      yaw_ff,
                      (unsigned long)drop_count);
    BlueSerial_Printf("TUNE rotatepid=%.4f,%.4f,%.4f rotate.ff=%.2fdegps\r\n",
                      rotate_kp, rotate_ki, rotate_kd, rotate_ff);
    BlueSerial_Printf("TUNE cross.left=%.4f cross.right=%.4f "
                      "posx=%.4f,%.4f,%.4f envx=%.3f "
                      "posy=%.4f,%.4f,%.4f envy=%.3f\r\n",
                      cross_left,
                      cross_right,
                      position_x_kp,
                      position_x_ki,
                      position_x_kd,
                      position_x_envelope,
                      position_y_kp,
                      position_y_ki,
                      position_y_kd,
                      position_y_envelope);
    BlueSerial_Printf("TUNE line.kp=%.4f line.min=%.3fcmps\r\n",
                      line_guide_kp,
                      line_guide_min_cmps);
    BlueSerial_Printf("TUNE posback=%.4f,%.4f,%.4f\r\n",
                      position_backward_x_kp,
                      position_backward_x_ki,
                      position_backward_x_kd);
    if (point_target_valid)
    {
        BlueSerial_Printf("TARGET pending=1 target_m=%.3f,%.3f\r\n",
                          point_target_x_m,
                          point_target_y_m);
    }
    BlueSerial_PrintSpeedTuneStatus();
}

static uint8 BlueSerial_RunButton(const char *command)
{
    if (strcmp(command, "forward") == 0 || strcmp(command, "go ahead") == 0)
    {
        BlueSerial_SelectMoveDirection(BLUESERIAL_MOVE_FORWARD);
        return 1U;
    }
    if (strcmp(command, "backward") == 0 || strcmp(command, "go back") == 0)
    {
        BlueSerial_SelectMoveDirection(BLUESERIAL_MOVE_BACKWARD);
        return 1U;
    }
    if (strcmp(command, "left") == 0)
    {
        BlueSerial_SelectMoveDirection(BLUESERIAL_MOVE_LEFT);
        return 1U;
    }
    if (strcmp(command, "right") == 0)
    {
        BlueSerial_SelectMoveDirection(BLUESERIAL_MOVE_RIGHT);
        return 1U;
    }
    if (strcmp(command, "start") == 0 || strcmp(command, "run") == 0 ||
        strcmp(command, "apply") == 0 || strcmp(command, "move") == 0)
    {
        if (g_speed_summary_pending || g_speed_log_dumping)
        {
            BlueSerial_Printf("ERR wait_speed_report\r\n");
            return 1U;
        }
        if (!BlueSerial_TuningIsIdle())
        {
            BlueSerial_Printf("ERR stop_first start\r\n");
            return 1U;
        }
        if (BlueSerial_HasPointTarget())
        {
            if (!BlueSerial_StartPointTargetMove())
            {
                BlueSerial_Printf("ERR target_start_failed\r\n");
            }
            return 1U;
        }
        if (!BlueSerial_StartConfiguredMove())
        {
            BlueSerial_Printf("ERR set_next_speed_and_position_first\r\n");
            return 1U;
        }
        BlueSerial_Printf("OK start direction=%s speed=%.1fcmps position=%.1fcm\r\n",
                          BlueSerial_MoveDirectionName(g_next_move_direction),
                          g_target_speed_cmps,
                          g_target_position_cm);
        return 1U;
    }
    if (strcmp(command, "speed") == 0 || strcmp(command, "cruise") == 0 ||
        strcmp(command, "speed.start") == 0)
    {
        if (g_speed_summary_pending || g_speed_log_dumping)
        {
            BlueSerial_Printf("ERR wait_speed_report\r\n");
            return 1U;
        }
        if (!BlueSerial_TuningIsIdle())
        {
            BlueSerial_Printf("ERR stop_first speed\r\n");
            return 1U;
        }
        if (!BlueSerial_StartSpeedCruise())
        {
            BlueSerial_Printf("ERR set_target_speed_first\r\n");
            return 1U;
        }
        BlueSerial_Printf("OK speed_cruise direction=%s speed=%.1fcmps\r\n",
                          BlueSerial_MoveDirectionName(g_cruise_move_direction),
                          g_target_speed_cmps);
        return 1U;
    }
    if (strcmp(command, "turn90") == 0)
    {
        if (!BlueSerial_TuningIsIdle())
        {
            BlueSerial_Printf("ERR stop_first turn90\r\n");
            return 1U;
        }
        BlueSerial_StartTurn(1U, 90.0f);
        BlueSerial_Printf("OK turn90 ccw\r\n");
        return 1U;
    }
    if (strcmp(command, "turn360") == 0)
    {
        if (!BlueSerial_TuningIsIdle())
        {
            BlueSerial_Printf("ERR stop_first turn360\r\n");
            return 1U;
        }
        BlueSerial_StartTurn(4U, 90.0f);
        BlueSerial_Printf("OK turn360 ccw\r\n");
        return 1U;
    }
    if (strcmp(command, "turn90r") == 0)
    {
        if (!BlueSerial_TuningIsIdle())
        {
            BlueSerial_Printf("ERR stop_first turn90r\r\n");
            return 1U;
        }
        BlueSerial_StartTurn(1U, -90.0f);
        BlueSerial_Printf("OK turn90 cw\r\n");
        return 1U;
    }
    if (strcmp(command, "turn360r") == 0)
    {
        if (!BlueSerial_TuningIsIdle())
        {
            BlueSerial_Printf("ERR stop_first turn360r\r\n");
            return 1U;
        }
        BlueSerial_StartTurn(4U, -90.0f);
        BlueSerial_Printf("OK turn360 cw\r\n");
        return 1U;
    }
    if (strcmp(command, "stop") == 0)
    {
        BlueSerial_StopControl();
        BlueSerial_Printf("OK stop\r\n");
        return 1U;
    }
    if (strcmp(command, "yawhold") == 0 || strcmp(command, "yaw.hold") == 0)
    {
        BlueSerial_ToggleYawHold();
        return 1U;
    }
    if (strcmp(command, "spd.status") == 0)
    {
        BlueSerial_PrintSpeedTuneStatus();
        return 1U;
    }
    if (strcmp(command, "spd.help") == 0)
    {
        BlueSerial_Printf("SPDHELP [slider,test.profile,0|1] [slider,test.yaw,0|1] "
                          "[slider,spd.log,0|1]\r\n");
        BlueSerial_Printf("SPDHELP [slider,spd.<ul|ur|dl|dr|all>.<kp|ki|kd|alpha>,v]\r\n");
        BlueSerial_Printf("SPDHELP [slider,dz.<ul|ur|dl|dr|all>.<fwd|rev>,v] "
                          "[slider,dz.min,v]\r\n");
        BlueSerial_Printf("SPDHELP [spd.status] [spd.summary] [spd.dump] "
                          "[spd.dump.stop] [spd.reset]\r\n");
        return 1U;
    }
    if (strcmp(command, "yaw.help") == 0 || strcmp(command, "rotate.help") == 0)
    {
        BlueSerial_Printf("YAWHELP hold [slider,yaw.<kp|ki|kd|ff>,v]\r\n");
        BlueSerial_Printf("YAWHELP rotate [slider,rotate.<kp|ki|kd|ff>,v] "
                          "[turn90|turn90r|turn360|turn360r]\r\n");
        return 1U;
    }
    if (strcmp(command, "spd.summary") == 0)
    {
        g_speed_summary_pending = 0U;
        BlueSerial_PrintSpeedSummary();
        return 1U;
    }
    if (strcmp(command, "spd.dump") == 0)
    {
        if (g_speed_log_recording || !g_speed_log_ready)
        {
            BlueSerial_Printf("ERR speed_log_not_ready rec=%u\r\n",
                              (unsigned int)g_speed_log_recording);
            return 1U;
        }
        g_speed_log_dump_index = 0U;
        g_speed_log_dumping = 1U;
        BlueSerial_Printf("OK spd.dump ID=%lu rows=%u total=%lu ovf=%u\r\n",
                          (unsigned long)g_speed_log_run_id,
                          (unsigned int)g_speed_log_count,
                          (unsigned long)g_speed_log_total_ticks,
                          (unsigned int)g_speed_log_overflow);
        return 1U;
    }
    if (strcmp(command, "spd.dump.stop") == 0)
    {
        g_speed_log_dumping = 0U;
        BlueSerial_Printf("OK spd.dump.stop\r\n");
        return 1U;
    }
    if (strcmp(command, "spd.reset") == 0)
    {
        uint32 primask;

        if (!BlueSerial_TuningIsIdle())
        {
            BlueSerial_Printf("ERR stop_first spd.reset\r\n");
            return 1U;
        }
        primask = interrupt_global_disable();
        BlueSerial_SpeedLogResetLocked();
        interrupt_global_enable(primask);
        BlueSerial_Printf("OK spd.reset\r\n");
        return 1U;
    }
    if (strcmp(command, "status") == 0)
    {
        BlueSerial_PrintStatus();
        return 1U;
    }
    if (strcmp(command, "position") == 0)
    {
        BlueSerial_StartPositionRequest();
        return 1U;
    }
    if (strcmp(command, "IMG1") == 0 || strcmp(command, "img1") == 0)
    {
        BlueSerial_StartVisionRequest(VISION_RECOGNITION_IMG,
                                      VISION_RECOGNITION_DISTANCE_ONE_GRID);
        return 1U;
    }
    if (strcmp(command, "IMG3") == 0 || strcmp(command, "img3") == 0)
    {
        BlueSerial_StartVisionRequest(VISION_RECOGNITION_IMG,
                                      VISION_RECOGNITION_DISTANCE_THREE_GRID);
        return 1U;
    }
    if (strcmp(command, "NUM1") == 0 || strcmp(command, "num1") == 0)
    {
        BlueSerial_StartVisionRequest(VISION_RECOGNITION_NUM,
                                      VISION_RECOGNITION_DISTANCE_ONE_GRID);
        return 1U;
    }
    if (strcmp(command, "NUM3") == 0 || strcmp(command, "num3") == 0)
    {
        BlueSerial_StartVisionRequest(VISION_RECOGNITION_NUM,
                                      VISION_RECOGNITION_DISTANCE_THREE_GRID);
        return 1U;
    }

    return 0U;
}

static void BlueSerial_ParseCommand(char *raw_command)
{
    char type[16];
    char name[32];
    char *trimmed_type;
    char *trimmed_name;
    float value;
    float target_x_m;
    char extra;
    char *command = BlueSerial_TrimCommand(raw_command);

    if (sscanf(command, "%15[^,],%31[^,],%f", type, name, &value) == 3)
    {
        trimmed_type = BlueSerial_TrimCommand(type);
        trimmed_name = BlueSerial_TrimCommand(name);
        if (strcmp(trimmed_type, "target") == 0 ||
            strcmp(trimmed_type, "goto") == 0 ||
            strcmp(trimmed_type, "point") == 0)
        {
            if (sscanf(trimmed_name, "%f %c", &target_x_m, &extra) != 1)
            {
                BlueSerial_Printf("ERR invalid_target_x\r\n");
                return;
            }
            (void)BlueSerial_SetPointTarget(target_x_m, value);
            return;
        }
        if (strcmp(trimmed_type, "slider") != 0)
        {
            BlueSerial_Printf("ERR unknown_frame_type %s\r\n", trimmed_type);
            return;
        }
        if (!isfinite(value))
        {
            BlueSerial_Printf("ERR invalid_value %s\r\n", trimmed_name);
            return;
        }
        if (BlueSerial_SetSlider(trimmed_name, value))
        {
            return;
        }
        BlueSerial_Printf("ERR unknown_slider %s\r\n", trimmed_name);
        return;
    }

    if (strncmp(command, "button,", 7U) == 0)
    {
        command = BlueSerial_TrimCommand(command + 7U);
    }

    if (!BlueSerial_RunButton(command))
    {
        BlueSerial_Printf("ERR unknown_command %s\r\n", command);
    }
}

static void BlueSerial_CommandTask(void)
{
    char command[BLUESERIAL_RX_FRAME_LEN];
    uint8 read_index;

    while (g_rx_queue_read != g_rx_queue_write)
    {
        read_index = g_rx_queue_read;
        (void)strncpy(command, g_rx_queue[read_index], sizeof(command) - 1U);
        command[sizeof(command) - 1U] = '\0';
        g_rx_queue_read = (uint8)((read_index + 1U) % BLUESERIAL_RX_QUEUE_LEN);
        BlueSerial_ParseCommand(command);
    }
}

void BlueSerial_TelemetryTick10ms(void)
{
    if (!g_enabled)
    {
        return;
    }
    g_telemetry_tick_count++;
    if (g_telemetry_tick_count >= BLUESERIAL_TELEMETRY_PERIOD_TICKS)
    {
        g_telemetry_tick_count = 0U;
        g_telemetry_sequence++;
        g_telemetry_report_pending = 1U;
    }
}

static void BlueSerial_ServiceTelemetryReport(void)
{
    uint32 primask;
    uint32 sequence = 0U;
    uint8 report_pending;
    uint8 report_active;
    uint8 speed_recording = 0U;
    uint32 speed_total_ticks = 0U;
    unsigned int mode = 0U;
    path_follow_status_t status = {0};
    motor_speed_debug_snapshot_t motor = {0};
    float actual_yaw_deg = 0.0f;

    primask = interrupt_global_disable();
    report_pending = g_telemetry_report_pending;
    if (report_pending)
    {
        g_telemetry_report_pending = 0U;
        sequence = g_telemetry_sequence;
        mode = (unsigned int)g_control_mode;
        path_follow_get_status(&status);
        actual_yaw_deg = eulerAngle.yaw;
        speed_recording = g_speed_log_recording;
        speed_total_ticks = g_speed_log_total_ticks;
        if (speed_recording)
        {
            motor_speed_debug_get_snapshot(&motor);
        }
    }
    interrupt_global_enable(primask);

    if (!report_pending)
    {
        return;
    }

    report_active = (status.active || mode != (unsigned int)BLUESERIAL_MODE_STOP) ? 1U : 0U;
    if (!report_active && !g_telemetry_was_active)
    {
        return;
    }
    g_telemetry_was_active = report_active;

    if (speed_recording)
    {
        if ((sequence % 10U) == 0U)
        {
            BlueSerial_Printf("SPDLIVE ID=%lu K=%lu TEST=%u YAW=%u "
                              "T=%d,%d,%d,%d F=%d,%d,%d,%d Y=%.2f P=%.2f/%.2f\r\n",
                              (unsigned long)g_speed_log_run_id,
                              (unsigned long)speed_total_ticks,
                              (unsigned int)status.speed_test_enabled,
                              (unsigned int)status.speed_test_yaw_enabled,
                              motor.target_counts[0], motor.target_counts[1],
                              motor.target_counts[2], motor.target_counts[3],
                              motor.filtered_counts[0], motor.filtered_counts[1],
                              motor.filtered_counts[2], motor.filtered_counts[3],
                              BlueSerial_WrapDeg(actual_yaw_deg - g_speed_log_start_yaw_deg),
                              status.speed_test_profile_time_s,
                              status.speed_test_profile_total_s);
        }
        return;
    }

    BlueSerial_Printf("CAR100 n=%lu TPOS=%.3f,%.3f APOS=%.3f,%.3f "
                      "TYAW=%.2f AYAW=%.2f YERR=%.2f\r\n",
                      (unsigned long)sequence,
                      status.target_x_m,
                      status.target_y_m,
                      status.x_m,
                      status.y_m,
                      status.target_yaw_deg,
                      actual_yaw_deg,
                      status.yaw_error_deg);
}

void BlueSerial_ControlTick10ms(void)
{
    if (g_control_mode == BLUESERIAL_MODE_RAW_PWM)
    {
        path_follow_output_t unused_output = {0};

        /*
         * 原始 PWM 调试时也更新一次里程计。这样退出 PWM 后再用路径遥控，
         * path_follow 内部位姿不会长时间停留在旧值。
         */
        path_follow_update(eulerAngle.yaw, &unused_output);
        BlueSerial_ApplyRawPwm();
        return;
    }

    if (g_control_mode == BLUESERIAL_MODE_PATH_MOTION)
    {
        distance_speed_strategy();
        motor_control(speed_encoder);
        BlueSerial_SpeedLogCapture10ms();
        return;
    }

    if (g_control_mode == BLUESERIAL_MODE_SPEED_CRUISE)
    {
        path_follow_output_t yaw_hold = {0};

        path_follow_update_yaw_hold(eulerAngle.yaw, &yaw_hold);
        switch (g_cruise_move_direction)
        {
            case BLUESERIAL_MOVE_FORWARD:
                speed_three_array[0] = g_target_speed_cmps;
                speed_three_array[1] = 0.0f;
                break;
            case BLUESERIAL_MOVE_BACKWARD:
                speed_three_array[0] = -g_target_speed_cmps;
                speed_three_array[1] = 0.0f;
                break;
            case BLUESERIAL_MOVE_LEFT:
                speed_three_array[0] = 0.0f;
                speed_three_array[1] = g_target_speed_cmps;
                break;
            case BLUESERIAL_MOVE_RIGHT:
                speed_three_array[0] = 0.0f;
                speed_three_array[1] = -g_target_speed_cmps;
                break;
            default:
                speed_three_array[0] = 0.0f;
                speed_three_array[1] = 0.0f;
                break;
        }
        speed_three_array[2] = g_speed_test_yaw_enabled ? yaw_hold.omega_cmd : 0.0f;
        Kinematics_Inverse(speed_three_array, speed_encoder);
        motor_control(speed_encoder);
        BlueSerial_SpeedLogCapture10ms();
        return;
    }

    if (g_yaw_hold_enabled)
    {
        distance_speed_strategy();
        motor_control(speed_encoder);
        return;
    }

    motor_pwm(0, 0, 0, 0);
}

void BlueSerial_Task(void)
{
    if (!g_enabled)
    {
        return;
    }
    BlueSerial_ServiceMotionCompletion();
    BlueSerial_ServiceSpeedSummary();
    BlueSerial_CommandTask();
    BlueSerial_ServiceMotionCompletion();
    BlueSerial_ServiceSpeedSummary();
    BlueSerial_ServicePositionRequest();
    BlueSerial_ServiceVisionRequest();
    BlueSerial_ServiceTelemetryReport();
    BlueSerial_ServiceSpeedDump();
}
