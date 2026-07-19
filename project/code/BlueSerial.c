/*
 * UART8 蓝牙串口调参和遥控模块。
 *
 * 兼容的蓝牙帧格式：
 *   [slider,name,value]  滑条调参，例如 [slider,speed,40]
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
#define BLUESERIAL_MAX_TARGET_SPEED_CMPS        150.0f
#define BLUESERIAL_MAX_TARGET_POSITION_CM       500.0f
#define BLUESERIAL_MAX_YAW_KP                   20.0f
#define BLUESERIAL_MAX_YAW_KI                   10.0f
#define BLUESERIAL_MAX_YAW_KD                   50.0f
#define BLUESERIAL_MAX_YAW_FF_DEGPS             120.0f
#define BLUESERIAL_MAX_Y_CROSSTALK_ABS          0.100f
#define BLUESERIAL_MAX_POSITION_KP               20.0f
#define BLUESERIAL_MAX_POSITION_KI               10.0f
#define BLUESERIAL_MAX_POSITION_KD               50.0f
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

static volatile uint8 g_rx_collecting = 0U;
static volatile uint8 g_rx_work_len = 0U;
static char g_rx_work[BLUESERIAL_RX_FRAME_LEN];
static char g_rx_queue[BLUESERIAL_RX_QUEUE_LEN][BLUESERIAL_RX_FRAME_LEN];
static volatile uint8 g_rx_queue_write = 0U;
static volatile uint8 g_rx_queue_read = 0U;
static volatile uint32 g_rx_drop_count = 0U;
static char g_last_rx_frame[BLUESERIAL_RX_FRAME_LEN] = "";
static volatile uint8 g_last_rx_frame_len = 0U;
/* 正式比赛默认关闭；菜单/Flash 只在需要调车时显式打开。 */
static volatile uint8 g_blueserial_enabled = 0U;
static uint8 g_blueserial_uart_initialized = 0U;

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

static void BlueSerial_ClearWheelPid(void)
{
    PID_Clear(&ULpid);
    PID_Clear(&URpid);
    PID_Clear(&DLpid);
    PID_Clear(&DRpid);
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

/* 调用者需关中断，防止 UART8 ISR 与主循环同时修改收帧队列。 */
static void BlueSerial_ResetRxStateLocked(void)
{
    g_rx_collecting = 0U;
    g_rx_work_len = 0U;
    g_rx_queue_write = 0U;
    g_rx_queue_read = 0U;
    g_rx_drop_count = 0U;
    g_last_rx_frame[0] = '\0';
    g_last_rx_frame_len = 0U;
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

    BlueSerial_StopControlLocked();
    interrupt_global_enable(primask);
}

void BlueSerial_SetEnabled(uint8 enabled)
{
    uint32 primask;
    uint8 discard = 0U;
    uint8 normalized = (enabled != 0U) ? 1U : 0U;

    /* 先关外设 RX 中断，再修改被 ISR 和主循环共享的状态。 */
    if (g_blueserial_uart_initialized)
    {
        uart_rx_interrupt(BLUESERIAL_UART, ZF_DISABLE);
    }

    primask = interrupt_global_disable();
    if (!normalized &&
        (g_control_mode != BLUESERIAL_MODE_STOP || g_yaw_hold_enabled))
    {
        /* 仅当蓝牙确实占用底盘时停车，避免误清除自动控制路径。 */
        BlueSerial_StopControlLocked();
    }
    g_blueserial_enabled = normalized;
    BlueSerial_ResetRxStateLocked();
    g_position_request_pending = 0U;
    g_vision_request_pending = 0U;
    interrupt_global_enable(primask);

    if (g_blueserial_uart_initialized && normalized)
    {
        /* 丢弃关闭期间积存在硬件 FIFO 中的旧命令，禁止重新开启后误动作。 */
        while (uart_query_byte(BLUESERIAL_UART, &discard))
        {
        }
        uart_rx_interrupt(BLUESERIAL_UART, ZF_ENABLE);
    }
}

uint8 BlueSerial_GetEnabled(void)
{
    return g_blueserial_enabled;
}

uint8 BlueSerial_IsControlActive(void)
{
    uint32 primask;
    uint8 active;

    primask = interrupt_global_disable();
    active = (g_blueserial_enabled &&
              (g_control_mode != BLUESERIAL_MODE_STOP || g_yaw_hold_enabled)) ? 1U : 0U;
    interrupt_global_enable(primask);
    return active;
}

static void BlueSerial_ToggleYawHold(void)
{
    uint32 primask;
    uint8 enabled;
    float hold_yaw_deg = 0.0f;

    primask = interrupt_global_disable();
    if (g_yaw_hold_enabled)
    {
        BlueSerial_StopControlLocked();
        enabled = 0U;
    }
    else
    {
        /* 原地航向保持：记录当前 yaw，让 path_follow 只输出角速度，不生成平移速度。 */
        hold_yaw_deg = eulerAngle.yaw;
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

    if (g_target_speed_cmps <= 0.0f)
    {
        return 0U;
    }

    primask = interrupt_global_disable();
    hold_yaw_deg = eulerAngle.yaw;
    g_cruise_move_direction = g_next_move_direction;
    g_turn_quarters_remaining = 0U;
    g_yaw_hold_enabled = 0U;
    path_follow_set_path(NULL, 0U);
    path_follow_set_stationary_yaw_hold_enabled(0U);
    BlueSerial_ZeroSpeedCommand();
    BlueSerial_ClearWheelPid();
    PID_Clear(&pid_yaw);
    path_follow_set_target_yaw(hold_yaw_deg);
    path_follow_set_stationary_yaw_hold_enabled(1U);
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
    return (distance == VISION_RECOGNITION_DISTANCE_TWO_GRID) ? 2U : 1U;
}

static void BlueSerial_PrintVisionResult(VisionRecognitionType type,
                                         const VisionRecognitionResult *result)
{
    if (result == NULL)
    {
        BlueSerial_Printf("VISION type=%s success=0 label=? score=NA mode=0\r\n",
                          BlueSerial_VisionTypeName(type));
        return;
    }

    if (result->score < 0)
    {
        BlueSerial_Printf("VISION type=%s success=%u label=%s score=NA mode=%u\r\n",
                          BlueSerial_VisionTypeName(type),
                          result->success ? 1U : 0U,
                          result->label,
                          result->mode_marker ? 1U : 0U);
    }
    else
    {
        BlueSerial_Printf("VISION type=%s success=%u label=%s score=%d mode=%u\r\n",
                          BlueSerial_VisionTypeName(type),
                          result->success ? 1U : 0U,
                          result->label,
                          (int)result->score,
                          result->mode_marker ? 1U : 0U);
    }
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
    uint8 move_x_first = (fabsf(delta_x_m) >= fabsf(delta_y_m)) ? 1U : 0U;
    int start_row;
    int start_col;
    int target_row;
    int target_col;
    size_t steps = 1U;

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

    if (move_x_first)
    {
        if (target_row != start_row)
        {
            g_blueserial_point_target_path[steps].row = (uint8)target_row;
            g_blueserial_point_target_path[steps].col = (uint8)start_col;
            g_blueserial_point_target_path[steps].id = 0U;
            steps++;
        }
        if (target_col != start_col)
        {
            g_blueserial_point_target_path[steps].row = (uint8)target_row;
            g_blueserial_point_target_path[steps].col = (uint8)target_col;
            g_blueserial_point_target_path[steps].id = 0U;
            steps++;
        }
    }
    else
    {
        if (target_col != start_col)
        {
            g_blueserial_point_target_path[steps].row = (uint8)start_row;
            g_blueserial_point_target_path[steps].col = (uint8)target_col;
            g_blueserial_point_target_path[steps].id = 0U;
            steps++;
        }
        if (target_row != start_row)
        {
            g_blueserial_point_target_path[steps].row = (uint8)target_row;
            g_blueserial_point_target_path[steps].col = (uint8)target_col;
            g_blueserial_point_target_path[steps].id = 0U;
            steps++;
        }
    }

    path_follow_set_path_with_grid(g_blueserial_point_target_path, steps, grid_m, 0U);
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
        path_follow_set_target_yaw(status.yaw_deg);
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

static void BlueSerial_StartCenteredRelativeMove(float delta_x_m, float delta_y_m, float yaw_deg)
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

    path_follow_reset_pose((float)center_cell * grid_m, (float)center_cell * grid_m, yaw_deg);
    path_follow_set_target_yaw(yaw_deg);
    path_follow_set_path_with_grid(g_blueserial_relative_path, 2U, grid_m, 0U);
}

static uint8 BlueSerial_StartConfiguredMove(void)
{
    uint32 primask;
    blueserial_move_direction_t direction;
    path_follow_status_t status = {0};
    float yaw_rad;
    float position_m;
    float delta_x_m = 0.0f;
    float delta_y_m = 0.0f;

    if (g_target_speed_cmps <= 0.0f || g_target_position_cm <= 0.0f)
    {
        return 0U;
    }

    primask = interrupt_global_disable();
    direction = g_next_move_direction;
    path_follow_get_status(&status);
    yaw_rad = status.yaw_deg * ((float)M_PI / 180.0f);
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
            break;

        case BLUESERIAL_MOVE_BACKWARD:
            delta_x_m = -cosf(yaw_rad) * position_m;
            delta_y_m = -sinf(yaw_rad) * position_m;
            break;

        case BLUESERIAL_MOVE_LEFT:
            delta_x_m = -sinf(yaw_rad) * position_m;
            delta_y_m = cosf(yaw_rad) * position_m;
            break;

        case BLUESERIAL_MOVE_RIGHT:
            delta_x_m = sinf(yaw_rad) * position_m;
            delta_y_m = -cosf(yaw_rad) * position_m;
            break;

        default:
            interrupt_global_enable(primask);
            return 0U;
    }

    g_turn_quarters_remaining = 0U;
    BlueSerial_ApplyConfiguredPathSpeedLocked();
    BlueSerial_EnterPathMode();
    BlueSerial_StartCenteredRelativeMove(delta_x_m, delta_y_m, status.yaw_deg);
    interrupt_global_enable(primask);
    return 1U;
}

static void BlueSerial_StartNextQuarterTurn(void)
{
    path_follow_status_t status = {0};

    path_follow_get_status(&status);
    path_follow_start_rotate_to_yaw(status.yaw_deg + g_turn_step_deg);
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
    BlueSerial_EnterPathMode();
    BlueSerial_StartNextQuarterTurn();
    interrupt_global_enable(primask);
    return 1U;
}

static void BlueSerial_ServiceMotionCompletion(void)
{
    uint32 primask;
    uint8 completed = 0U;
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
        BlueSerial_Printf("DONE\r\n");
    }
}

static void BlueSerial_EnterRawPwmMode(void)
{
    g_turn_quarters_remaining = 0U;
    g_yaw_hold_enabled = 0U;
    path_follow_set_path(NULL, 0U);
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
    uint32 primask = interrupt_global_disable();

    BlueSerial_ResetRxStateLocked();
    interrupt_global_enable(primask);
    uart_init(BLUESERIAL_UART, BLUESERIAL_BAUDRATE, BLUESERIAL_TX_PIN, BLUESERIAL_RX_PIN);
    g_blueserial_uart_initialized = 1U;
    interrupt_set_priority(BLUESERIAL_IRQN, BLUESERIAL_IRQ_PRIORITY);
    uart_rx_interrupt(BLUESERIAL_UART,
                      g_blueserial_enabled ? ZF_ENABLE : ZF_DISABLE);
}

void BlueSerial_SendByte(uint8 Byte)
{
    if (!g_blueserial_enabled)
    {
        return;
    }
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

static void BlueSerial_RecordRxByteForDisplay(uint8 ch)
{
    if (ch == '[')
    {
        g_last_rx_frame_len = 0U;
        g_last_rx_frame[0] = '\0';
        return;
    }

    if (ch == ']' || ch == '\r' || ch == '\n')
    {
        return;
    }

    if (ch < 32U || ch > 126U)
    {
        return;
    }

    if (g_last_rx_frame_len >= (BLUESERIAL_RX_FRAME_LEN - 1U))
    {
        return;
    }

    g_last_rx_frame[g_last_rx_frame_len++] = (char)ch;
    g_last_rx_frame[g_last_rx_frame_len] = '\0';
}

static void BlueSerial_ProcessRxByte(uint8 ch)
{
    uint8 next_write;
    uint8 i;

    BlueSerial_RecordRxByteForDisplay(ch);

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
        if (g_blueserial_enabled)
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
#if MOTOR_BOARD_REMAP_ORDER_2341
    static const char *const names[4] = {"m4", "m2", "m3", "m1"};
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

#if MOTOR_BOARD_REMAP_ORDER_2341
    if (strcmp(name, "pwm.m1") == 0)
    {
        return 3;
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
        return 0;
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

static uint8 BlueSerial_SetSlider(const char *name, float value)
{
    uint32 primask;
    int wheel_index;
    int pwm_value;
    float applied_value;

    wheel_index = BlueSerial_GetLogicalWheelIndex(name);
    if (wheel_index >= 0)
    {
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
    if (strcmp(name, "pos.kp") == 0 || strcmp(name, "position.kp") == 0)
    {
        applied_value = BlueSerial_ClampFloat(value, 0.0f, BLUESERIAL_MAX_POSITION_KP);
        primask = interrupt_global_disable();
        path_follow_set_position_pid_gains(applied_value, pid_stay.fKi, pid_stay.fKd);
        interrupt_global_enable(primask);
        BlueSerial_Printf("OK pos.kp=%.4f\r\n", applied_value);
        return 1U;
    }
    if (strcmp(name, "pos.ki") == 0 || strcmp(name, "position.ki") == 0)
    {
        applied_value = BlueSerial_ClampFloat(value, 0.0f, BLUESERIAL_MAX_POSITION_KI);
        primask = interrupt_global_disable();
        path_follow_set_position_pid_gains(pid_stay.fKp, applied_value, pid_stay.fKd);
        interrupt_global_enable(primask);
        BlueSerial_Printf("OK pos.ki=%.4f\r\n", applied_value);
        return 1U;
    }
    if (strcmp(name, "pos.kd") == 0 || strcmp(name, "position.kd") == 0)
    {
        applied_value = BlueSerial_ClampFloat(value, 0.0f, BLUESERIAL_MAX_POSITION_KD);
        primask = interrupt_global_disable();
        path_follow_set_position_pid_gains(pid_stay.fKp, pid_stay.fKi, applied_value);
        interrupt_global_enable(primask);
        BlueSerial_Printf("OK pos.kd=%.4f\r\n", applied_value);
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
    float cross_left;
    float cross_right;
    float position_kp;
    float position_ki;
    float position_kd;
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
    cross_left = path_y_crosstalk_left_x_comp_k;
    cross_right = path_y_crosstalk_right_x_comp_k;
    position_kp = pid_stay.fKp;
    position_ki = pid_stay.fKi;
    position_kd = pid_stay.fKd;
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
    BlueSerial_Printf("TUNE cross.left=%.4f cross.right=%.4f pospid=%.4f,%.4f,%.4f\r\n",
                      cross_left, cross_right,
                      position_kp, position_ki, position_kd);
    if (point_target_valid)
    {
        BlueSerial_Printf("TARGET pending=1 target_m=%.3f,%.3f\r\n",
                          point_target_x_m,
                          point_target_y_m);
    }
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
        BlueSerial_StartTurn(1U, 90.0f);
        BlueSerial_Printf("OK turn90 ccw\r\n");
        return 1U;
    }
    if (strcmp(command, "turn360") == 0)
    {
        BlueSerial_StartTurn(4U, 90.0f);
        BlueSerial_Printf("OK turn360 ccw\r\n");
        return 1U;
    }
    if (strcmp(command, "turn90r") == 0)
    {
        BlueSerial_StartTurn(1U, -90.0f);
        BlueSerial_Printf("OK turn90 cw\r\n");
        return 1U;
    }
    if (strcmp(command, "turn360r") == 0)
    {
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
    if (strcmp(command, "IMG2") == 0 || strcmp(command, "img2") == 0)
    {
        BlueSerial_StartVisionRequest(VISION_RECOGNITION_IMG,
                                      VISION_RECOGNITION_DISTANCE_TWO_GRID);
        return 1U;
    }
    if (strcmp(command, "NUM1") == 0 || strcmp(command, "num1") == 0)
    {
        BlueSerial_StartVisionRequest(VISION_RECOGNITION_NUM,
                                      VISION_RECOGNITION_DISTANCE_ONE_GRID);
        return 1U;
    }
    if (strcmp(command, "NUM2") == 0 || strcmp(command, "num2") == 0)
    {
        BlueSerial_StartVisionRequest(VISION_RECOGNITION_NUM,
                                      VISION_RECOGNITION_DISTANCE_TWO_GRID);
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

void BlueSerial_ControlTick10ms(void)
{
    if (!g_blueserial_enabled)
    {
        return;
    }

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
        speed_three_array[2] = yaw_hold.omega_cmd;
        Kinematics_Inverse(speed_three_array, speed_encoder);
        motor_control(speed_encoder);
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
    if (!g_blueserial_enabled)
    {
        return;
    }

    BlueSerial_CommandTask();
    BlueSerial_ServiceMotionCompletion();
    BlueSerial_ServicePositionRequest();
    BlueSerial_ServiceVisionRequest();
}
