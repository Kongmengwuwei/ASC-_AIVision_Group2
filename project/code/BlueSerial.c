/*
 * UART4 Bluetooth tuning module.
 *
 * Compatible frame format (same style as the old car project):
 *   [slider,name,value]
 *   [button_name]
 *
 * Parsing and control changes run in the main loop. The UART ISR only queues
 * complete frames, so sscanf/printf/path control never run in interrupt context.
 */

#include "zf_common_headfile.h"
#include "zf_driver_uart.h"
#include "zf_driver_gpio.h"
#include "zf_driver_pwm.h"
#include "path_follow.h"
#include "Motor.h"
#include "PID_config.h"
#include "Mymenu.h"
#include "Attitude.h"
#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.1415926f
#endif

#define BLUESERIAL_UART                         UART_4
#define BLUESERIAL_BAUDRATE                     115200U
#define BLUESERIAL_PATH_REPORT_PERIOD_TICKS     5U
#define BLUESERIAL_RX_FRAME_LEN                 80U
#define BLUESERIAL_RX_QUEUE_LEN                 4U
#define BLUESERIAL_PWM_LIMIT                    6000
#define BLUESERIAL_MAX_TARGET_SPEED_CMPS        150.0f
#define BLUESERIAL_MAX_TARGET_DISTANCE_CM       500.0f
#define BLUESERIAL_MAX_YAW_KP                   20.0f
#define BLUESERIAL_MAX_YAW_KI                   10.0f
#define BLUESERIAL_MAX_YAW_KD                   50.0f
#define BLUESERIAL_MAX_YAW_FF_DEGPS             120.0f

typedef enum
{
    BLUESERIAL_MODE_STOP = 0,
    BLUESERIAL_MODE_RAW_PWM,
    BLUESERIAL_MODE_PATH_MOTION
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

static volatile uint8 g_blueserial_path_report_pending = 0U;
static blueserial_control_mode_t g_control_mode = BLUESERIAL_MODE_STOP;
static int g_raw_pwm[4] = {0, 0, 0, 0};
static float g_target_speed_cmps = 40.0f;
static float g_target_distance_cm = 50.0f;
static uint8 g_turn_quarters_remaining = 0U;
static float g_turn_step_deg = 90.0f;
static uint8 g_yaw_hold_enabled = 0U;

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

/*
 * Exact raw PWM output for wheel-by-wheel testing.
 * Do not call motor_pwm() here: that function intentionally adds 400 duty to
 * the upper-right wheel, while a raw slider must map 1:1 to actual PWM duty.
 */
static void BlueSerial_ApplyRawPwm(void)
{
    int duty;

    duty = (g_raw_pwm[0] >= 0) ? g_raw_pwm[0] : -g_raw_pwm[0];
    gpio_set_level(MOTOR1_DIR, (g_raw_pwm[0] < 0) ? GPIO_HIGH : GPIO_LOW);
    pwm_set_duty(MOTOR1_PWM, (uint32)duty);

    duty = (g_raw_pwm[1] >= 0) ? g_raw_pwm[1] : -g_raw_pwm[1];
    gpio_set_level(MOTOR2_DIR, (g_raw_pwm[1] < 0) ? GPIO_LOW : GPIO_HIGH);
    pwm_set_duty(MOTOR2_PWM, (uint32)duty);

    duty = (g_raw_pwm[2] >= 0) ? g_raw_pwm[2] : -g_raw_pwm[2];
    gpio_set_level(MOTOR3_DIR, (g_raw_pwm[2] < 0) ? GPIO_HIGH : GPIO_LOW);
    pwm_set_duty(MOTOR3_PWM, (uint32)duty);

    duty = (g_raw_pwm[3] >= 0) ? g_raw_pwm[3] : -g_raw_pwm[3];
    gpio_set_level(MOTOR4_DIR, (g_raw_pwm[3] < 0) ? GPIO_LOW : GPIO_HIGH);
    pwm_set_duty(MOTOR4_PWM, (uint32)duty);
}

static void BlueSerial_StopControl(void)
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
    motor_pwm(0, 0, 0, 0);
}

static void BlueSerial_ToggleYawHold(void)
{
    float hold_yaw_deg;

    if (g_yaw_hold_enabled)
    {
        g_yaw_hold_enabled = 0U;
        g_control_mode = BLUESERIAL_MODE_STOP;
        g_turn_quarters_remaining = 0U;
        path_follow_set_path(NULL, 0U);
        path_follow_set_stationary_yaw_hold_enabled(0U);
        car_go_flag = 0U;
        car_stop_flag = 0U;
        BlueSerial_ZeroSpeedCommand();
        BlueSerial_ClearWheelPid();
        PID_Clear(&pid_yaw);
        motor_pwm(0, 0, 0, 0);
        BlueSerial_Printf("OK yawhold=off\r\n");
        return;
    }

    /* Capture the yaw at the instant the button is pressed. */
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
    BlueSerial_Printf("OK yawhold=on target=%.2fdeg\r\n", hold_yaw_deg);
}

static void BlueSerial_ApplyConfiguredPathSpeed(void)
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
    g_control_mode = BLUESERIAL_MODE_PATH_MOTION;
    car_go_flag = 1U;
    car_stop_flag = 0U;
    BlueSerial_ZeroSpeedCommand();
    BlueSerial_ClearWheelPid();
}

static uint8 BlueSerial_StartLinearMove(blueserial_move_direction_t direction)
{
    path_follow_status_t status = {0};
    float yaw_rad;
    float distance_m;
    float delta_x_m = 0.0f;
    float delta_y_m = 0.0f;

    if (g_target_speed_cmps <= 0.0f || g_target_distance_cm <= 0.0f)
    {
        return 0U;
    }

    path_follow_get_status(&status);
    yaw_rad = status.yaw_deg * ((float)M_PI / 180.0f);
    distance_m = g_target_distance_cm * 0.01f;

    switch (direction)
    {
        case BLUESERIAL_MOVE_FORWARD:
            delta_x_m = cosf(yaw_rad) * distance_m;
            delta_y_m = sinf(yaw_rad) * distance_m;
            break;

        case BLUESERIAL_MOVE_BACKWARD:
            delta_x_m = -cosf(yaw_rad) * distance_m;
            delta_y_m = -sinf(yaw_rad) * distance_m;
            break;

        case BLUESERIAL_MOVE_LEFT:
            delta_x_m = -sinf(yaw_rad) * distance_m;
            delta_y_m = cosf(yaw_rad) * distance_m;
            break;

        case BLUESERIAL_MOVE_RIGHT:
            delta_x_m = sinf(yaw_rad) * distance_m;
            delta_y_m = -cosf(yaw_rad) * distance_m;
            break;

        default:
            return 0U;
    }

    g_turn_quarters_remaining = 0U;
    BlueSerial_ApplyConfiguredPathSpeed();
    BlueSerial_EnterPathMode();
    path_follow_set_target_yaw(status.yaw_deg);
    path_follow_start_offset_move(delta_x_m, delta_y_m);
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
    if (quarter_count == 0U)
    {
        return 0U;
    }

    g_turn_quarters_remaining = quarter_count;
    g_turn_step_deg = step_deg;
    BlueSerial_EnterPathMode();
    BlueSerial_StartNextQuarterTurn();
    return 1U;
}

static void BlueSerial_ServiceMotionCompletion(void)
{
    path_follow_status_t status = {0};

    if (g_control_mode != BLUESERIAL_MODE_PATH_MOTION)
    {
        return;
    }

    path_follow_get_status(&status);
    if (status.active)
    {
        return;
    }

    if (g_turn_quarters_remaining > 0U)
    {
        g_turn_quarters_remaining--;
        if (g_turn_quarters_remaining > 0U)
        {
            BlueSerial_StartNextQuarterTurn();
            return;
        }
    }

    g_control_mode = BLUESERIAL_MODE_STOP;
    car_go_flag = g_yaw_hold_enabled ? 1U : 0U;
    car_stop_flag = 0U;
    BlueSerial_ZeroSpeedCommand();
    BlueSerial_ClearWheelPid();
    motor_pwm(0, 0, 0, 0);
    BlueSerial_Printf("DONE\r\n");
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
    g_rx_collecting = 0U;
    g_rx_work_len = 0U;
    g_rx_queue_write = 0U;
    g_rx_queue_read = 0U;
    g_rx_drop_count = 0U;
    uart_init(BLUESERIAL_UART, BLUESERIAL_BAUDRATE, UART4_TX_C16, UART4_RX_C17);
    interrupt_set_priority(LPUART4_IRQn, 3);
    uart_rx_interrupt(BLUESERIAL_UART, ZF_ENABLE);
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

void BlueSerial_RxIrqHandler(void)
{
    uint8 ch = 0U;
    uint8 next_write;
    uint8 i;

    if (!uart_query_byte(BLUESERIAL_UART, &ch))
    {
        return;
    }

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

static uint8 BlueSerial_SetSlider(const char *name, float value)
{
    int pwm_value;

    if (strcmp(name, "pwm.ul") == 0)
    {
        pwm_value = BlueSerial_ClampInt((int)lroundf(value), -BLUESERIAL_PWM_LIMIT, BLUESERIAL_PWM_LIMIT);
        g_raw_pwm[0] = pwm_value;
        BlueSerial_EnterRawPwmMode();
        BlueSerial_Printf("OK pwm.ul=%d\r\n", pwm_value);
        return 1U;
    }
    if (strcmp(name, "pwm.ur") == 0)
    {
        pwm_value = BlueSerial_ClampInt((int)lroundf(value), -BLUESERIAL_PWM_LIMIT, BLUESERIAL_PWM_LIMIT);
        g_raw_pwm[1] = pwm_value;
        BlueSerial_EnterRawPwmMode();
        BlueSerial_Printf("OK pwm.ur=%d\r\n", pwm_value);
        return 1U;
    }
    if (strcmp(name, "pwm.dl") == 0)
    {
        pwm_value = BlueSerial_ClampInt((int)lroundf(value), -BLUESERIAL_PWM_LIMIT, BLUESERIAL_PWM_LIMIT);
        g_raw_pwm[2] = pwm_value;
        BlueSerial_EnterRawPwmMode();
        BlueSerial_Printf("OK pwm.dl=%d\r\n", pwm_value);
        return 1U;
    }
    if (strcmp(name, "pwm.dr") == 0)
    {
        pwm_value = BlueSerial_ClampInt((int)lroundf(value), -BLUESERIAL_PWM_LIMIT, BLUESERIAL_PWM_LIMIT);
        g_raw_pwm[3] = pwm_value;
        BlueSerial_EnterRawPwmMode();
        BlueSerial_Printf("OK pwm.dr=%d\r\n", pwm_value);
        return 1U;
    }
    if (strcmp(name, "speed") == 0)
    {
        g_target_speed_cmps = BlueSerial_ClampFloat(value, 0.0f, BLUESERIAL_MAX_TARGET_SPEED_CMPS);
        if (g_target_speed_cmps > 0.0f)
        {
            BlueSerial_ApplyConfiguredPathSpeed();
        }
        else if (g_control_mode == BLUESERIAL_MODE_PATH_MOTION)
        {
            BlueSerial_StopControl();
        }
        BlueSerial_Printf("OK speed=%.1fcmps\r\n", g_target_speed_cmps);
        return 1U;
    }
    if (strcmp(name, "distance") == 0)
    {
        g_target_distance_cm = BlueSerial_ClampFloat(value, 0.0f, BLUESERIAL_MAX_TARGET_DISTANCE_CM);
        BlueSerial_Printf("OK distance=%.1fcm\r\n", g_target_distance_cm);
        return 1U;
    }
    if (strcmp(name, "yaw.kp") == 0)
    {
        pid_yaw.fKp = BlueSerial_ClampFloat(value, 0.0f, BLUESERIAL_MAX_YAW_KP);
        PID_Clear(&pid_yaw);
        BlueSerial_Printf("OK yaw.kp=%.4f\r\n", pid_yaw.fKp);
        return 1U;
    }
    if (strcmp(name, "yaw.ki") == 0)
    {
        pid_yaw.fKi = BlueSerial_ClampFloat(value, 0.0f, BLUESERIAL_MAX_YAW_KI);
        PID_Clear(&pid_yaw);
        BlueSerial_Printf("OK yaw.ki=%.4f\r\n", pid_yaw.fKi);
        return 1U;
    }
    if (strcmp(name, "yaw.kd") == 0)
    {
        pid_yaw.fKd = BlueSerial_ClampFloat(value, 0.0f, BLUESERIAL_MAX_YAW_KD);
        PID_Clear(&pid_yaw);
        BlueSerial_Printf("OK yaw.kd=%.4f\r\n", pid_yaw.fKd);
        return 1U;
    }
    if (strcmp(name, "yaw.ff") == 0)
    {
        path_yaw_feedforward_min_degps =
            BlueSerial_ClampFloat(value, 0.0f, BLUESERIAL_MAX_YAW_FF_DEGPS);
        BlueSerial_Printf("OK yaw.ff=%.2fdegps\r\n", path_yaw_feedforward_min_degps);
        return 1U;
    }

    return 0U;
}

static uint8 BlueSerial_RunButton(const char *command)
{
    if (strcmp(command, "forward") == 0 || strcmp(command, "go ahead") == 0)
    {
        if (!BlueSerial_StartLinearMove(BLUESERIAL_MOVE_FORWARD))
        {
            BlueSerial_Printf("ERR set_speed_and_distance_first\r\n");
            return 1U;
        }
        BlueSerial_Printf("OK forward speed=%.1fcmps distance=%.1fcm\r\n",
                          g_target_speed_cmps, g_target_distance_cm);
        return 1U;
    }
    if (strcmp(command, "backward") == 0 || strcmp(command, "go back") == 0)
    {
        if (!BlueSerial_StartLinearMove(BLUESERIAL_MOVE_BACKWARD))
        {
            BlueSerial_Printf("ERR set_speed_and_distance_first\r\n");
            return 1U;
        }
        BlueSerial_Printf("OK backward speed=%.1fcmps distance=%.1fcm\r\n",
                          g_target_speed_cmps, g_target_distance_cm);
        return 1U;
    }
    if (strcmp(command, "left") == 0)
    {
        if (!BlueSerial_StartLinearMove(BLUESERIAL_MOVE_LEFT))
        {
            BlueSerial_Printf("ERR set_speed_and_distance_first\r\n");
            return 1U;
        }
        BlueSerial_Printf("OK left speed=%.1fcmps distance=%.1fcm\r\n",
                          g_target_speed_cmps, g_target_distance_cm);
        return 1U;
    }
    if (strcmp(command, "right") == 0)
    {
        if (!BlueSerial_StartLinearMove(BLUESERIAL_MOVE_RIGHT))
        {
            BlueSerial_Printf("ERR set_speed_and_distance_first\r\n");
            return 1U;
        }
        BlueSerial_Printf("OK right speed=%.1fcmps distance=%.1fcm\r\n",
                          g_target_speed_cmps, g_target_distance_cm);
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
        BlueSerial_Printf("STATUS mode=%u yawhold=%u speed=%.1fcmps distance=%.1fcm pwm=%d,%d,%d,%d "
                          "yawpid=%.4f,%.4f,%.4f ff=%.2f drop=%lu\r\n",
                          (unsigned int)g_control_mode,
                          (unsigned int)g_yaw_hold_enabled,
                          g_target_speed_cmps,
                          g_target_distance_cm,
                          g_raw_pwm[0], g_raw_pwm[1], g_raw_pwm[2], g_raw_pwm[3],
                          pid_yaw.fKp, pid_yaw.fKi, pid_yaw.fKd,
                          path_yaw_feedforward_min_degps,
                          (unsigned long)g_rx_drop_count);
        return 1U;
    }

    return 0U;
}

static void BlueSerial_ParseCommand(char *raw_command)
{
    char type[16];
    char name[32];
    float value;
    char *command = BlueSerial_TrimCommand(raw_command);

    if (sscanf(command, "%15[^,],%31[^,],%f", type, name, &value) == 3)
    {
        if (strcmp(type, "slider") == 0 && BlueSerial_SetSlider(name, value))
        {
            return;
        }
        BlueSerial_Printf("ERR unknown_slider %s\r\n", name);
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

void BlueSerial_CommandTask(void)
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
    if (g_control_mode == BLUESERIAL_MODE_RAW_PWM)
    {
        path_follow_output_t unused_output = {0};
        path_follow_update(eulerAngle.yaw, &unused_output);
        BlueSerial_ApplyRawPwm();
        return;
    }

    if (g_control_mode == BLUESERIAL_MODE_PATH_MOTION)
    {
        distance_speed_strategy();
        motor_control(speed_encoder);
        BlueSerial_ServiceMotionCompletion();
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

void BlueSerial_PathDebugTick10ms(void)
{
    static uint8 tick_div = 0U;

    if (++tick_div >= BLUESERIAL_PATH_REPORT_PERIOD_TICKS)
    {
        tick_div = 0U;
        g_blueserial_path_report_pending = 1U;
    }
}

static void BlueSerial_GetActualBodySpeed(float *vx_cmps, float *vy_cmps, float *omega_radps)
{
    float count_to_mps;
    float w_ul;
    float w_ur;
    float w_dl;
    float w_dr;

    if (vx_cmps == NULL || vy_cmps == NULL || omega_radps == NULL)
    {
        return;
    }

    *vx_cmps = 0.0f;
    *vy_cmps = 0.0f;
    *omega_radps = 0.0f;

    if (pulse_per_meter <= 0.0)
    {
        return;
    }

    count_to_mps = ((float)PID_RATE) / (float)pulse_per_meter;
    w_ul = (float)up_L_all * count_to_mps;
    w_ur = (float)up_R_all * count_to_mps;
    w_dl = (float)down_L_all * count_to_mps;
    w_dr = (float)down_R_all * count_to_mps;

    *vx_cmps = 0.25f * (w_ul + w_ur + w_dl + w_dr) * 100.0f;
    *vy_cmps = 0.25f * (-w_ul + w_ur + w_dl - w_dr) * 100.0f;
    *omega_radps = (-w_ul + w_ur - w_dl + w_dr) / (2.0f * D_X + 2.0f * D_Y);
}

void BlueSerial_PathDebugReport(void)
{
    path_follow_status_t status = {0};
    float actual_vx_cmps = 0.0f;
    float actual_vy_cmps = 0.0f;
    float actual_omega_radps = 0.0f;
    float actual_speed_cmps;

    /* Keep the existing main.c usable without adding a new command-task call. */
    BlueSerial_CommandTask();

    if (g_control_mode == BLUESERIAL_MODE_RAW_PWM)
    {
        BlueSerial_ApplyRawPwm();
    }
    else if (g_control_mode == BLUESERIAL_MODE_PATH_MOTION)
    {
        car_go_flag = 1U;
        car_stop_flag = 0U;
        BlueSerial_ServiceMotionCompletion();
    }
    else if (g_yaw_hold_enabled)
    {
        car_go_flag = 1U;
        car_stop_flag = 0U;
    }

    if (g_blueserial_path_report_pending == 0U)
    {
        return;
    }
    g_blueserial_path_report_pending = 0U;

    path_follow_get_status(&status);
    BlueSerial_GetActualBodySpeed(&actual_vx_cmps, &actual_vy_cmps, &actual_omega_radps);
    actual_speed_cmps = sqrtf(actual_vx_cmps * actual_vx_cmps +
                              actual_vy_cmps * actual_vy_cmps);

    /*
     * 50 ms telemetry, units:
     * speed cm/s, position m, yaw degree.
     */
    BlueSerial_Printf("TSPD %.1f ASPD %.1f TPOS %.3f %.3f APOS %.3f %.3f "
                      "TYAW %.2f AYAW %.2f\r\n",
                      status.speed_ref_cmps,
                      actual_speed_cmps,
                      status.target_x_m,
                      status.target_y_m,
                      status.x_m,
                      status.y_m,
                      status.target_yaw_deg,
                      status.yaw_deg);
}
