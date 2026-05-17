/*********************************************************************************************************************
 * RT1064DVL6A Opensourec Library 即（RT1064DVL6A 开源库）是一个基于官方 SDK 接口的第三方开源库
 * Copyright (c) 2022 SEEKFREE 逐飞科技
 *
 * 本文件是 RT1064DVL6A 开源库的一部分
 *
 * RT1064DVL6A 开源库 是免费软件
 * 您可以根据自由软件基金会发布的 GPL（GNU General Public License，即 GNU通用公共许可证）的条款
 * 即 GPL 的第3版（即 GPL3.0）或（您选择的）任何后来的版本，重新发布和/或修改它
 *
 * 本开源库的发布是希望它能发挥作用，但并未对其作任何的保证
 * 甚至没有隐含的适销性或适合特定用途的保证
 * 更多细节请参见 GPL
 *
 * 您应该在收到本开源库的同时收到一份 GPL 的副本
 * 如果没有，请参阅<https://www.gnu.org/licenses/>
 *
 * 额外注明：
 * 本开源库使用 GPL3.0 开源许可证协议 以上许可申明为译文版本
 * 许可申明英文版在 libraries/doc 文件夹下的 GPL3_permission_statement.txt 文件中
 * 许可证副本在 libraries 文件夹下 即该文件夹下的 LICENSE 文件
 * 欢迎各位使用并传播本程序 但修改内容时必须保留逐飞科技的版权声明（即本声明）
 *
 * 文件名称          main
 * 公司名称          成都逐飞科技有限公司
 * 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
 * 开发环境          IAR 8.32.4 or MDK 5.33
 * 适用平台          RT1064DVL6A
 * 店铺链接          https://seekfree.taobao.com/
 *
 * 修改记录
 * 日期              作者                备注
 * 2022-09-21        SeekFree            first version
 ********************************************************************************************************************/

#include "zf_common_headfile.h"
#include "Flash.h"
#include "Mymenu.h"
#include "Attitude.h"
#include "Game_logic.h"
#include "data_handle.h"
#include "path_follow.h"
#include "Motor.h"
#include "PID.h"
#include "PID_config.h"
#include "Control.h"
#include "wifi.h"
#include <stdio.h>
#include <string.h>

#define PIT_PRIORITY_0 (PIT_IRQn)
#define PIT_PRIORITY_1 (PIT_IRQn)
#define PIT_PRIORITY_2 (PIT_IRQn)

#define ALGORITHM_TEST_ENABLE 0     // 1: 启用算法测试，使用预设地图数据进行路径规划测试；0: 关闭算法测试，正常运行控制流程
#define MOTOR_ENCODER_MAP_TEST_ENABLE 1

#if MOTOR_ENCODER_MAP_TEST_ENABLE

#define MAP_TEST_PWM_DUTY      1200
#define MAP_TEST_PULSE_MS      300U
#define MAP_TEST_SETTLE_MS     80U
#define MAP_TEST_REFRESH_MS    20U
#define MAP_TEST_DEADBAND      2

typedef struct
{
  int16 raw[4];
  int16 feedback[4];
  uint8 max_index;
  uint8 valid;
  int16 max_raw;
  int16 max_feedback;
} motor_map_test_result_t;

static uint8 s_test_motor = 0U;
static int s_test_dir = 1;
static motor_map_test_result_t s_last_result = {{0}, {0}, 0U, 0U, 0, 0};

static const char *const s_motor_pin_desc[4] =
{
  "PH4 C7 / EN4 C6",
  "PH3 C9 / EN3 C8",
  "PH2 C11 / EN2 C10",
  "PH1 D3 / EN1 D2",
};

static int16 motor_map_abs16(int16 value)
{
  return (value < 0) ? (int16)(-value) : value;
}

static void motor_map_clear_encoders(void)
{
  encoder_clear_count(ENCODER_1);
  encoder_clear_count(ENCODER_2);
  encoder_clear_count(ENCODER_3);
  encoder_clear_count(ENCODER_4);
}

static void motor_map_read_raw(int16 out_raw[4])
{
  out_raw[0] = encoder_get_count(ENCODER_1);
  out_raw[1] = encoder_get_count(ENCODER_2);
  out_raw[2] = encoder_get_count(ENCODER_3);
  out_raw[3] = encoder_get_count(ENCODER_4);
}

static void motor_map_raw_to_feedback(const int16 raw[4], int16 feedback[4])
{
  feedback[0] = (int16)(-raw[0]);
  feedback[1] = raw[1];
  feedback[2] = (int16)(-raw[2]);
  feedback[3] = raw[3];
}

static void motor_map_drive_selected(uint8 motor_index, int speed)
{
  int pwm[4] = {0, 0, 0, 0};

  if (motor_index < 4U)
  {
    pwm[motor_index] = speed;
  }
  motor_pwm(pwm[0], pwm[1], pwm[2], pwm[3]);
}

static void motor_map_run_pulse(void)
{
  uint8 i = 0U;
  int16 max_abs = 0;

  memset(&s_last_result, 0, sizeof(s_last_result));
  motor_pwm(0, 0, 0, 0);
  motor_map_clear_encoders();
  system_delay_ms(MAP_TEST_SETTLE_MS);

  motor_map_drive_selected(s_test_motor, s_test_dir * MAP_TEST_PWM_DUTY);
  system_delay_ms(MAP_TEST_PULSE_MS);
  motor_pwm(0, 0, 0, 0);
  system_delay_ms(MAP_TEST_SETTLE_MS);

  motor_map_read_raw(s_last_result.raw);
  motor_map_raw_to_feedback(s_last_result.raw, s_last_result.feedback);
  motor_map_clear_encoders();

  for (i = 0U; i < 4U; i++)
  {
    int16 current_abs = motor_map_abs16(s_last_result.raw[i]);
    if (current_abs > max_abs)
    {
      max_abs = current_abs;
      s_last_result.max_index = i;
    }
  }

  if (max_abs >= MAP_TEST_DEADBAND)
  {
    s_last_result.valid = 1U;
    s_last_result.max_raw = s_last_result.raw[s_last_result.max_index];
    s_last_result.max_feedback = s_last_result.feedback[s_last_result.max_index];
  }
}

static void motor_map_show_signed(uint16 x, uint16 y, int16 value)
{
  ips200_show_string(x, y, "      ");
  ips200_show_int(x, y, value, 5);
}

static void motor_map_draw_screen(void)
{
  char line[32];

  ips200_clear();
  ips200_show_string(0, 0, "MOTOR ENCODER MAP TEST");
  ips200_show_string(0, 16, "K1/K4:CH K2:DIR K3:RUN");

  snprintf(line, sizeof(line), "SW:M%u DIR:%c PWM:%d",
           (unsigned int)(s_test_motor + 1U),
           (s_test_dir > 0) ? '+' : '-',
           MAP_TEST_PWM_DUTY);
  ips200_show_string(0, 32, line);

  ips200_show_string(0, 48, "PIN:");
  ips200_show_string(40, 48, s_motor_pin_desc[s_test_motor]);

  snprintf(line, sizeof(line), "PULSE:%ums", (unsigned int)MAP_TEST_PULSE_MS);
  ips200_show_string(0, 64, line);

  ips200_show_string(0, 88, "RAW E1    E2    E3    E4");
  motor_map_show_signed(0, 104, s_last_result.raw[0]);
  motor_map_show_signed(56, 104, s_last_result.raw[1]);
  motor_map_show_signed(112, 104, s_last_result.raw[2]);
  motor_map_show_signed(168, 104, s_last_result.raw[3]);

  ips200_show_string(0, 128, "FB  F1    F2    F3    F4");
  motor_map_show_signed(0, 144, s_last_result.feedback[0]);
  motor_map_show_signed(56, 144, s_last_result.feedback[1]);
  motor_map_show_signed(112, 144, s_last_result.feedback[2]);
  motor_map_show_signed(168, 144, s_last_result.feedback[3]);

  if (s_last_result.valid)
  {
    snprintf(line, sizeof(line), "MAX:E%u RAW:%d",
             (unsigned int)(s_last_result.max_index + 1U),
             (int)s_last_result.max_raw);
    ips200_show_string(0, 176, line);
    snprintf(line, sizeof(line), "SOFT FB:%d", (int)s_last_result.max_feedback);
    ips200_show_string(0, 192, line);
  }
  else
  {
    ips200_show_string(0, 176, "MAX: none/run K3");
  }

  ips200_show_string(0, 224, "Rule: SW Mx -> MAX Ex");
  ips200_show_string(0, 240, "FB sign should match DIR");
  ips200_show_string(0, 256, "Test with wheels lifted.");
}

static void motor_map_handle_keys(void)
{
  key_state_enum k1;
  key_state_enum k2;
  key_state_enum k3;
  key_state_enum k4;
  uint8 redraw = 0U;

  key_scanner();
  k1 = key_get_state(KEY_1);
  k2 = key_get_state(KEY_2);
  k3 = key_get_state(KEY_3);
  k4 = key_get_state(KEY_4);

  if (k1 == KEY_SHORT_PRESS)
  {
    s_test_motor = (uint8)((s_test_motor + 3U) % 4U);
    redraw = 1U;
  }
  if (k4 == KEY_SHORT_PRESS)
  {
    s_test_motor = (uint8)((s_test_motor + 1U) % 4U);
    redraw = 1U;
  }
  if (k2 == KEY_SHORT_PRESS)
  {
    s_test_dir = -s_test_dir;
    redraw = 1U;
  }
  if (k3 == KEY_SHORT_PRESS)
  {
    motor_map_run_pulse();
    redraw = 1U;
  }

  key_clear_state(KEY_1);
  key_clear_state(KEY_2);
  key_clear_state(KEY_3);
  key_clear_state(KEY_4);

  if (redraw)
  {
    motor_map_draw_screen();
  }
}

int main(void)
{
  clock_init(SYSTEM_CLOCK_600M);
  debug_init();
  system_delay_ms(300);

  ips200_set_dir(IPS200_PORTAIT);
  ips200_set_font(IPS200_8X16_FONT);
  ips200_set_color(RGB565_WHITE, RGB565_BLACK);
  ips200_init(IPS200_TYPE_SPI);
  key_init(20);

  motor_init();
  encoder_init();
  motor_pwm(0, 0, 0, 0);
  motor_map_clear_encoders();
  motor_map_draw_screen();

  while (1)
  {
    motor_map_handle_keys();
    system_delay_ms(MAP_TEST_REFRESH_MS);
  }
}

void pit_0_handler(void)
{
}

void pit_1_handler(void)
{
}

void pit_2_handler(void)
{
}

#else

int main(void)
{
  clock_init(SYSTEM_CLOCK_600M); // 不可删除
  debug_init();                  // 调试端口初始化
  // 模块初始化
  system_delay_ms(300); // 等待主板其他外设上电完成
  Menu_Init();          // 菜单系统初始化(包含按键，显示屏等相关初始化)
  uart_blob_init();     // 摄像头串口接收与解析模块初始化
  flash_init();         // Flash模块初始化
  motor_init();         // 电机控制模块初始化
  encoder_init();       // 电机编码器模块初始化
  imu963ra_init();      // IMU模块初始化
  Attitude_Init();      // 姿态解算模块初始化
  wifi_init("HDUASC_saidao", "zyz520520", NULL);          // Wi-Fi模块初始化

  PID_Init(&ULpid, &ULPidInitStruct);
  PID_Init(&URpid, &URPidInitStruct);
  PID_Init(&DLpid, &DLPidInitStruct);
  PID_Init(&DRpid, &DRPidInitStruct);
  PID_Init(&Yawpid, &YawPidInitStruct);
  PID_Init(&Camera_x_pid, &Camera_x_PidInitStruct);
  PID_Init(&Camera_y_pid, &Camera_y_PidInitStruct);
  PID_Init(&Gyro_rotate_pid, &Gyro_Rotate_PidInitStruct);
  Kinematics_Init();
  path_follow_init(GRID_SIZE_M, (float)pulse_per_meter);

  //中断初始化
  pit_ms_init(PIT_CH0, 20);                  // PIT0 periodic interrupt: 20 ms.
  interrupt_set_priority(PIT_PRIORITY_0, 2); // PIT0 interrupt priority.
  pit_ms_init(PIT_CH1, 10);                  // PIT1 periodic interrupt: 10 ms.
  interrupt_set_priority(PIT_PRIORITY_1, 1); // PIT1 interrupt priority.
  pit_ms_init(PIT_CH2, 2);                   // PIT2 periodic interrupt: 2 ms.
  interrupt_set_priority(PIT_PRIORITY_2, 0); // PIT2 interrupt priority.
  interrupt_set_priority(LPUART1_IRQn, 3);   // UART1 interrupt priority.
  interrupt_set_priority(LPUART4_IRQn, 4);   // UART4 interrupt priority.
  interrupt_global_enable(0);

  control_init();

  //算法测试
#if ALGORITHM_TEST_ENABLE
  parse_map_from_string(map_text3);

  boxes[0].id = 0;
  boxes[1].id = 1;
  boxes[2].id = 2;
  targets[0].id = 0;
  targets[1].id = 1;
  targets[2].id = 2;
	
  Test_Data_Load();  // 数据加载到内部测试地图
  Plan_path_Identify(); // 路径规划

  Test_Path_Init(); // 路径初始化
#endif

  // 主循环
  while (1)
  {
    //算法测试
  #if ALGORITHM_TEST_ENABLE
    Test_Data_Save(); // 将内部测试地图数据保存回全局数组
  #endif

    // 控制主流程：初始定位 -> 摄像头数据到齐 -> 路径规划 -> 路径执行 -> 执行中动态校正
    control_process();

    // 菜单系统主循环调用：响应按键事件，更新显示等。
    Menu_Switch();
    Menu_Show();
    
  }
}

/* PIT0 周期中断（20ms）：负责按键扫描。 */
void pit_0_handler(void)
{
  key_scanner();
}

void pit_1_handler(void)
{
  // 采样编码器
  encoder_get();

  if (control_is_path_plan_paused())
  {
    // 规划保护期内，清零速度目标，避免执行层继续输出旧命令
    memset(speed_three_array, 0, sizeof(speed_three_array));
    memset(speed_encoder, 0, sizeof(speed_encoder));
  }
  else
  {
    // 正常状态下按路径/姿态策略更新目标速度
    distance_speed_strategy();
  }

  // 电机输出控制：按运行/停车标志选择控制分支
  if (car_go_flag == 1 && car_stop_flag == 0)
  {
    if (wait_stop == 1) // 兼容历史“等待完全停稳”模式。
    {
      motor_control(car_stop_array);
      // 四轮都接近静止后清空 PID 累积并断 PWM
      if (abs(up_L_all) < 5 && abs(up_R_all) < 5 && abs(down_L_all) < 5 && abs(down_R_all) < 5)
      {
        PID_Clear(&ULpid);
        PID_Clear(&URpid);
        PID_Clear(&DLpid);
        PID_Clear(&DRpid);
        motor_pwm(0, 0, 0, 0);
      }
    }
    else
    {
      motor_control(speed_encoder);
    }
  }
  else if (car_go_flag == 1 && car_stop_flag == 1)
  {
    // 运行态但要求急停，统一下发 0 速度
    motor_control(car_stop_array);
    if (abs(up_L_all) < 5 && abs(up_R_all) < 5 && abs(down_L_all) < 5 && abs(down_R_all) < 5)
    {
      PID_Clear(&ULpid);
      PID_Clear(&URpid);
      PID_Clear(&DLpid);
      PID_Clear(&DRpid);
      motor_pwm(0, 0, 0, 0);
    }
  }
  else
  {
    motor_pwm(0, 0, 0, 0);
  }

}

/* PIT2 周期中断（2ms）：姿态解算 */
void pit_2_handler(void)
{
  /* 规划保护期内不更新姿态，避免重规划期间姿态状态与路径状态不同步。 */
  if (control_is_path_plan_paused())
    return;
  Attitude_Calculate();
}

#endif
