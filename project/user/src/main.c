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
#include <stdio.h>
#include <string.h>

/* RT1064 的四个 PIT 通道共用同一个 PIT_IRQn，优先级只能统一设置一次。 */
#define PIT_SHARED_IRQ_PRIORITY 2U

int main(void)
{
  clock_init(SYSTEM_CLOCK_600M); // 不可删除
  // 模块初始化
  system_delay_ms(300);                          // 等待主板其他外设上电完成
  flash_init();
  Menu_Init();
  uart_blob_init();
  motor_init();
  encoder_init();
  imu963ra_init();
  Attitude_Init();
  Blue_Serial_Init();
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

  // Interrupt initialization.
  pit_ms_init(PIT_CH0, 20);                                // PIT0 periodic interrupt: 20 ms.
  pit_ms_init(PIT_CH1, 10);                                // PIT1 periodic interrupt: 10 ms.
  pit_ms_init(PIT_CH2, 2);                                 // PIT2 periodic interrupt: 2 ms.
  interrupt_set_priority(PIT_IRQn, PIT_SHARED_IRQ_PRIORITY); // Shared PIT interrupt priority.
  interrupt_set_priority(LPUART1_IRQn, 3);   // UART1 interrupt priority.
#if MOTOR_BOARD_USE_NEW
  interrupt_set_priority(LPUART4_IRQn, 8);   // UART4 vision recognition interrupt priority.
  /* UART8 priority and RX interrupt state are owned by BlueSerial. */
#else
  interrupt_set_priority(LPUART8_IRQn, 8);   // Old-board Bluetooth or recognition UART.
#endif
  interrupt_global_enable(0);

  control_init();

  // Main loop.
  while (1)
  {
    if (!BlueSerial_IsControlActive())
    {
      control_process();
    }

    Menu_Switch();
    Menu_Show();
    BlueSerial_Task();
  }
}

/* PIT0 周期中断（20ms）：负责按键扫描。 */
void pit_0_handler(void)
{
  key_scanner();
}

void pit_1_handler(void)
{
  // Sample encoders before either autonomous or Bluetooth motor control.
  encoder_get();
  control_tick_10ms();
  // Schedule one compact car-state frame every 100 ms.  Formatting and UART
  // transmission stay in BlueSerial_Task(), outside the control interrupt.
  BlueSerial_TelemetryTick10ms();
  // Bluetooth takeover: when active, let BlueSerial own this 10ms motor-control tick.
  if (BlueSerial_IsControlActive())
  {
    BlueSerial_ControlTick10ms();
    return;
  }
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
    motor_control(speed_encoder);
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
  /* 姿态解算始终更新；规划保护期只暂停速度目标，不能让 IMU yaw 短暂停更。 */
  Attitude_Calculate();
}
