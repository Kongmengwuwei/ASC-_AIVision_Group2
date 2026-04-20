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
#include <string.h>

#define PIT_PRIORITY_0 (PIT_IRQn)
#define PIT_PRIORITY_1 (PIT_IRQn)
#define PIT_PRIORITY_2 (PIT_IRQn)

int car_stop_array[4] = {0};

int main(void)
{
  clock_init(SYSTEM_CLOCK_600M); // 不可删除
  debug_init();                  // 调试端口初始化

  // 模块初始化
  system_delay_ms(300); // 等待主板其他外设上电完成
  uart_blob_init();     // 摄像头串口接收与解析模块初始化
  flash_init();         // Flash模块初始化
  Menu_Init();          // 菜单系统初始化(包含按键，显示屏等相关初始化)
  motor_init();         // 电机控制模块初始化
  encoder_init();       // 电机编码器模块初始化
  imu963ra_init();      // IMU模块初始化
  Attitude_Init();      // 姿态解算模块初始化

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

  // 中断初始化
  pit_ms_init(PIT_CH0, 20);                  // PIT0 periodic interrupt: 20 ms.
  interrupt_set_priority(PIT_PRIORITY_0, 2); // PIT0 interrupt priority.
  pit_ms_init(PIT_CH1, 10);                  // PIT1 periodic interrupt: 10 ms.
  interrupt_set_priority(PIT_PRIORITY_1, 1); // PIT1 interrupt priority.
  pit_ms_init(PIT_CH2, 2);                   // PIT2 periodic interrupt: 2 ms.
  interrupt_set_priority(PIT_PRIORITY_2, 0); // PIT2 interrupt priority.
  interrupt_set_priority(LPUART1_IRQn, 3);   // UART1 interrupt priority.
  interrupt_set_priority(LPUART4_IRQn, 4);   // UART4 interrupt priority.
  interrupt_global_enable(0);

  /*以下为算法测试*/
  Menu_Show();
  if (Algo_Test_auto || Algo_Test_hand)
  {
  parse_map_from_string(map_text3);

  boxes[0].id = 0;
  boxes[1].id = 1;
  boxes[2].id = 2;
  targets[0].id = 0;
  targets[1].id = 1;
  targets[2].id = 2;

  Test_Data_Load();  // 数据加载到内部测试地图
  Plan_path_Mode2(); // 关卡一路径规划

  Test_Path_Init(); // 路径初始化
  }
  /*以上为算法测试*/

  control_init();

  // 主循环
  while (1)
  {
    /*以下为算法测试*/
    if (Algo_Test_auto || Algo_Test_hand)
    {
      Test_Data_Save(); // 将内部测试地图数据保存回全局数组
    }
    /*以上为算法测试*/

    // 控制主流程：初始定位 -> 摄像头数据到齐 -> 路径规划 -> 路径执行 -> 执行中动态校正
    control_process();

    // 菜单系统主循环调用：响应按键事件，更新显示等。
    Menu_Switch();
    Menu_Show();

    ips200_show_float(0, 100, eulerAngle.yaw, 3, 2); // 显示当前航向角，便于调试
  }
}

/* PIT0 周期中断（20ms）：负责按键扫描。 */
void pit_0_handler(void)
{
  key_scanner();
}

/* PIT1 周期中断（10ms）：编码器采样 + 速度环控制输出。 */
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
    // 未进入运行态，直接关闭四路 PWM
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
