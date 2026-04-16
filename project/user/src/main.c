#include "zf_common_headfile.h"
#include "Flash.h"
#include "Mymenu.h"
#include "Attitude.h"
#include "Algorithm.h"
#include "Algorithm_Test.h"
#include "Game_logic.h"
#include "data_handle.h"
#include <string.h>
#include "BlueSerial.h"

int main(void)
{
  clock_init(SYSTEM_CLOCK_600M); // 不可删除
  debug_init();                  // 调试端口初始化

  // 模块初始化
  system_delay_ms(300); // 等待主板其他外设上电完成
  uart_blob_init();     // 摄像头串口接收与解析模块初始化
  flash_init();         // Flash模块初始化
  Menu_Init();          // 菜单系统初始化(包含按键，显示屏等相关初始化)
  imu963ra_init();      // IMU块初始化
  Attitude_Init();      // 姿态解算模块初始化
  Blue_Serial_Init();


  // 中断初始化
  pit_ms_init(PIT_CH0, 20);                // 初始化 PIT_CH0 为周期中断 20ms 周期
  interrupt_set_priority(PIT_IRQn, 2);     // 设置 PIT0 对周期中断的中断优先级为 2
  pit_ms_init(PIT_CH1, 10);                // 初始化 PIT_CH1 为周期中断 10ms 周期
  interrupt_set_priority(PIT_IRQn, 1);     // 设置 PIT1 对周期中断的中断优先级为 1
  pit_ms_init(PIT_CH2, 2);                 // 初始化 PIT_CH2 为周期中断 2ms 周期
  interrupt_set_priority(PIT_IRQn, 0);     // 设置 PIT2 对周期中断的中断优先级为 0
  interrupt_set_priority(LPUART1_IRQn, 3); // 设置中断优先级（中等）
  interrupt_global_enable(0);              // 中断使能

  

  // 主循环
  while (1)
  {
    
	BlueSerial_Printf("%.2f\r\n",eulerAngle.yaw);
  }
}

// 20ms中断：按键扫描
void pit_0_handler(void)
{
  key_scanner();
}

// 10ms中断：运动控制
void pit_1_handler(void)
{

}

// 2ms中断：姿态解算
void pit_2_handler(void)
{
  Attitude_Calculate();
}
