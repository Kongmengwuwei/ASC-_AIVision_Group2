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
#include "Algorithm.h"
#include "Algorithm_Test.h"
#include "Game_logic.h"
#include "data_handle.h"
#include <string.h>

// 打开新的工程或者工程移动了位置务必执行以下操作
// 第一步 关闭上面所有打开的文件
// 第二步 project->clean  等 待下方进度条走完

int main(void)
{
  clock_init(SYSTEM_CLOCK_600M); // 不可删除
  debug_init();                  // 调试端口初始化

  // 模块初始化
  system_delay_ms(300); // 等待主板其他外设上电完成
  uart_blob_init();     // 摄像头串口接收与解析模块初始化
  flash_init();         // Flash模块初始化
  Menu_Init();          // 菜单系统初始化(包含按键，显示屏等相关初始化)
  // imu963ra_init();      // IMU块初始化
  // Attitude_Init();      // 姿态解算模块初始化

  // 中断初始化
  pit_ms_init(PIT_CH0, 20);                // 初始化 PIT_CH0 为周期中断 20ms 周期
  interrupt_set_priority(PIT_IRQn, 2);     // 设置 PIT0 对周期中断的中断优先级为 2
  pit_ms_init(PIT_CH1, 10);                // 初始化 PIT_CH1 为周期中断 10ms 周期
  interrupt_set_priority(PIT_IRQn, 1);     // 设置 PIT1 对周期中断的中断优先级为 1
  pit_ms_init(PIT_CH2, 2);                 // 初始化 PIT_CH2 为周期中断 2ms 周期
  interrupt_set_priority(PIT_IRQn, 0);     // 设置 PIT2 对周期中断的中断优先级为 0
  interrupt_set_priority(LPUART1_IRQn, 3); // 设置中断优先级（中等）
  interrupt_global_enable(0);              // 中断使能

  /*以下为算法测试*/
	Menu_Show();
  // 在此直接输入地图
  // const char *map_text = ".#............\n"
  //                        "........#####.\n"
  //                        "#B###...#T..#.\n"
  //                        "....#...#T#.#.\n"
  //                        "....#####T#.#.\n"
  //                        "C......B..B.#.\n"
  //                        "..........###.\n"
  //                        "..............\n"
  //                        ".....####.....\n"
  //                        "..............\n";
  const char *map_text = ".#...#..#....T\n"
                         "...#..#.####..\n"
                         "#.###.#.#.....\n"
                         ".T#.#.#.#..#..\n"
                         ".##.#.........\n"
                         "..#.......##..\n"
                         "CD.......D.#..\n"
                         "..B.#.....BD..\n"
                         ".B..########..\n"
                         "....#T........\n";
  // const char *map_text = "......#....#T#\n"
  //                        "...........#.#\n"
  //                        ".....#.#...#.#\n"
  //                        ".#.#..B....#..\n"
  //                        ".#.#.#.#......\n"
  //                        "C#.#..B..#.#..\n"
  //                        ".....#.#.#.#..\n"
  //                        ".#.##.B..#.##.\n"
  //                        ".....#.#......\n"
  //                        "....T.......T.\n";
  // const char *map_text = ".#.T....#.....\n"
  //                        "........#..T..\n"
  //                        "......D.#.....\n"
  //                        "........#..B..\n"
  //                        "C...#####.....\n"
  //                        "..B.#.........\n"
  //                        "....#....B....\n"
  //                        "....#.........\n"
  //                        "....#.......T.\n"
  //                        "....#.........\n";

  parse_map_from_string(map_text);

  boxes[0].id = 0;
  boxes[1].id = 1;
  boxes[2].id = 2 ;
  targets[0].id = 0;
  targets[1].id = 1;
  targets[2].id = 2;

  Test_Data_Load(); // 数据加载到内部测试地图
  Plan_path_Mode2(); // 关卡一路径规划

  Test_Path_Init(); // 路径初始化
  /*以上为算法测试*/

  // 主循环
  while (1)
  {
    /*以下为算法测试*/
    Test_Data_Save(); // 将内部测试地图数据保存回全局数组
    /*以上为算法测试*/

    Menu_Switch();
    Menu_Show();

    // // 地图获取调试
    // static uint8_t map_req_sent = 0;
    // uint8_t row = 0;

    // if (!map_req_sent)
    // {
    //   uart_send_map_request(); // 向上位机发送 "MAP"
    //   map_req_sent = 1;
    // }
    // process_blob_data();

    // if (map_data_updated)
    // {
    //   uart_write_string(UART_INDEX, "$MAP\r\n");
    //   for (row = 0; row < MAP_ROWS; row++)
    //   {
    //     uart_write_buffer(UART_INDEX, (const uint8_t *)map_data[row], MAP_COLS);
    //     uart_write_string(UART_INDEX, "\r\n");
    //   }
    //   uart_write_string(UART_INDEX, "$END\r\n");
    //   map_data_updated = false;
    // }

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
  //  Attitude_Calculate();
}
