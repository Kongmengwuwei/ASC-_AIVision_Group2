#ifndef _ALGORITHM_TEST_H
#define _ALGORITHM_TEST_H

#include "Camera_handler.h"
#include "Algorithm.h"

// 地图位图标志定义（按位存储）
#define CELL_OBSTACLE 0x01
#define CELL_BOX      0x02
#define CELL_TARGET   0x04
#define CELL_BOMB     0x08

//运动结果枚举
typedef enum
{
    MOVE_BLOCKED = 0,
    MOVE_OK = 1,
    MOVE_BOX_TARGET_CLEARED = 2,
    MOVE_BOMB_EXPLODED = 3
} Move_Result;

/*
 * 执行一次小车移动：
 * 1) 撞墙不能走；
 * 2) 可推动箱子一格；
 * 3) 箱子进目标点后，箱子和目标点同时消失；
 * 4) 可推动炸弹；炸弹撞墙时爆炸并清除3x3墙体；
 * 5) 最外圈障碍物不可被炸毁。
 */
Move_Result Move_car(char move_cmd);

// 从全局识别结果加载地图
void Test_Data_Load(void);

// 将内部状态回写到全局数组
void Test_Data_Save(void);

#endif
