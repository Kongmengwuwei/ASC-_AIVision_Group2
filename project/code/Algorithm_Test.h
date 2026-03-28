#ifndef _ALGORITHM_TEST_H
#define _ALGORITHM_TEST_H

#include "Camera_handler.h"
#include "zf_common_typedef.h"

/*
 * 内部地图单元位标记：
 * 一个格子可同时包含多种状态，所以使用按位或方式存储。
 * 例如：某格同时是目标点和箱子，位图中会同时包含 TARGET 和 BOX。
 */
#define CELL_OBSTACLE 0x01
#define CELL_BOX      0x02
#define CELL_TARGET   0x04
#define CELL_BOMB     0x08

typedef enum
{
    MOVE_BLOCKED = 0,            // 移动失败：撞墙、越界、或推动条件不满足
    MOVE_OK = 1,                 // 普通移动成功（可包含推箱子/推炸弹但未触发特殊效果）
    MOVE_BOX_TARGET_CLEARED = 2, // 箱子被推入目标点，箱子和目标点一起消失
    MOVE_BOMB_EXPLODED = 3       // 炸弹撞墙触发爆炸，清除墙体
} Move_Result;

// 从 Camera_handler 的全局识别结果读取地图元素并初始化内部地图。
void Test_Data_Load(void);

// 将内部地图状态回写到全局数组（obstacles/boxes/targets/map_bombs/car）。
void Test_Data_Save(void);

/*
 * 执行一步移动。
 * 支持命令：W/S/A/D。
 * 规则：
 * 1) 小车撞墙不可前进；
 * 2) 小车可把箱子沿运动方向推动 1 格；
 * 3) 箱子被推到目标点时，箱子与目标点同时消失；
 * 4) 小车可推动炸弹，炸弹撞墙时炸弹消失，并清除撞击墙周围 3x3 范围内的墙。
 */
Move_Result Move_car(char move_cmd);

#endif
