#ifndef _ALGORITHM_TEST_H
#define _ALGORITHM_TEST_H

#include "Map_Route_Data.h"

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

extern size_t s_path_index; // 当前路径执行到的下一个点索引(全局可见，供菜单显示)

// 从全局识别结果加载地图
void Test_Data_Load(void);

// 将内部状态回写到全局数组
void Test_Data_Save(void);

// 执行一次小车移动
Move_Result Move_car(char move_cmd);

// 路径执行初始化：将小车对齐路径起点，准备执行路径。
void Test_Path_Init(void);

/*
 * 执行 Map_Route_Data.h 中的 car_path 路径中的下一步：
 * 返回值：
 * 1    : 成功执行一步
 * 0    : 路径执行完毕
 * -1   : 未初始化（需先调用 Test_Path_Player_Init）
 * -3   : 路径存在非法跳点（非上下左右一步）
 * -4   : 执行被阻塞（Move_car 返回 MOVE_BLOCKED）
 *
 * 可选输出：
 * out_cmd   : 本步实际发给 Move_car 的指令（W/S/A/D）
 */
int Test_Path_Step(char *out_cmd);

/*
 * 按 Map_Route_Data.h 中的 car_path 执行整段路径测试：
 * 返回值：
 * >= 0 : 成功执行的移动步数
 * -1   : 地图未初始化或路径点数量不足（<2）
 * -2   : 路径起点越界
 * -3   : 路径存在非单步相邻点（不是上下左右一步）
 * -4   : 执行过程中某一步被阻塞（Move_car 返回 MOVE_BLOCKED）
 * 说明：若路径中存在相邻重复点，会自动跳过该点。
 */
int Test_Path_ALL(void);

#endif
