#ifndef _MAP_ROUTE_DATA_H_
#define _MAP_ROUTE_DATA_H_

#include "zf_common_typedef.h"

// 地图尺寸定义
#define MAP_ROWS 10
#define MAP_COLS 14

// 地图元素坐标上限定义
#define MAX_OBSTACLES 100
#define MAX_BOXES 10
#define MAX_TARGETS 10
#define MAX_BOMBS 10
#define MAX_CAR_PATH 250

typedef struct
{
    int row; // 栅格行号
    int col; // 栅格列号
} Position;
typedef struct
{
    float row; // 车辆行坐标（可带小数）
    float col; // 车辆列坐标（可带小数）
} CarPosition;

// 当前生效对象数量
extern size_t Obstacles_count; // 当前障碍物数量
extern size_t Boxes_count;     // 当前箱子数量
extern size_t Targets_count;   // 当前目标点数量
extern size_t Bombs_count;     // 当前炸弹数量
extern size_t Car_path_count;  // 当前路径点数量

// 当前生效地图对象与车辆位置
extern Position obstacles[MAX_OBSTACLES]; // 障碍物坐标列表
extern Position boxes[MAX_BOXES];         // 箱子坐标列表
extern Position targets[MAX_TARGETS];     // 目标点坐标列表
extern Position bombs[MAX_BOMBS];         // 炸弹坐标列表
extern Position car;                      // 车辆整数栅格位置
extern Position car_path[MAX_CAR_PATH];   // 规划路径坐标列表
extern CarPosition car_position;          // 车辆浮点栅格位置
extern CarPosition car_position_m;        // 车辆米制位置（row/col * GRID_SIZE_M）

#endif 
