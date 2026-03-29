#ifndef _ALGORITHM_H
#define _ALGORITHM_H

#include "Camera_handler.h"

#define available_max_grid_size 140 // 不含墙的有效区域
#define max_boxes_cnt 10            // 最大箱子数量
#define max_bombs_cnt 5             // 支持的最大炸弹数量

/*方格识别码*/
#define OBSTACLE 0x01
#define BOMB 0x02
#define BLOCKED_BOMB 0x04
#define BOX 0x08

/*限制边参数*/
#define banned_edge_cnt (4 * available_max_grid_size)
#define banned_edge_volume ((banned_edge_cnt + 31) / 32)

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

typedef struct
{
    int f_cost;
    int g_cost;
    int h_cost;
    int parent_index;
    uint8_t open_or_close;
} a_star_param;
typedef struct
{
    int f_cost;
    int g_cost;
    int h_cost;
    int parent_index;
    uint8_t open_or_close;
    uint8_t is_push;
} a_star_3d_param;
typedef struct
{
    int index[available_max_grid_size];
    int size;
} binary_heap;
typedef struct
{
    int index[available_max_grid_size * 4];
    int size;
} binary_heap_3d;

typedef struct
{
    int total_steps;
    Position car_path[available_max_grid_size * 8];
    int used_bomb;
    int bomb_index;
    Position bomb_target;
    Position wall_target;
    Position box_target;
} path_plan_result;

void simulate_bomb_explosion(Position *obstacles, int *obstacles_cnt, Position bomb_target);
int integrated_path_output(int row_cnt, int col_cnt,
                           const Position *obstacles, int obstacles_cnt,
                           const Position *bombs, int bombs_cnt,
                           const Position *boxes, int boxes_cnt,
                           const Position *targets, int targets_cnt,
                           int box_index,
                           Position car_start,
                           path_plan_result *result);

#define MAX_OBSTACLES 100
#define MAX_BOXES 10
#define MAX_TARGETS 10
#define MAX_BOMBS 10
#define MAX_CAR_PATH 250

// 当前生效对象数量
extern size_t Obstacles_count; // 当前障碍物数量
extern size_t Boxes_count;     // 当前箱子数量
extern size_t Targets_count;   // 当前目标点数量
extern size_t Bombs_count;     // 当前炸弹数量
extern size_t Car_path_count;  // 当前路径点数量（若使用）

// 当前生效地图对象与车辆位置
extern Position obstacles[MAX_OBSTACLES]; // 障碍物坐标列表
extern Position boxes[MAX_BOXES];         // 箱子坐标列表
extern Position targets[MAX_TARGETS];     // 目标点坐标列表
extern Position bombs[MAX_BOMBS];         // 炸弹坐标列表
extern Position car_path[MAX_CAR_PATH];   // 规划路径坐标列表（命名沿用原工程）
extern Position car;                      // 车辆整数栅格位置
extern CarPosition car_position;          // 车辆浮点栅格位置
extern CarPosition car_position_m;        // 车辆米制位置（row/col * GRID_SIZE_M）

#endif
