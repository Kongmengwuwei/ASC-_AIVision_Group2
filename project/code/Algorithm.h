#ifndef _ALGORITHM_H
#define _ALGORITHM_H

#include "Map_Path_Data.h"

/* 不含外墙的有效规划栅格总数。 */
#define grid_size 140

/* 网格位标记。 */
#define OBSTACLE        0x01
#define BOMB            0x02
#define BLOCKED_BOMB    0x04
#define BOX             0x08

/* A* 边约束位图参数（预留）。 */
#define banned_edge_cnt     (4 * grid_size)
#define banned_edge_volume  ((banned_edge_cnt + 31) / 32)

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
    int index[grid_size];
    int size;
} binary_heap;

typedef struct
{
    int index[grid_size * 4];
    int size;
} binary_heap_3d;

/* 单次规划输出：完整小车路径 + 是否使用炸弹 + 关键事件点。 */
typedef struct
{
    int total_steps;
    Position car_path[grid_size * 8];
    int used_bomb;
    int bomb_index;
    Position bomb_target;
    Position wall_target;
    Position box_target;
} path_plan_result;

/* 模拟炸弹爆炸后的地形变化（清除 3x3 范围内障碍）。 */
void simulate_bomb_explosion(Position *obstacles, int *obstacles_cnt, Position bomb_target);

/* 单箱子综合规划入口：在“直推”与“先炸后推”方案中选最优。 */
int integrated_path_output(int row_cnt, int col_cnt,
                           const Position *obstacles, int obstacles_cnt,
                           const Position *bombs, int bombs_cnt,
                           const Position *boxes, int boxes_cnt,
                           const Position *targets, int targets_cnt,
                           int box_index,
                           Position car_start,
                           path_plan_result *result);

#endif
