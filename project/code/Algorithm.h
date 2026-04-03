#ifndef _ALGORITHM_H
#define _ALGORITHM_H

#include "Map_Route_Data.h"
#include "Camera_handler.h"

#define grid_size       140 // 不含墙的有效区域

/*方格识别码*/
#define OBSTACLE        0x01
#define BOMB            0x02
#define BLOCKED_BOMB    0x04
#define BOX             0x08

/*限制边参数*/
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

void simulate_bomb_explosion(Position *obstacles, int *obstacles_cnt, Position bomb_target);
int integrated_path_output(int row_cnt, int col_cnt,
                           const Position *obstacles, int obstacles_cnt,
                           const Position *bombs, int bombs_cnt,
                           const Position *boxes, int boxes_cnt,
                           const Position *targets, int targets_cnt,
                           int box_index,
                           Position car_start,
                           path_plan_result *result);


                           
#endif
