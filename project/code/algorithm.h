#ifndef __ALGORITHM_H
#define __ALGORITHM_H
#include "zf_common_headfile.h"
//typedef 或 宏定义
#define grid_size               192        //网格尺寸(含墙)
#define available_max_grid_size 140        //不含墙的有效区域
#define max_boxes_cnt           10         //最大箱子数量
#define max_bombs_cnt           5          //支持的最大炸弹数量*

    /*方格识别码*/
#define OBSTACLE                0x01
#define BOMB                    0x02
#define BLOCKED_BOMB            0x04
#define BOX                     0x08

    /*限制边参数*/
#define banned_edge_cnt         (4 * available_max_grid_size)
#define banned_edge_volume      ((banned_edge_cnt + 31) / 32)

typedef struct{
int8_t row; int8_t col;
}position;

typedef struct{
int f_cost; int g_cost; int h_cost;
int parent_index;
uint8_t open_or_close;
}a_star_param;
typedef struct {
int f_cost; int g_cost; int h_cost;
int parent_index;
uint8_t open_or_close; uint8_t is_push;
}a_star_3d_param;
typedef struct{
int index[available_max_grid_size];
int size;
}binary_heap;
typedef struct {
int index[available_max_grid_size * 4];
int size;
}binary_heap_3d;
//外部变量

//函数声明

#endif
