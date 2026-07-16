#ifndef _MAP_PATH_DATA_H_
#define _MAP_PATH_DATA_H_

#include <stddef.h>
#include "zf_common_typedef.h"

// 地图尺寸定义
#define MAP_ROWS 10
#define MAP_COLS 14
#define GRID_SIZE_M 0.20f

// 地图元素坐标上限定义
#define MAX_OBSTACLES 100
#define MAX_BOXES 10
#define MAX_TARGETS 10
#define MAX_BOMBS 10
#define MAX_CAR_PATH 1000
#define MAP_PRESET_UNKNOWN_ID 0U

typedef enum
{
    MAP_PRESET_PLAN_IDENTIFY = 0U,
    MAP_PRESET_PLAN_MODE1 = 1U,
    MAP_PRESET_PLAN_MODE2 = 2U
} map_preset_plan_mode_t;

/*
 * 路径事件位掩码（Position.id 在路径上下文中的含义）。
 * 同一路径点可能同时是推箱终点、爆炸点和转折点，必须用按位或组合，
 * 判断事件时也必须使用 (id & EVENT) 而不是相等比较。
 */
#define PATH_EVENT_NONE       0x00U
#define BOMB_EXPLOSION        0x01U  // 炸弹爆炸位置标记
#define TURNING_POINT         0x02U  // 路径转折点标记
#define IDENTIFICATION        0x04U  // 识别位置统一标记
#define PUSH_START_POINT      0x08U  // 推箱起步位置标记
#define PUSH_END_POINT        0x10U  // 推箱结束位置标记
/* 保留旧拼写，兼容已有调用代码。 */
#define PUSH_StART_POINT      PUSH_START_POINT
#define PATH_ALL_EVENTS       (BOMB_EXPLOSION | TURNING_POINT | IDENTIFICATION | \
                               PUSH_START_POINT | PUSH_END_POINT)
#define PATH_REQUIRED_EVENTS  (BOMB_EXPLOSION | IDENTIFICATION | \
                               PUSH_START_POINT | PUSH_END_POINT)

typedef struct
{
    uint8 row; // 栅格行号
    uint8 col; // 栅格列号
    uint8 id;  // 对象上下文为编号；路径上下文为 PATH_* 事件位掩码
} Position;

typedef struct
{
    int32 x_raw;
    int32 y_raw;
    int32 yaw_raw;
    float x;
    float y;
    float yaw;
} CarPose;

typedef struct
{
    const char *name;
    const char *map_text;
    map_preset_plan_mode_t plan_mode;
    float car_yaw_deg;
    const uint8 *box_ids;
    size_t box_id_count;
    const uint8 *target_ids;
    size_t target_id_count;
} MapPresetTextConfig;

typedef struct
{
    const char *name;
    map_preset_plan_mode_t plan_mode;
    Position car_start;
    float car_yaw_deg;
    size_t obstacles_count;
    size_t boxes_count;
    size_t targets_count;
    size_t bombs_count;
    Position obstacles[MAX_OBSTACLES];
    Position boxes[MAX_BOXES];
    Position targets[MAX_TARGETS];
    Position bombs[MAX_BOMBS];
} MapPresetConfig;

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
extern CarPose car_position;              // 车辆浮点栅格位置

extern const char map_text0[]; // 预设地图文本0
extern const char map_text1[]; // 预设地图文本1
extern const char map_text2[]; // 预设地图文本2
extern const char map_text3[]; // 预设地图文本3
extern const char map_text4[]; // 预设地图文本4
extern const char map_text5[]; // 预设地图文本5
extern const char map_text6[]; // 预设地图文本6
extern const char map_text7[]; // 预设地图文本7（1-3.txt）
extern const char map_text8[]; // 预设地图文本8（2-3.txt）

extern const MapPresetTextConfig map_preset_texts[];
extern const size_t Map_preset_count;
uint8 Map_Preset_BuildConfig(size_t preset_index, MapPresetConfig *out_config);

#endif 
