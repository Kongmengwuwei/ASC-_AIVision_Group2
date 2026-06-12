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

//路径特殊点ID定义
#define BOMB_EXPLOSION 1        // 炸弹爆炸位置标记
#define TURNING_POINT  2        // 路径转折点标记
#define IDENTIFICATION 3        // 识别位置统一标记
#define PUSH_StART_POINT 4        // 推箱起步位置标记
#define PUSH_END_POINT 5        // 推箱结束位置标记

typedef struct
{
    uint8 row; // 栅格行号
    uint8 col; // 栅格列号
    uint8 id;  // 元素ID（箱子编号，目标编号，路径特殊点）
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

extern const MapPresetTextConfig map_preset_texts[];
extern const size_t Map_preset_count;
uint8 Map_Preset_BuildConfig(size_t preset_index, MapPresetConfig *out_config);

#endif 
