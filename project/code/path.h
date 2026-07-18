#ifndef _PATH_H
#define _PATH_H

#include "Map_Path_Data.h"
#include <stddef.h>

/*
 * 路径执行坐标系补偿开关。
 *
 * 规划层使用地图坐标 (row, col)，path_follow 使用执行坐标 (x=row, y=col)。
 * 当前实车现象需要先行列转置，再翻转执行坐标的上下轴；Control 的视觉位姿映射
 * 和本模块的执行点 remap 必须共用这两个开关。
 */
#define PATH_COORD_TRANSPOSE_COMPENSATE 1U
#define PATH_COORD_FLIP_VERTICAL 1U
#define PATH_SAFE_RELOCATION_MAX_POINTS (MAP_ROWS * MAP_COLS)

#if PATH_COORD_TRANSPOSE_COMPENSATE
#define PATH_WORLD_X_MAX_M ((float)(MAP_COLS - 1) * GRID_SIZE_M)
#define PATH_WORLD_Y_MAX_M ((float)(MAP_ROWS - 1) * GRID_SIZE_M)
#else
#define PATH_WORLD_X_MAX_M ((float)(MAP_ROWS - 1) * GRID_SIZE_M)
#define PATH_WORLD_Y_MAX_M ((float)(MAP_COLS - 1) * GRID_SIZE_M)
#endif

/**
 * @brief 路径压缩需要用到的地图对象快照。
 *
 * Control 在规划前保存一份快照，规划后再保存一份模拟后的地图。
 * path.c 同时参考两份地图，避免斜线/捷径只对某一个时刻的对象位置安全。
 */
typedef struct
{
    size_t obstacles_count;
    size_t boxes_count;
    size_t targets_count;
    size_t bombs_count;
    Position obstacles_buf[MAX_OBSTACLES];
    Position boxes_buf[MAX_BOXES];
    Position targets_buf[MAX_TARGETS];
    Position bombs_buf[MAX_BOMBS];
    Position car_pose_grid;
} path_map_snapshot_t;

/*
 * Snapshot protocol used by path_build_exec_from_planner():
 * extra_map is the state before planning and current_map is the simulated final
 * state. path.c reconstructs intermediate states from path event bits. If the
 * reconstruction cannot be verified against current_map, shortcut checks fall
 * back to the conservative union of the two snapshots.
 */

/**
 * @brief 将地图规划坐标转换为 path_follow 使用的执行坐标。
 */
void path_remap_exec_point(Position *p);

/**
 * @brief 将 path_follow 执行坐标还原为地图规划坐标。
 */
void path_inverse_remap_exec_point(Position *p);

/**
 * @brief 设置是否允许执行路径使用斜线捷径。
 *
 * @param enabled 1：允许斜线；0：只允许水平/竖直执行段。
 */
void path_set_diagonal_enabled(uint8 enabled);

/**
 * @brief 获取当前斜线捷径开关状态。
 *
 * @return uint8 1：允许斜线；0：只允许水平/竖直执行段。
 */
uint8 path_get_diagonal_enabled(void);

/* 识别路径后处理开关：保持原始路线坐标不变，只移动识别事件。先尽可能把两格识别
 * 换成一格识别；近距离识别数相同时优先减少识别车头转向，且不允许增加实际路程。
 * 默认开启，测试时可关闭以获得基线结果。 */
void path_set_identify_near_postprocess_enabled(uint8 enabled);
uint8 path_get_identify_near_postprocess_enabled(void);

/**
 * @brief 从规划路径生成最终下发给 path_follow 的执行路径。
 *
 * @param planner_path Game_logic/Algorithm 输出的原始规划路径，仍为地图坐标。
 * @param planner_steps 原始规划路径点数。
 * @param current_map 规划完成后的地图快照。
 * @param extra_map 额外参与障碍检查的地图快照，通常为规划前地图，可为 NULL。
 * @param exec_path 输出执行路径，函数会在最后转换为 path_follow 执行坐标。
 * @param exec_capacity 输出路径数组容量。
 * @param exec_steps 输出执行路径点数。
 * @return uint8 1 表示构建成功，0 表示输入无效或没有可用执行路径。
 */
uint8 path_build_exec_from_planner(const Position *planner_path,
                                   size_t planner_steps,
                                   const path_map_snapshot_t *current_map,
                                   const path_map_snapshot_t *extra_map,
                                   Position *exec_path,
                                   size_t exec_capacity,
                                   size_t *exec_steps);

/**
 * @brief 搜索最近可达的不贴墙格，并生成经过现有路径提取/捷径处理的执行路径。
 *
 * 安全格自身及四邻域不能有墙；允许邻近箱子和目标点，但路线和终点不能
 * 穿过或占用墙、箱子、目标点或炸弹。
 *
 * @param start_map 起点，使用地图坐标。
 * @param map 当前执行阶段的地图快照。
 * @param exec_path 输出路径，使用 path_follow 执行坐标。
 * @param exec_capacity 输出数组容量。
 * @param exec_steps 输出点数；当前位置已经安全时为 1。
 * @return uint8 1 表示找到安全格并成功生成路径，0 表示无可达安全格或输入无效。
 */
uint8 path_build_nearest_wall_clear_exec(const Position *start_map,
                                         const path_map_snapshot_t *map,
                                         Position *exec_path,
                                         size_t exec_capacity,
                                         size_t *exec_steps);

#endif
