#ifndef _CONTROL_SIMPLE_H
#define _CONTROL_SIMPLE_H

#include "Map_Path_Data.h"
#include "zf_common_typedef.h"

#define CONTROL_PRESTART_DEPART_DIR_MIN 0U
#define CONTROL_PRESTART_DEPART_DIR_MAX 4U

/* 自定义路径点标记：到达后转向面对目标 + 暂停 2 秒。
 * 方向相对于路径前进方向（FAKE 点 → 下一个路径点）：
 *   UP(11)    = 正前方（base）
 *   RIGHT(10) = 右侧（base + 90°）
 *   DOWN(13)  = 后方（base + 180°）
 *   LEFT(12)  = 左侧（base - 90°）
 */
#define FAKE_IDENTIFY_FACE_RIGHT  10U
#define FAKE_IDENTIFY_FACE_UP     11U
#define FAKE_IDENTIFY_FACE_LEFT   12U
#define FAKE_IDENTIFY_FACE_DOWN   13U

/**
 * @brief 简化版控制流程阶段定义。
 */
typedef enum
{
    CONTROL_STAGE_IDLE = 0,              /**< 空闲态：流程未启动。 */
    CONTROL_STAGE_PRESTART_MOVE = 1,     /**< 起步态：沿选定方向平移驶出发车区。 */

    CONTROL_STAGE_FAKE_LOCALIZE = 20,         /**< 定位态：IMU 航向作为基准，里程计归零。 */
    CONTROL_STAGE_FAKE_IDENTIFY_PAUSE = 21,   /**< 伪识别态：延时等待，假装在做摄像头识别。 */
    CONTROL_STAGE_LOAD_CUSTOM_PATH = 22,      /**< 下发态：装载自定义路径到 path_follow。 */
    CONTROL_STAGE_EXECUTE_PATH = 23,          /**< 执行态：沿路径运动，中途暂停点自动停车。 */
    CONTROL_STAGE_FINISHED = 24,              /**< 完成态：全部路径执行完毕，停车保持。 */

    CONTROL_STAGE_ERROR = 99                  /**< 错误态：延时后自动重试。 */
} control_stage_t;

extern control_stage_t g_control_stage;

/**
 * @brief 用户定义的自定义路径。
 *
 * 路径点使用栅格坐标 (row, col)，path_follow 内部会按 GRID_SIZE_M 换算为米。
 * - row / col : 栅格坐标，相邻点必须满足 |Δrow|≤1 且 |Δcol|≤1。
 * - id        : 0 = 普通点，FAKE_IDENTIFY_POINT = 到达后暂停。
 */
extern Position g_custom_path[MAX_CAR_PATH];
extern size_t g_custom_path_steps;

/* ========================= 对外接口 ========================= */

void control_init(void);
void control_process(void);
void control_restart(void);
void control_set_start_enabled(uint8 enabled);
uint8 control_get_start_enabled(void);
void control_set_prestart_depart_dir(uint8 dir);
uint8 control_get_prestart_depart_dir(void);
control_stage_t control_get_stage(void);
const Position *control_get_exec_path(size_t *steps);
void control_set_diagonal_path_enabled(uint8 enabled);
uint8 control_get_diagonal_path_enabled(void);
uint8 control_is_path_plan_paused(void);

#endif
