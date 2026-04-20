#ifndef _CONTROL_H
#define _CONTROL_H

#include "zf_common_typedef.h"
#include "data_handle.h"
#include "Map_Path_Data.h"
#include "path_follow.h"

/**
 * @file Control.h
 * @brief 视觉推箱车主控制流程接口。
 *
 * 该模块负责把上层任务拆成固定阶段并串起来执行：
 * 1) 启动右移出发车区（向右一格）
 * 2) 启动初始定位
 * 3) 等待摄像头地图数据
 * 4) 路径规划
 * 5) 路径下发并执行
 * 6) 执行中动态校正
 */

/**
 * @brief 控制流程阶段定义。
 */
typedef enum
{
    CONTROL_STAGE_IDLE = 0,         /**< 空闲态：流程未启动。 */
    CONTROL_STAGE_PRESTART_MOVE,    /**< 起步态：先向右移动一格，驶出发车区。 */
    CONTROL_STAGE_STARTUP_LOCALIZE, /**< 初始定位态：采集相机位姿并对齐里程计。 */
    CONTROL_STAGE_WAIT_CAMERA_DATA, /**< 等待地图态：周期请求并等待摄像头地图帧。 */
    CONTROL_STAGE_PLAN_PATH,        /**< 规划态：调用 Game_logic 进行路径规划。 */
    CONTROL_STAGE_LOAD_PATH,        /**< 下发态：把规划结果转换并下发给 path_follow。 */
    CONTROL_STAGE_EXECUTE_PATH,     /**< 执行态：沿路径运动并进行动态校正。 */
    CONTROL_STAGE_FINISHED,         /**< 完成态：整段路径执行完毕并保持停车。 */
    CONTROL_STAGE_ERROR             /**< 错误态：规划/下发失败，等待新地图后重试。 */
} control_stage_t;

extern control_stage_t g_control_stage;

/**
 * @brief 初始化控制状态机（上电后调用一次）。
 *
 * 主要动作：
 * - 清空控制内部缓存与标志位
 * - 关闭运动输出（car_go_flag/car_stop_flag）
 * - 把流程状态切到“起步右移”阶段
 */
void control_init(void);

/**
 * @brief 控制主循环入口（在 main 的 while(1) 中持续调用）。
 *
 * 该函数会在每次调用时推进状态机一步，并根据当前阶段执行：
 * - 串口数据解析
 * - 初始定位
 * - 路径规划与下发
 * - 动态校正与完结判断
 */
void control_process(void);

/**
 * @brief 人工触发重新开始一轮“定位->规划->执行”流程。
 *
 * 典型用途：
 * - 用户手动复位任务
 * - 需要基于新场景重新开始
 */
void control_restart(void);

/**
 * @brief 获取当前控制阶段。
 *
 * @return control_stage_t 当前状态机阶段值。
 */
control_stage_t control_get_stage(void);

/**
 * @brief 查询是否处于“路径规划保护期”。
 *
 * 保护期用于在规划/切换路径时暂停速度目标更新，避免执行层继续
 * 输出旧路径命令，通常在 PIT 中断里读取该状态进行保护。
 *
 * @return uint8
 * - 1：处于保护期
 * - 0：非保护期
 */
uint8 control_is_path_plan_paused(void);

#endif
