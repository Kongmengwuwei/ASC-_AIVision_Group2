#ifndef _CONTROL_H
#define _CONTROL_H

#include "Map_Path_Data.h"
#include "zf_common_typedef.h"

#define CONTROL_PRESTART_DEPART_DIR_MIN 0U
#define CONTROL_PRESTART_DEPART_DIR_MAX 4U

/**
 * @brief 控制流程阶段定义。
 */
typedef enum
{
    CONTROL_STAGE_IDLE = 0,          /**< 空闲态：流程未启动。 */
    CONTROL_STAGE_PRESTART_MOVE = 1, /**< 起步态：先沿车头方向运动 0.2m，驶出发车区。 */

    /* 识别阶段：负责到达识别点、识别箱子/目标 ID，并保存给后续推箱子阶段使用。 */
    CONTROL_STAGE_IDENTIFY_LOCALIZE = 20,        /**< 识别-定位态：采集相机位姿并对齐里程计。 */
    CONTROL_STAGE_IDENTIFY_WAIT_CAMERA_DATA = 21, /**< 识别-等待地图态：请求并等待识别用地图帧。 */
    CONTROL_STAGE_IDENTIFY_PLAN_PATH = 22,       /**< 识别-规划态：调用识别路径规划。 */
    CONTROL_STAGE_IDENTIFY_LOAD_PATH = 23,       /**< 识别-下发态：切分识别路径并准备分段执行。 */
    CONTROL_STAGE_IDENTIFY_EXECUTE_PATH = 24,    /**< 识别-执行态：移动、转向并触发识别。 */
    CONTROL_STAGE_IDENTIFY_FINISHED = 25,        /**< 识别-完成态：识别流程结束，准备切入推箱子。 */

    /* 推箱子阶段：使用识别结果规划真实推箱路径，并下发给 path_follow 执行。 */
    CONTROL_STAGE_PUSHBOX_LOCALIZE = 30,         /**< 推箱子-定位态：重新采集车位姿，降低阶段切换误差。 */
    CONTROL_STAGE_PUSHBOX_WAIT_CAMERA_DATA = 31, /**< 推箱子-等待地图态：请求并等待推箱子地图帧。 */
    CONTROL_STAGE_PUSHBOX_PLAN_PATH = 32,        /**< 推箱子-规划态：调用 Game_logic 进行推箱路径规划。 */
    CONTROL_STAGE_PUSHBOX_LOAD_PATH = 33,        /**< 推箱子-下发态：把执行路径下发给 path_follow。 */
    CONTROL_STAGE_PUSHBOX_EXECUTE_PATH = 34,     /**< 推箱子-执行态：沿路径运动并处理爆炸点停留。 */
    CONTROL_STAGE_PUSHBOX_FINISHED = 35,         /**< 推箱子-完成态：整段任务完成并保持停车。 */

    CONTROL_STAGE_ERROR = 99                     /**< 错误态：规划/下发失败，等待新地图后重试。 */
} control_stage_t;

/**
 * @brief 路径规划模式枚举。
 *
 * CONTROL_PLAN_MODE_1:
 * 在 CONTROL_STAGE_PUSHBOX_PLAN_PATH 阶段调用 Plan_path_Mode1()。
 *
 * CONTROL_PLAN_MODE_2:
 * 在 CONTROL_STAGE_PUSHBOX_PLAN_PATH 阶段调用 Plan_path_Mode2()。
 *
 * CONTROL_PLAN_MODE_IDENTIFY:
 * 预留值。识别流程由控制状态机首轮自动执行，不通过该标志手动切换。
 */
typedef enum
{
    CONTROL_PLAN_MODE_1 = 1U,
    CONTROL_PLAN_MODE_2 = 2U,
    CONTROL_PLAN_MODE_IDENTIFY = 0U
} control_plan_mode_t;

extern control_stage_t g_control_stage;

/**
 * @brief 初始化控制状态机（上电后调用一次）。
 *
 * 主要动作：
 * - 清空控制内部缓存与标志位
 * - 关闭运动输出（car_go_flag/car_stop_flag）
 * - 把流程状态切到“起步动作”阶段
 */
void control_init(void);

/**
 * @brief 控制主循环入口（在 main 的 while(1) 中持续调用）。
 *
 * 该函数会在每次调用时推进状态机一步，并根据当前阶段执行：
 * - 串口数据解析
 * - 初始定位
 * - 路径规划与下发
 * - 路径执行状态与完结判断
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

void control_set_start_enabled(uint8 enabled);

uint8 control_get_start_enabled(void);

/**
 * @brief 设置起步发车方向选项。
 *
 * 取值会被限制在 CONTROL_PRESTART_DEPART_DIR_MIN ~ CONTROL_PRESTART_DEPART_DIR_MAX。
 *
 * 当前方向约定：
 * - 0：地图右
 * - 1：地图上
 * - 2：地图左
 * - 3：地图下
 * - 4：保留/默认，按地图右处理
 *
 * @param dir 菜单输入的发车方向编号。
 */
void control_set_prestart_depart_dir(uint8 dir);

/**
 * @brief 获取当前起步发车方向选项。
 *
 * @return uint8 当前发车方向编号。
 */
uint8 control_get_prestart_depart_dir(void);

/**
 * @brief 获取当前控制阶段。
 *
 * @return control_stage_t 当前状态机阶段值。
 */
control_stage_t control_get_stage(void);

/**
 * @brief 获取当前执行路径缓存。
 *
 * 返回的是已经转换到 path_follow 执行坐标系的路径点；若用于地图显示，
 * 需要调用 path_inverse_remap_exec_point() 转回地图栅格坐标。
 *
 * @param[out] steps 当前执行路径点数量，可传 NULL。
 * @return const Position* 执行路径只读指针。
 */
const Position *control_get_exec_path(size_t *steps);

/**
 * @brief 设置是否允许执行路径使用斜线捷径。
 *
 * 该开关影响下一次执行路径构建；已经下发给 path_follow 的路径不会被立即改写。
 *
 * @param enabled 1：允许斜线；0：只允许水平/竖直执行段。
 */
void control_set_diagonal_path_enabled(uint8 enabled);

/**
 * @brief 获取当前斜线执行路径开关状态。
 *
 * @return uint8 1：允许斜线；0：只允许水平/竖直执行段。
 */
uint8 control_get_diagonal_path_enabled(void);

/**
 * @brief 设置初始定位之后是否继续使用视觉定位修正。
 *
 * 起步后的第一次视觉定位始终启用；该开关只影响识别结束后的二次定位，
 * 以及等待地图阶段是否用新 CAR 位姿轻量同步 path_follow。
 *
 * @param enabled 1：后续继续使用视觉定位；0：后续依靠里程计/IMU。
 */
void control_set_followup_vision_localization_enabled(uint8 enabled);

/**
 * @brief 获取后续视觉定位修正开关状态。
 *
 * @return uint8 1：后续继续使用视觉定位；0：后续依靠里程计/IMU。
 */
uint8 control_get_followup_vision_localization_enabled(void);

/**
 * @brief 设置识别阶段是否启用段前提前转向。
 *
 * 开启后，每段识别短路程出发前会先转到识别朝向，到点后直接识别；
 * 关闭后，到达识别点后再原地转向识别。
 *
 * @param enabled 1：启用；0：关闭。
 */
void control_set_identify_prerotate_enabled(uint8 enabled);

/**
 * @brief 获取识别阶段段前提前转向开关状态。
 *
 * @return uint8 1：启用；0：关闭。
 */
uint8 control_get_identify_prerotate_enabled(void);

/**
 * @brief 设置是否启用三轮完整流程模式。
 *
 * 开启后，一轮推箱完成并返回左侧发车区、车头回正后，会再次发车开始新流程，
 * 总共执行 3 次“发车->识别->推箱->返场”。
 *
 * @param enabled 1：启用三轮流程；0：单轮流程。
 */
void control_set_repeat_three_flow_enabled(uint8 enabled);

/**
 * @brief 获取三轮完整流程模式开关状态。
 *
 * @return uint8 1：启用三轮流程；0：单轮流程。
 */
uint8 control_get_repeat_three_flow_enabled(void);

/**
 * @brief 设置控制流程使用的路径规划模式。
 *
 * @param mode 规划模式标志位：
 * - CONTROL_PLAN_MODE_1：使用 Plan_path_Mode1()
 * - CONTROL_PLAN_MODE_2：使用 Plan_path_Mode2()
 */
void control_set_plan_mode(control_plan_mode_t mode);

/**
 * @brief 获取当前路径规划模式标志位。
 *
 * @return control_plan_mode_t 当前规划模式。
 */
control_plan_mode_t control_get_plan_mode(void);

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
