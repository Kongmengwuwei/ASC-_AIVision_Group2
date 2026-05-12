#include "path_follow.h"
#include "motor.h"
#include "pid.h"
#include "zf_device_ips200.h"
#include <math.h>
#include "Attitude.h"


/**
 * @file path_follow.c
 * @brief 路径跟随与底盘速度规划实现文件。
 *
 * 本文件负责将上层给出的路径点或拐点序列转换为底盘可执行的连续速度命令，
 * 控制过程依次经过路径几何分析、标量速度规划、姿态控制和车体系速度合成。
 * 当前实现兼容网格地图产生的横平竖直路径，并保留启动偏移、位姿校正等辅助动作接口。
 */


#ifndef M_PI
#define M_PI 3.1415926
#endif

/** @brief 接近目标点时开始混合定点保持修正的距离阈值，单位 m。 */
#define PATH_HOLD_TRIM_RELEASE_DISTANCE 0.25f
/** @brief 直线段法向纠偏比例系数，单位 (cm/s)/cm。 */
#define PATH_LINE_GUIDE_KP 1.0f
/**
 * @brief 直线段法向纠偏的最小修正速度，单位 cm/s。
 *
 * 作用：
 * - 当横向误差刚刚超过死区时，单纯 Kp 可能不足以克服静摩擦；
 * - 这里额外给一个“最小侧向修正速度前馈”，帮助小车先动起来。
 *
 * 调参建议：
 * - 设为 0 可退回到原来的纯 Kp 纠偏；
 * - 若仍推不动，可逐步增大；
 * - 若出现明显来回抽动，可适当减小。
 */
#define PATH_LINE_GUIDE_MIN_CMPS 4.0f
/** @brief 直线段法向纠偏速度限幅，单位 cm/s。 */
#define PATH_LINE_GUIDE_MAX_CMPS 12.0f
/** @brief 直线段法向纠偏死区，单位 m。 */
#define PATH_LINE_GUIDE_DEADBAND_M 0.005f
/** @brief 是否启用直线段法向纠偏：1 启用，0 关闭。 */
#define PATH_FOLLOW_ENABLE_LINE_GUIDE 1

/** @brief 非终点路径段在段末保	留的过渡速度，单位 cm/s。 */
#define SCURVE_SEGMENT_END_SPEED_CMPS 30.0f
/** @brief S 曲线 band 0 的距离上限，单位 m。 */
#define PATH_FOLLOW_SCURVE_BAND0_UPPER_M 0.30f
/** @brief S 曲线 band 1 的距离上限，单位 m。 */
#define PATH_FOLLOW_SCURVE_BAND1_UPPER_M 0.50f
/** @brief S 曲线 band 2 的距离上限，单位 m。 */
#define PATH_FOLLOW_SCURVE_BAND2_UPPER_M 0.70f
/** @brief S 曲线 band 3 的距离上限，单位 m。 */
#define PATH_FOLLOW_SCURVE_BAND3_UPPER_M 0.90f
/** @brief S 曲线最后一个 band 的兜底距离上限，单位 m。 */
#define PATH_FOLLOW_SCURVE_LAST_UPPER_M 1000.0f
/** @brief 当前未锁定任何 band 时的无效索引。 */
#define PATH_FOLLOW_INVALID_SCURVE_BAND_IDX 0xFFU
/** @brief 启动偏移与定位修正临时路径的离散分辨率，单位 m。 */
#define PRESTART_OFFSET_RESOLUTION_M 0.01f
/** @brief 拐点附近是否启用双轴 PID 保持修正：1 启用，0 关闭。 */
#define PATH_FOLLOW_ENABLE_CORNER_DUAL_AXIS_TRIM 1
/** @brief 启用拐点 handover 时，拐点双轴保持修正的权重缩放系数。 */
#define PATH_FOLLOW_CORNER_HOLD_TRIM_SCALE 9.0f
/** @brief 普通中间拐点是否启用速度向量 handover：1 启用，0 回退到原始简单段末速度保留。 */
#define PATH_FOLLOW_ENABLE_CORNER_HANDOVER 1
#if PATH_FOLLOW_ENABLE_CORNER_HANDOVER


/* [CornerHandover重构] 保证 enter / commit 窗口之间至少留出的距离，单位 m。 */
#define PATH_CORNER_HANDOVER_WINDOW_GAP_MIN_M 0.005f
/* [CornerHandover重构] 普通中间拐点正式切段时的最小横向误差门限，单位 m。 */
#define PATH_CORNER_COMMIT_LATERAL_GATE_MIN_M 0.025f
/* [CornerHandover重构] 普通中间拐点正式切段时横向误差门限相对 pos_tol 的放大系数。 */
#define PATH_CORNER_COMMIT_LATERAL_GATE_SCALE 1.20f
/* [CornerHandover重构] 交接中段对标量速度做轻微压低，避免拐点切向速度过硬。 */
#define PATH_CORNER_HANDOVER_K_DROP 0.16f
/* [CornerHandover] 固定交接窗口的开始距离，单位 m。 */
#define PATH_CORNER_HANDOVER_ENTER_DISTANCE_M 0.09f
/* [CornerHandover] 固定交接窗口的末端距离，单位 m。 */
#define PATH_CORNER_HANDOVER_COMMIT_DISTANCE_M 0.015f
/* [CornerHandover] 单个拐点交接最多允许持续的时间，单位 s。 */
#define PATH_CORNER_HANDOVER_TIMEOUT_S 5U

#endif

/** @brief 路径跟随到点判定的基础位置容差，单位 m。 */
#define PATH_FOLLOW_POSITION_TOLERANCE_M PATH_CORNER_COMMIT_LATERAL_GATE_MIN_M
/** @brief 原地转向完成判定的航向容差，单位 deg。 */
#define PATH_FOLLOW_YAW_TOLERANCE_DEG 2.0f
/** @brief 路径跟随允许的线速度上限，单位 m/s。 */
#define PATH_FOLLOW_MAX_LINEAR_SPEED_MPS 3.0f
/**
 * @brief 当前实现中姿态环输出限幅使用的角速度上限数值。
 *
 * @note 该值直接沿用现有实现中的数值语义，只做宏抽取，不改变当前控制框架。
 */
#define PATH_FOLLOW_MAX_ANGULAR_SPEED_LIMIT 360.0f

/** @brief 世界系位置环 PID 比例系数。 */
#define PATH_FOLLOW_PID_WORLD_KP 2.2f
/** @brief 世界系位置环 PID 积分系数。 */
#define PATH_FOLLOW_PID_WORLD_KI 0.0f
/** @brief 世界系位置环 PID 微分系数。 */
#define PATH_FOLLOW_PID_WORLD_KD 1.8f
/** @brief 世界系位置环 D 项滤波系数。 */
#define PATH_FOLLOW_PID_WORLD_ALPHA 0.9f

/** @brief 近目标保持位置环 PID 比例系数。 */
#define PATH_FOLLOW_PID_STAY_KP 0.7f
/** @brief 近目标保持位置环 PID 积分系数。 */
#define PATH_FOLLOW_PID_STAY_KI 0.0f
/** @brief 近目标保持位置环 PID 微分系数。 */
#define PATH_FOLLOW_PID_STAY_KD 0.3f
/** @brief 近目标保持位置环积分限幅，单位 cm/s。 */
#define PATH_FOLLOW_PID_STAY_MAX_IOUT_CMPS 200.0f
/** @brief 近目标保持位置环输出限幅，单位 cm/s。 */
#define PATH_FOLLOW_PID_STAY_MAX_OUT_CMPS 200.0f
/** @brief 近目标保持位置环 D 项滤波系数。 */
#define PATH_FOLLOW_PID_STAY_ALPHA 0.9f

/** @brief 航向环 PID 比例系数。 */
#define PATH_FOLLOW_PID_YAW_KP 6.2f
/** @brief 航向环 PID 积分系数。 */
#define PATH_FOLLOW_PID_YAW_KI 0.0f
/** @brief 航向环 PID 微分系数。 */
#define PATH_FOLLOW_PID_YAW_KD 10.5f
/** @brief 航向环 D 项滤波系数。 */
#define PATH_FOLLOW_PID_YAW_ALPHA 0.9f

/** @brief 预留姿态扩展环 PID 比例系数。 */
#define PATH_FOLLOW_PID_ACCEL_YAW_KP 1.2f
/** @brief 预留姿态扩展环 PID 积分系数。 */
#define PATH_FOLLOW_PID_ACCEL_YAW_KI 0.0f
/** @brief 预留姿态扩展环 PID 微分系数。 */
#define PATH_FOLLOW_PID_ACCEL_YAW_KD 2.1f
/** @brief 预留姿态扩展环 D 项滤波系数。 */
#define PATH_FOLLOW_PID_ACCEL_YAW_ALPHA 0.9f

/**
 * @brief 世界坐标系下的二维位姿。
 */
typedef struct
{
    float x_m;      // 世界坐标系 X，单位 m
    float y_m;      // 世界坐标系 Y，单位 m
    float yaw_deg;  // 航向角，度
} pose2d_t;

/**
 * @brief 姿态控制模式枚举。
 */
typedef enum
{
    PATH_FOLLOW_HEADING_FIXED = 0
} path_follow_heading_mode_t;

/**
 * @brief 速度规划参数集合。
 */
typedef struct
{
    float segment_end_speed_cmps;   // 非终点路径段在段末保留的目标速度，单位 cm/s
    float hold_release_distance_m;  // 开始混合定点保持修正的距离阈值，单位 m
} path_follow_speed_cfg_t;

/**
 * @brief 当前路径段锁定的 S 曲线 band 运行参数。
 */
typedef struct
{
    uint8 band_idx;         // 当前路径段锁定的 band 索引
    float max_speed_cmps;   // 当前路径段锁定的最大速度，单位 cm/s
    float accel_cmpss;      // 当前路径段锁定的最大加速度，单位 cm/s^2
    float jerk_cmpsss;      // 当前路径段锁定的最大 jerk，单位 cm/s^3
} path_follow_scurve_runtime_cfg_t;

/**
 * @brief 路径几何层输出。
 */
typedef struct
{
    Position target_point; // 当前目标离散点
    float target_x_m;      // 当前目标点 X 坐标，单位 m
    float target_y_m;      // 当前目标点 Y 坐标，单位 m
    float delta_x_m;       // 目标相对当前位置的 X 方向误差，单位 m
    float delta_y_m;       // 目标相对当前位置的 Y 方向误差，单位 m
    float distance_m;      // 当前目标点欧氏距离，单位 m
    float dir_x;           // 指向目标的单位方向向量 X 分量
    float dir_y;           // 指向目标的单位方向向量 Y 分量
    uint8 segment_axis;    // 当前段主轴类型
} path_follow_geometry_t;

/**
 * @brief 速度规划层输出。
 */
typedef struct
{
    float safety_cap_cmps; // 仅用于异常保护的安全速度上限，单位 cm/s
    float end_speed_cmps;  // 当前段末端目标速度，单位 cm/s
    float ref_speed_cmps;  // 本周期最终标量速度参考，单位 cm/s
} path_follow_speed_plan_t;

/**
 * @brief 当前路径段的标准 7 段 jerk-limited S 曲线参数。
 *
 * @note 本结构描述的是“一条路径段一次性规划完成”的完整 profile，
 *       后续控制周期只按累计时间采样，不再滚动重建。
 */
typedef struct
{
    float s;      // 当前路径段总位移，单位 cm
    float x0;     // 轨迹起点位置，单位 cm，当前实现固定为 0
    float x1;     // 轨迹终点位置，单位 cm
    float v0;     // 段起点速度，单位 cm/s
    float v1;     // 段终点速度，单位 cm/s
    float vmax;   // 配置给定的最大速度约束，单位 cm/s
    float amax;   // 本条轨迹最终采用的最大加速度，单位 cm/s^2
    float jmax;   // 本条轨迹采用的最大 jerk，单位 cm/s^3
    float vlim;   // 本条轨迹实际可达到的峰值速度，单位 cm/s
    float alima;  // 加速侧实际峰值加速度，单位 cm/s^2
    float alimd;  // 减速侧实际峰值减速度幅值，单位 cm/s^2
    float Tj1;    // 加速侧 jerk 作用时间，单位 s
    float Ta;     // 整个加速阶段时长，单位 s
    float Tv;     // 匀速阶段时长，单位 s
    float Tj2;    // 减速侧 jerk 作用时间，单位 s
    float Td;     // 整个减速阶段时长，单位 s
    float T;      // 轨迹总时长，单位 s
    uint8 valid;
} path_follow_scurve_profile_t;

/**
 * @brief 姿态层输出。
 */
typedef struct
{
    float target_yaw_deg;   // 当前姿态层目标航向，单位 deg
    float omega_cmd_radps;  // 当前姿态层输出角速度，单位 rad/s
} path_follow_attitude_plan_t;

/**
 * @brief 车体运动命令缓存。
 */
typedef struct
{
    float vx_world_cmps;  // 世界系 X 方向速度命令，单位 cm/s
    float vy_world_cmps;  // 世界系 Y 方向速度命令，单位 cm/s
    float vx_body_cmps;   // 车体系 X 方向速度命令，单位 cm/s
    float vy_body_cmps;   // 车体系 Y 方向速度命令，单位 cm/s
} path_follow_motion_cmd_t;

/**
 * @brief 调试状态快照。
 */
typedef struct
{
    float distance_m;       // 当前目标距离，单位 m
    float dir_x;            // 当前单位方向向量 X 分量
    float dir_y;            // 当前单位方向向量 Y 分量
    float speed_ref_cmps;   // 当前标量速度参考，单位 cm/s
    float target_yaw_deg;   // 当前目标航向，单位 deg
    float vx_world_cmps;    // 当前世界系 X 速度命令，单位 cm/s
    float vy_world_cmps;    // 当前世界系 Y 速度命令，单位 cm/s
    uint8 segment_axis;     // 当前路径段主轴类型
} path_follow_debug_state_t;

/**
 * @brief 路径跟随模块运行上下文。
 */
typedef struct
{
    const Position *path;  // 规划路径指针
    size_t steps;          // 路径长度
    size_t idx;            // 当前目标点索引
    float grid_m;          // 网格边长，m/格
    float default_grid_m;  // 地图路径网格边长，m/格
    float pulses_per_meter;// 编码器每米脉冲数
    float pos_tol_m;       // 位置容差
    float yaw_tol_deg;     // 航向容差（保留，当前未用）
    float max_v_mps;       // 线速度上限 m/s
    float max_w_rad;       // 角速度上限 rad/s
    float target_yaw_deg;  // 当前姿态控制目标
    uint8 heading_mode;    // 当前姿态控制模式
    pose2d_t pose;         // 当前里程计位姿
    path_follow_speed_cfg_t speed_cfg;          // 速度规划参数集合
    path_follow_scurve_runtime_cfg_t active_scurve_cfg; // 当前路径段锁定的 S 曲线参数
    path_follow_scurve_profile_t active_profile;// 当前目标段的整段式 S 曲线 profile
    path_follow_debug_state_t debug;            // 调试状态快照
    float profile_time_s;                       // 当前 profile 已执行的累计时间
    float last_ref_speed_cmps;                 // 上一周期输出的标量速度参考
    size_t profile_target_idx;                 // 当前 profile 绑定的目标点索引
    size_t pause_indices[PATH_FOLLOW_MAX_PAUSE_POINTS]; // 需要在到点后暂停的 corner 索引列表
    size_t pause_count;                        // pause_indices 中的有效元素个数
    size_t pause_cursor;                       // 下一个待触发的暂停点游标
    uint32 pause_cycles_cfg;                   // 每次暂停统一持续的控制周期数
    uint32 pause_cycles_remaining;             // 当前暂停剩余控制周期数
    uint8 pause_events_enabled;                // 仅普通地图 corner_path 允许触发暂停
    uint8 paused;                              // 1: 正处于暂停窗口
    uint8 rotate_only_active;                  // 1: 当前仅执行原地转向
    uint8 profile_active;                      // 1: 当前段已有有效 profile
#if PATH_FOLLOW_ENABLE_CORNER_HANDOVER
    uint8 corner_handover_active;              // 1: 当前普通中间拐点已进入 handover 区
    size_t corner_handover_idx;                // 当前 handover 对应的 target idx
    float corner_enter_distance_m;             // 当前拐点进入 handover 的距离阈值，单位 m
    float corner_commit_distance_m;            // 当前拐点允许正式切段的提前量，单位 m
#endif
    uint8 active;                              // 1: 正在跟随
} path_follow_ctx_t;

/** @brief 路径跟随模块全局上下文。 */
static path_follow_ctx_t g_ctx = {0};
/** @brief 默认的 S 曲线 band 配置表。 */
static const path_follow_scurve_band_cfg_t g_path_follow_scurve_band_default_cfg[PATH_FOLLOW_SCURVE_BAND_COUNT] = {
    {PATH_FOLLOW_SCURVE_BAND0_UPPER_M, 0.40f, 0.60f, 3.00f},
    {PATH_FOLLOW_SCURVE_BAND1_UPPER_M, 0.50f, 0.70f, 3.50f},
    {PATH_FOLLOW_SCURVE_BAND2_UPPER_M, 0.65f, 0.80f, 4.00f},
    {PATH_FOLLOW_SCURVE_BAND3_UPPER_M, 0.75f, 0.85f, 4.70f},
    {PATH_FOLLOW_SCURVE_LAST_UPPER_M, 1.50f, 0.95f, 5.50f},
};
/** @brief 对外可调的 S 曲线 band 配置表。 */
path_follow_scurve_band_cfg_t g_path_follow_scurve_band_cfg[PATH_FOLLOW_SCURVE_BAND_COUNT] = {
    {PATH_FOLLOW_SCURVE_BAND0_UPPER_M, 0.40f, 0.60f, 3.00f},
    {PATH_FOLLOW_SCURVE_BAND1_UPPER_M, 0.50f, 0.70f, 3.50f},
    {PATH_FOLLOW_SCURVE_BAND2_UPPER_M, 0.65f, 0.80f, 4.00f},
    {PATH_FOLLOW_SCURVE_BAND3_UPPER_M, 0.75f, 0.85f, 4.70f},
    {PATH_FOLLOW_SCURVE_LAST_UPPER_M, 1.50f, 0.95f, 5.50f},
};
/** @brief 历史 X 轴位置环 PID，当前主要保留参数与兼容接口。 */
tagPID_T pid_world_x;
/** @brief 历史 Y 轴位置环 PID，当前主要保留参数与兼容接口。 */
tagPID_T pid_world_y;
/** @brief 近目标保持使用的 X 轴 PID。 */
tagPID_T pid_stay;
/** @brief 近目标保持使用的 Y 轴 PID。 */
tagPID_T pid_stay_y;
/** @brief 航向控制 PID。 */
tagPID_T pid_yaw;
/** @brief 预留的姿态扩展 PID。 */
tagPID_T pid_accel_yaw;

/** @brief X/Y 位置环初始化参数。 */
static PIDInitStruct pid_world_init;
/** @brief 定点保持 PID 初始化参数。 */
static PIDInitStruct pid_stay_init;
/** @brief 航向 PID 初始化参数。 */
static PIDInitStruct pid_yaw_init;
/** @brief 预留姿态扩展 PID 初始化参数。 */
static PIDInitStruct pid_accel_yaw_init;
/** @brief 当前路径段主轴标记：0 停止，1 X 段，2 Y 段，3 斜段。 */
uint8 car_direction = 0;
/** @brief 外层保留的停车等待标志。 */
uint8 wait_stop = 0;
/** @brief 发车前左移偏置量，单位 m。 */
float prestart_move_left_m = 0.12f;
/** @brief 发车前右移偏置量，单位 m。 */
float prestart_move_right_m = 0.00f;
/** @brief 发车前前移偏置量，单位 m。 */
float prestart_move_forward_m = 0.35f;
/** @brief 发车前后移偏置量，单位 m。 */
float prestart_move_backward_m = 0.00f;
/** @brief 单目标模式使用的两点临时路径缓存。 */
static Position g_single_target_path[2];
/** @brief 发车前偏移动作使用的临时路径缓存。 */
static Position g_prestart_offset_path[3];
/** @brief 位姿修正动作使用的临时路径缓存。 */
static Position g_pose_correction_path[3];
/** @brief 蓝牙位置上报挂起标志，由 10ms 控制节拍置位、主循环消费。 */
static volatile uint8 g_path_follow_bt_report_pending = 0U;

/**
 * @brief 对输入值做对称限幅。
 *
 * @param v 待限幅输入值。
 * @param limit 正向与负向共用的限幅绝对值。
 * @return 限幅后的结果。
 */
static float clamp_sym(float v, float limit)
{
    if (v > limit)
    {
        return limit;
    }
    if (v < -limit)
    {
        return -limit;
    }
    return v;
}

/**
 * @brief 将弧度归一化到 (-pi, pi] 区间。
 *
 * @param rad 原始弧度值。
 * @return 归一化后的弧度值。
 */
static float wrap_pi(float rad)
{
    while (rad > (float)M_PI)
    {
        rad -= 2.0f * (float)M_PI;
    }
    while (rad < -(float)M_PI)
    {
        rad += 2.0f * (float)M_PI;
    }
    return rad;
}

static float path_follow_wrap_deg(float deg)
{
    while (deg > 180.0f)
    {
        deg -= 360.0f;
    }
    while (deg < -180.0f)
    {
        deg += 360.0f;
    }
    return deg;
}

static float path_follow_yaw_error_deg(float current_yaw_deg, float target_yaw_deg)
{
    return path_follow_wrap_deg(target_yaw_deg - current_yaw_deg);
}

/**
 * @brief 将 m/s 转成 cm/s。
 */
static float path_follow_mps_to_cmps(float value_mps)
{
    return value_mps * 100.0f;
}

/**
 * @brief 将 m/s^2 转成 cm/s^2。
 */
static float path_follow_mps2_to_cmpss(float value_mps2)
{
    return value_mps2 * 100.0f;
}

/**
 * @brief 将 m/s^3 转成 cm/s^3。
 */
static float path_follow_mps3_to_cmpsss(float value_mps3)
{
    return value_mps3 * 100.0f;
}

/**
 * @brief 复位 S 曲线 band 配置到默认值。
 */
void path_follow_reset_scurve_band_defaults(void)
{
    uint8 i;

    for (i = 0U; i < PATH_FOLLOW_SCURVE_BAND_COUNT; ++i)
    {
        g_path_follow_scurve_band_cfg[i] = g_path_follow_scurve_band_default_cfg[i];
    }
}

/**
 * @brief 校正对外可调的 S 曲线 band 配置，非法值回退到默认值。
 */
void path_follow_sanitize_scurve_band_cfg(void)
{
    uint8 i;

    for (i = 0U; i < PATH_FOLLOW_SCURVE_BAND_COUNT; ++i)
    {
        g_path_follow_scurve_band_cfg[i].distance_upper_m =
            g_path_follow_scurve_band_default_cfg[i].distance_upper_m;

        if (!(g_path_follow_scurve_band_cfg[i].vmax_mps > 0.0f))
        {
            g_path_follow_scurve_band_cfg[i].vmax_mps =
                g_path_follow_scurve_band_default_cfg[i].vmax_mps;
        }
        if (!(g_path_follow_scurve_band_cfg[i].amax_mps2 > 0.0f))
        {
            g_path_follow_scurve_band_cfg[i].amax_mps2 =
                g_path_follow_scurve_band_default_cfg[i].amax_mps2;
        }
        if (!(g_path_follow_scurve_band_cfg[i].jmax_mps3 > 0.0f))
        {
            g_path_follow_scurve_band_cfg[i].jmax_mps3 =
                g_path_follow_scurve_band_default_cfg[i].jmax_mps3;
        }
    }
}

/**
 * @brief 复位当前路径段锁定的 S 曲线 band 参数。
 */
static void path_follow_reset_active_scurve_cfg(void)
{
    g_ctx.active_scurve_cfg.band_idx = PATH_FOLLOW_INVALID_SCURVE_BAND_IDX;
    g_ctx.active_scurve_cfg.max_speed_cmps = 0.0f;
    g_ctx.active_scurve_cfg.accel_cmpss = 0.0f;
    g_ctx.active_scurve_cfg.jerk_cmpsss = 0.0f;
}

/**
 * @brief 根据段长选择当前路径段应锁定的 S 曲线 band。
 *
 * @param distance_m 当前路径段总长度，单位 m。
 * @param runtime_cfg 选中的运行参数输出。
 */
static void path_follow_select_scurve_band(float distance_m,
                                           path_follow_scurve_runtime_cfg_t *runtime_cfg)
{
    uint8 i;

    if (runtime_cfg == NULL)
    {
        return;
    }

    path_follow_sanitize_scurve_band_cfg();
    distance_m = fmaxf(distance_m, 0.0f);

    for (i = 0U; i < PATH_FOLLOW_SCURVE_BAND_COUNT; ++i)
    {
        if (distance_m <= g_path_follow_scurve_band_cfg[i].distance_upper_m)
        {
            runtime_cfg->band_idx = i;
            runtime_cfg->max_speed_cmps = path_follow_mps_to_cmps(g_path_follow_scurve_band_cfg[i].vmax_mps);
            runtime_cfg->accel_cmpss = path_follow_mps2_to_cmpss(g_path_follow_scurve_band_cfg[i].amax_mps2);
            runtime_cfg->jerk_cmpsss = path_follow_mps3_to_cmpsss(g_path_follow_scurve_band_cfg[i].jmax_mps3);
            return;
        }
    }

    runtime_cfg->band_idx = PATH_FOLLOW_SCURVE_BAND_COUNT - 1U;
    runtime_cfg->max_speed_cmps = path_follow_mps_to_cmps(g_path_follow_scurve_band_cfg[PATH_FOLLOW_SCURVE_BAND_COUNT - 1U].vmax_mps);
    runtime_cfg->accel_cmpss = path_follow_mps2_to_cmpss(g_path_follow_scurve_band_cfg[PATH_FOLLOW_SCURVE_BAND_COUNT - 1U].amax_mps2);
    runtime_cfg->jerk_cmpsss = path_follow_mps3_to_cmpsss(g_path_follow_scurve_band_cfg[PATH_FOLLOW_SCURVE_BAND_COUNT - 1U].jmax_mps3);
}

/**
 * @brief 根据当前上下文中的最大速度/角速度更新 PID 输出限幅。
 *
 * @note 当前主链路未主动调用该函数，但保留它用于后续在线调参或恢复旧控制器时复用。
 */
static void path_follow_update_pid_limits(void)
{
    // 按 max_v/max_w 更新 PID 限幅
    pid_world_init.fMax_Iout = g_ctx.max_v_mps * 100.0f;
    pid_world_init.fMax_Out = g_ctx.max_v_mps * 100.0f;
    PID_Update(&pid_world_x, &pid_world_init);
    PID_Update(&pid_world_y, &pid_world_init);

    pid_yaw_init.fMax_Iout = g_ctx.max_w_rad * 20.0f;
    pid_yaw_init.fMax_Out = g_ctx.max_w_rad * 20.0f;
    PID_Update(&pid_yaw, &pid_yaw_init);
}

/**
 * @brief 为单条路径段一次性构造标准 7 段 jerk-limited S 曲线 profile。
 *
 * @param profile 输出 profile。
 * @param distance_cm 当前路径段总位移，单位 cm。
 * @param v0_cmps 起始速度，单位 cm/s。
 * @param v1_cmps 末端速度，单位 cm/s。
 * @param v_max_cmps 最大速度约束，单位 cm/s。
 * @param a_max_cmpss 最大加速度约束，单位 cm/s^2。
 * @param j_max_cmpsss 最大 jerk 约束，单位 cm/s^3。
 * @return `1` 表示构造成功，`0` 表示在给定边界条件和约束下当前段不可行。
 */
static uint8 path_follow_build_scurve_profile(path_follow_scurve_profile_t *profile,
                                              float distance_cm,
                                              float v0_cmps,
                                              float v1_cmps,
                                              float v_max_cmps,
                                              float a_max_cmpss,
                                              float j_max_cmpsss)
{
    float T1;
    float T2;
    float Tjs;
    float Tj1;
    float Tj2;
    float Ta;
    float Td;
    float Tv;
    float Tj;
    float delta;
    float a_work;

    if (profile == NULL)
    {
        return 0U;
    }

    *profile = (path_follow_scurve_profile_t){0};

    if (distance_cm <= 0.0f ||
        v_max_cmps <= 0.0f ||
        a_max_cmpss <= 0.0f ||
        j_max_cmpsss <= 0.0f)
    {
        return 0U;
    }

    v0_cmps = fminf(fmaxf(v0_cmps, 0.0f), v_max_cmps);
    v1_cmps = fminf(fmaxf(v1_cmps, 0.0f), v_max_cmps);

    T1 = sqrtf(fabsf(v1_cmps - v0_cmps) / j_max_cmpsss);
    T2 = v_max_cmps / j_max_cmpsss;
    Tjs = fminf(T1, T2);

    if ((T1 <= T2 && distance_cm < (Tjs * (v0_cmps + v1_cmps))) ||
        (T1 > T2 && distance_cm < (0.5f * (v0_cmps + v1_cmps) *
                                   (Tjs + fabsf(v1_cmps - v0_cmps) / a_max_cmpss))))
    {
        return 0U;
    }

    if ((v_max_cmps - v0_cmps) * j_max_cmpsss < (a_max_cmpss * a_max_cmpss))
    {
        Tj1 = sqrtf(fmaxf(v_max_cmps - v0_cmps, 0.0f) / j_max_cmpsss);
        Ta = 2.0f * Tj1;
        profile->alima = j_max_cmpsss * Tj1;
    }
    else
    {
        Tj1 = a_max_cmpss / j_max_cmpsss;
        Ta = Tj1 + (v_max_cmps - v0_cmps) / a_max_cmpss;
        profile->alima = a_max_cmpss;
    }

    if ((v_max_cmps - v1_cmps) * j_max_cmpsss < (a_max_cmpss * a_max_cmpss))
    {
        Tj2 = sqrtf(fmaxf(v_max_cmps - v1_cmps, 0.0f) / j_max_cmpsss);
        Td = 2.0f * Tj2;
        profile->alimd = j_max_cmpsss * Tj2;
    }
    else
    {
        Tj2 = a_max_cmpss / j_max_cmpsss;
        Td = Tj2 + (v_max_cmps - v1_cmps) / a_max_cmpss;
        profile->alimd = a_max_cmpss;
    }

    Tv = distance_cm / v_max_cmps -
         Ta * 0.5f * (1.0f + v0_cmps / v_max_cmps) -
         Td * 0.5f * (1.0f + v1_cmps / v_max_cmps);
    if (Tv > 0.0f)
    {
        profile->s = distance_cm;
        profile->x0 = 0.0f;
        profile->x1 = distance_cm;
        profile->v0 = v0_cmps;
        profile->v1 = v1_cmps;
        profile->vmax = v_max_cmps;
        profile->amax = a_max_cmpss;
        profile->jmax = j_max_cmpsss;
        profile->vlim = v_max_cmps;
        profile->Tj1 = Tj1;
        profile->Tj2 = Tj2;
        profile->Ta = Ta;
        profile->Td = Td;
        profile->Tv = Tv;
        profile->T = Ta + Tv + Td;
        profile->valid = 1U;
        return 1U;
    }

    Tv = 0.0f;
    Tj = a_max_cmpss / j_max_cmpsss;
    Tj1 = Tj;
    Tj2 = Tj;
    delta = powf(a_max_cmpss, 4) / powf(j_max_cmpsss, 2) +
            2.0f * (v0_cmps * v0_cmps + v1_cmps * v1_cmps) +
            a_max_cmpss * (4.0f * distance_cm -
                           2.0f * a_max_cmpss / j_max_cmpsss * (v0_cmps + v1_cmps));
    delta = fmaxf(delta, 0.0f);
    Ta = (powf(a_max_cmpss, 2) / j_max_cmpsss - 2.0f * v0_cmps + sqrtf(delta)) /
         (2.0f * a_max_cmpss);
    Td = (powf(a_max_cmpss, 2) / j_max_cmpsss - 2.0f * v1_cmps + sqrtf(delta)) /
         (2.0f * a_max_cmpss);
    if (Ta > 2.0f * Tj && Td > 2.0f * Tj)
    {
        profile->s = distance_cm;
        profile->x0 = 0.0f;
        profile->x1 = distance_cm;
        profile->v0 = v0_cmps;
        profile->v1 = v1_cmps;
        profile->vmax = v_max_cmps;
        profile->amax = a_max_cmpss;
        profile->jmax = j_max_cmpsss;
        profile->Tj1 = Tj1;
        profile->Tj2 = Tj2;
        profile->Ta = Ta;
        profile->Td = Td;
        profile->Tv = Tv;
        profile->T = Ta + Td;
        profile->alima = a_max_cmpss;
        profile->alimd = a_max_cmpss;
        profile->vlim = v0_cmps + (Ta - Tj1) * profile->alima;
        profile->valid = 1U;
        return 1U;
    }

    a_work = a_max_cmpss;
    while (Ta < 2.0f * Tj || Td < 2.0f * Tj)
    {
        if (Ta > 0.0f && Td > 0.0f)
        {
            a_work *= 0.99f;
            if (a_work <= 0.0f)
            {
                return 0U;
            }

            Tj = a_work / j_max_cmpsss;
            Tj1 = Tj;
            Tj2 = Tj;
            delta = powf(a_work, 4) / powf(j_max_cmpsss, 2) +
                    2.0f * (v0_cmps * v0_cmps + v1_cmps * v1_cmps) +
                    a_work * (4.0f * distance_cm -
                              2.0f * a_work / j_max_cmpsss * (v0_cmps + v1_cmps));
            delta = fmaxf(delta, 0.0f);
            Ta = (powf(a_work, 2) / j_max_cmpsss - 2.0f * v0_cmps + sqrtf(delta)) /
                 (2.0f * a_work);
            Td = (powf(a_work, 2) / j_max_cmpsss - 2.0f * v1_cmps + sqrtf(delta)) /
                 (2.0f * a_work);
            continue;
        }

        if ((v0_cmps + v1_cmps) <= 0.0f)
        {
            return 0U;
        }

        if (Ta <= 0.0f)
        {
            float numer;
            float denom;
            Ta = 0.0f;
            Tj1 = 0.0f;
            Td = 2.0f * distance_cm / (v0_cmps + v1_cmps);
            numer = j_max_cmpsss * distance_cm -
                    sqrtf(fmaxf(j_max_cmpsss * (j_max_cmpsss * distance_cm * distance_cm +
                                                (v1_cmps + v0_cmps) * (v1_cmps + v0_cmps) *
                                                (v1_cmps - v0_cmps)),
                                0.0f));
            denom = j_max_cmpsss * (v1_cmps + v0_cmps);
            if (fabsf(denom) <= 1e-6f)
            {
                return 0U;
            }
            Tj2 = numer / denom;
        }
        else
        {
            float numer;
            float denom;
            Td = 0.0f;
            Tj2 = 0.0f;
            Ta = 2.0f * distance_cm / (v0_cmps + v1_cmps);
            numer = j_max_cmpsss * distance_cm -
                    sqrtf(fmaxf(j_max_cmpsss * (j_max_cmpsss * distance_cm * distance_cm -
                                                (v1_cmps + v0_cmps) * (v1_cmps + v0_cmps) *
                                                (v1_cmps - v0_cmps)),
                                0.0f));
            denom = j_max_cmpsss * (v1_cmps + v0_cmps);
            if (fabsf(denom) <= 1e-6f)
            {
                return 0U;
            }
            Tj1 = numer / denom;
        }

        profile->s = distance_cm;
        profile->x0 = 0.0f;
        profile->x1 = distance_cm;
        profile->v0 = v0_cmps;
        profile->v1 = v1_cmps;
        profile->vmax = v_max_cmps;
        profile->amax = a_work;
        profile->jmax = j_max_cmpsss;
        profile->Tj1 = Tj1;
        profile->Tj2 = Tj2;
        profile->Ta = Ta;
        profile->Td = Td;
        profile->Tv = 0.0f;
        profile->T = Ta + Td;
        profile->alima = j_max_cmpsss * Tj1;
        profile->alimd = j_max_cmpsss * Tj2;
        profile->vlim = v0_cmps + (Ta - Tj1) * profile->alima;
        profile->valid = 1U;
        return 1U;
    }

    profile->s = distance_cm;
    profile->x0 = 0.0f;
    profile->x1 = distance_cm;
    profile->v0 = v0_cmps;
    profile->v1 = v1_cmps;
    profile->vmax = v_max_cmps;
    profile->amax = a_work;
    profile->jmax = j_max_cmpsss;
    profile->Tj1 = Tj1;
    profile->Tj2 = Tj2;
    profile->Ta = Ta;
    profile->Td = Td;
    profile->Tv = 0.0f;
    profile->T = Ta + Td;
    profile->alima = j_max_cmpsss * Tj1;
    profile->alimd = j_max_cmpsss * Tj2;
    profile->vlim = v0_cmps + (Ta - Tj1) * profile->alima;
    profile->valid = 1U;
    return 1U;
}

static uint8 path_follow_target_requires_pause(size_t target_idx);
static uint8 path_follow_is_route_corner_target(void);
#if PATH_FOLLOW_ENABLE_CORNER_HANDOVER || PATH_FOLLOW_ENABLE_LINE_GUIDE
static uint8 path_follow_get_segment_points(const Position **prev_point,
                                            const Position **curr_point,
                                            const Position **next_point);
static uint8 path_follow_compute_segment_progress_m(const pose2d_t *pose,
                                                    const Position *seg_start,
                                                    const Position *seg_end,
                                                    float *progress_m,
                                                    float *segment_length_m,
                                                    float *normal_error_m);
#endif
#if PATH_FOLLOW_ENABLE_CORNER_HANDOVER
static void path_follow_reset_corner_handover_state(void);
#endif
#if PATH_FOLLOW_ENABLE_LINE_GUIDE
static uint8 path_follow_apply_line_guidance(float ref_speed_cmps,
                                             float *vx_world_cmps,
                                             float *vy_world_cmps);
#endif

/**
 * @brief 按当前路径段的累计执行时间采样标准 S 曲线目标速度。
 *
 * @param t_s 从当前 profile 起点开始累计的全局时间，单位 s。
 * @param profile 由 `path_follow_build_scurve_profile()` 构造的 profile。
 * @return 当前采样时刻的目标速度，单位 cm/s。
 */
static float path_follow_sample_scurve_velocity(float t_s,
                                                const path_follow_scurve_profile_t *profile)
{
    if (profile == NULL || !profile->valid)
    {
        return 0.0f;
    }
    if (t_s <= 0.0f)
    {
        return profile->v0;
    }
    if (t_s >= profile->T)
    {
        return profile->v1;
    }

    if (t_s < profile->Tj1)
    {
        return profile->v0 + profile->jmax * t_s * t_s * 0.5f;
    }
    if (t_s < (profile->Ta - profile->Tj1))
    {
        return profile->v0 + profile->alima * (t_s - profile->Tj1 * 0.5f);
    }
    if (t_s < profile->Ta)
    {
        float dt = profile->Ta - t_s;
        return profile->vlim - profile->jmax * dt * dt * 0.5f;
    }
    if (t_s < (profile->Ta + profile->Tv))
    {
        return profile->vlim;
    }
    if (t_s < (profile->T - profile->Td + profile->Tj2))
    {
        float dt = t_s - profile->T + profile->Td;
        return profile->vlim - profile->jmax * dt * dt * 0.5f;
    }
    if (t_s < (profile->T - profile->Tj2))
    {
        return profile->vlim - profile->alimd *
               (t_s - profile->T + profile->Td - profile->Tj2 * 0.5f);
    }

    {
        float dt = profile->T - t_s;
        return profile->v1 + profile->jmax * dt * dt * 0.5f;
    }
}

#if PATH_FOLLOW_ENABLE_CORNER_HANDOVER || PATH_FOLLOW_ENABLE_LINE_GUIDE
/**
 * @brief 获取当前目标相关的前一角点、当前角点和后一角点。
 *
 * @note 若某一侧不存在，对应输出会被置为 NULL。
 *
 * @param prev_point 前一角点输出。
 * @param curr_point 当前目标角点输出。
 * @param next_point 后一角点输出。
 * @return `1` 表示当前至少存在有效 curr_point，`0` 表示当前路径上下文无效。
 */
static uint8 path_follow_get_segment_points(const Position **prev_point,
                                            const Position **curr_point,
                                            const Position **next_point)
{
    if (prev_point != NULL)
    {
        *prev_point = NULL;
    }
    if (curr_point != NULL)
    {
        *curr_point = NULL;
    }
    if (next_point != NULL)
    {
        *next_point = NULL;
    }

    if (g_ctx.path == NULL || g_ctx.idx >= g_ctx.steps)
    {
        return 0U;
    }

    if (prev_point != NULL && g_ctx.idx > 0U)
    {
        *prev_point = &g_ctx.path[g_ctx.idx - 1U];
    }
    if (curr_point != NULL)
    {
        *curr_point = &g_ctx.path[g_ctx.idx];
    }
    if (next_point != NULL && (g_ctx.idx + 1U) < g_ctx.steps)
    {
        *next_point = &g_ctx.path[g_ctx.idx + 1U];
    }

    return 1U;
}

/**
 * @brief 计算当前位置在 prev->curr 段上的投影进度、段长和横向误差。
 *
 * @param pose 当前位姿。
 * @param seg_start 当前段起点 prev。
 * @param seg_end 当前段终点 curr。
 * @param progress_m 输出当前投影进度 s，单位 m。
 * @param segment_length_m 输出当前段长度 L，单位 m。
 * @param normal_error_m 输出当前横向误差 e_n，单位 m。
 * @return `1` 表示计算成功，`0` 表示输入非法或段长退化。
 */
static uint8 path_follow_compute_segment_progress_m(const pose2d_t *pose,
                                                    const Position *seg_start,
                                                    const Position *seg_end,
                                                    float *progress_m,
                                                    float *segment_length_m,
                                                    float *normal_error_m)
{
    float start_x_m;
    float start_y_m;
    float end_x_m;
    float end_y_m;
    float seg_dx_m;
    float seg_dy_m;
    float seg_norm_m;
    float seg_dir_x;
    float seg_dir_y;
    float rel_x_m;
    float rel_y_m;

    if (progress_m != NULL)
    {
        *progress_m = 0.0f;
    }
    if (segment_length_m != NULL)
    {
        *segment_length_m = 0.0f;
    }
    if (normal_error_m != NULL)
    {
        *normal_error_m = 0.0f;
    }

    if (pose == NULL || seg_start == NULL || seg_end == NULL ||
        progress_m == NULL || segment_length_m == NULL || normal_error_m == NULL)
    {
        return 0U;
    }

    start_x_m = seg_start->row * g_ctx.grid_m;
    start_y_m = seg_start->col * g_ctx.grid_m;
    end_x_m = seg_end->row * g_ctx.grid_m;
    end_y_m = seg_end->col * g_ctx.grid_m;
    seg_dx_m = end_x_m - start_x_m;
    seg_dy_m = end_y_m - start_y_m;
    seg_norm_m = sqrtf(seg_dx_m * seg_dx_m + seg_dy_m * seg_dy_m);
    if (seg_norm_m <= 1e-6f)
    {
        return 0U;
    }

    seg_dir_x = seg_dx_m / seg_norm_m;
    seg_dir_y = seg_dy_m / seg_norm_m;
    rel_x_m = pose->x_m - start_x_m;
    rel_y_m = pose->y_m - start_y_m;

    *progress_m = rel_x_m * seg_dir_x + rel_y_m * seg_dir_y;
    *segment_length_m = seg_norm_m;
    *normal_error_m = rel_x_m * (-seg_dir_y) + rel_y_m * seg_dir_x;
    return 1U;
}
#endif

#if PATH_FOLLOW_ENABLE_CORNER_HANDOVER
/**
 * @brief 根据前后段较短长度计算进入 handover 的距离阈值。
 *
 * @param prev_length_m 前一段长度，单位 m。
 * @param next_length_m 后一段长度，单位 m。
 * @return 建议的 handover 进入距离，单位 m。
 */

#endif

/**
 * @brief 计算当前路径段末端希望保留的过渡速度。
 *
 * @return 当前段末端目标速度，单位 cm/s。
 */
static float path_follow_compute_segment_end_speed(void)
{
    if ((g_ctx.idx + 1U) >= g_ctx.steps)
    {
        return 0.0f;
    }
    if (path_follow_target_requires_pause(g_ctx.idx))
    {
        return 0.0f;
    }



    return g_ctx.speed_cfg.segment_end_speed_cmps;
}

/**
 * @brief 根据当前剩余距离计算一个仅用于异常保护的安全速度上限。
 *
 * @param distance_m 当前段剩余距离，单位 m。
 * @param end_speed_cmps 当前段末端目标速度，单位 cm/s。
 * @param max_speed_cmps 当前路径段锁定的最大速度，单位 cm/s。
 * @param accel_cmpss 当前路径段锁定的最大加速度，单位 cm/s^2。
 * @return 按最大加速度约束换算得到的安全速度上限，单位 cm/s。
 */
static float path_follow_compute_brake_speed_cap(float distance_m,
                                                 float end_speed_cmps,
                                                 float max_speed_cmps,
                                                 float accel_cmpss)
{
    float distance_cm = fmaxf(distance_m, 0.0f) * 100.0f;
    accel_cmpss = fmaxf(accel_cmpss, 1.0f);
    float speed_sq = end_speed_cmps * end_speed_cmps +
                     2.0f * accel_cmpss * distance_cm;

    return fminf(max_speed_cmps,
                 sqrtf(fmaxf(speed_sq, 0.0f)));
}

/**
 * @brief 当 profile 构造失败时，用单拍加速度限幅生成保守速度作为异常保护。
 *
 * @param current_speed_cmps 当前参考速度，单位 cm/s。
 * @param safety_speed_cmps 当前允许的安全速度上限，单位 cm/s。
 * @param accel_cmpss 当前路径段锁定的最大加速度，单位 cm/s^2。
 * @return 下一拍的速度参考，单位 cm/s。
 */
static float path_follow_compute_profile_fault_speed(float current_speed_cmps,
                                                     float safety_speed_cmps,
                                                     float accel_cmpss)
{
    float accel_step = fmaxf(accel_cmpss, 1.0f) / (float)PID_RATE;

    if (current_speed_cmps < safety_speed_cmps)
    {
        current_speed_cmps += accel_step;
        if (current_speed_cmps > safety_speed_cmps)
        {
            current_speed_cmps = safety_speed_cmps;
        }
    }
    else
    {
        current_speed_cmps -= accel_step;
        if (current_speed_cmps < safety_speed_cmps)
        {
            current_speed_cmps = safety_speed_cmps;
        }
    }

    return fmaxf(current_speed_cmps, 0.0f);
}

/**
 * @brief 清空对外可见的调试状态缓存。
 *
 * @note 该函数不会影响路径、位姿和 PID，仅清除状态查询与屏显用的调试量。
 */
static void path_follow_clear_debug_state(void)
{
    g_ctx.debug.distance_m = 0.0f;
    g_ctx.debug.dir_x = 0.0f;
    g_ctx.debug.dir_y = 0.0f;
    g_ctx.debug.speed_ref_cmps = 0.0f;
    g_ctx.debug.target_yaw_deg = 0.0f;
    g_ctx.debug.vx_world_cmps = 0.0f;
    g_ctx.debug.vy_world_cmps = 0.0f;
    g_ctx.debug.segment_axis = 0U;
}

#if PATH_FOLLOW_ENABLE_CORNER_HANDOVER
/**
 * @brief 清空普通中间拐点 handover 的运行态缓存。
 */
static void path_follow_reset_corner_handover_state(void)
{
    g_ctx.corner_handover_active = 0U;
    g_ctx.corner_handover_idx = SIZE_MAX;
    g_ctx.corner_enter_distance_m = 0.0f;
    g_ctx.corner_commit_distance_m = 0.0f;
}
#endif

/**
 * @brief 使当前路径段的 S 曲线 profile 失效，但保留上一拍速度参考。
 */
static void path_follow_invalidate_active_profile(void)
{
    g_ctx.active_profile = (path_follow_scurve_profile_t){0};
    g_ctx.profile_time_s = 0.0f;
    g_ctx.profile_target_idx = 0U;
    g_ctx.profile_active = 0U;
}

/**
 * @brief 复位整段式 S 曲线执行状态。
 */
static void path_follow_reset_motion_profile_state(void)
{
    path_follow_invalidate_active_profile();
#if PATH_FOLLOW_ENABLE_CORNER_HANDOVER
    path_follow_reset_corner_handover_state();
#endif
    path_follow_reset_active_scurve_cfg();
    g_ctx.last_ref_speed_cmps = 0.0f;
}

/**
 * @brief 将 `pid_stay` 的参数同步到 `pid_stay_y`。
 *
 * @note 两个 PID 使用相同参数，但保持各自独立的内部历史状态。
 */
static void path_follow_sync_stay_pid_gains(void)
{
    pid_stay_y.fKp = pid_stay.fKp;
    pid_stay_y.fKi = pid_stay.fKi;
    pid_stay_y.fKd = pid_stay.fKd;
    pid_stay_y.fMax_Iout = pid_stay.fMax_Iout;
    pid_stay_y.fMax_Out = pid_stay.fMax_Out;
    pid_stay_y.alpha = pid_stay.alpha;
}


/**
 * @brief 清空本模块内部控制状态。
 *
 * 该函数会清除当前段 S 曲线 profile、调试状态以及平移位置相关 PID 的历史量，
 * 用于新路径装载、位姿重置或整条路径结束等需要重新起控的场景。
 */
static void path_follow_reset_control_state(void)
{
    path_follow_reset_motion_profile_state();
    path_follow_clear_debug_state();
    PID_Clear(&pid_world_x);
    PID_Clear(&pid_world_y);
    PID_Clear(&pid_stay);
    PID_Clear(&pid_stay_y);
    //PID_Clear(&pid_yaw);
    //PID_Clear(&pid_accel_yaw);
}

static uint8 path_follow_finish_path(path_follow_output_t *out);

/**
 * @brief 复位暂停事件的运行态，不清除配置表本身。
 *
 * @note 新路径装载、临时路径切换以及恢复到普通跟随前，都需要把游标和剩余倒计时清零。
 */
static void path_follow_reset_pause_runtime(void)
{
    g_ctx.pause_cursor = 0U;
    g_ctx.pause_cycles_remaining = 0U;
    g_ctx.paused = 0U;
}

/**
 * @brief 清空暂停事件配置及其运行态。
 */
static void path_follow_clear_pause_config(void)
{
    size_t i;

    for (i = 0U; i < PATH_FOLLOW_MAX_PAUSE_POINTS; ++i)
    {
        g_ctx.pause_indices[i] = SIZE_MAX;
    }
    g_ctx.pause_count = 0U;
    g_ctx.pause_cycles_cfg = 0U;
    path_follow_reset_pause_runtime();
}

/**
 * @brief 将暂停游标追赶到当前目标索引，跳过已经过去的旧事件。
 *
 * @param current_idx 当前路径目标索引。
 */
static void path_follow_sync_pause_cursor(size_t current_idx)
{
    while (g_ctx.pause_cursor < g_ctx.pause_count &&
           g_ctx.pause_indices[g_ctx.pause_cursor] < current_idx)
    {
        g_ctx.pause_cursor++;
    }
}

/**
 * @brief 判断当前目标 corner 是否需要在到点后暂停。
 *
 * @param target_idx 当前目标 corner 索引。
 * @return `1` 表示该 target_idx 是下一个待触发暂停点，`0` 表示不是。
 */
static uint8 path_follow_target_requires_pause(size_t target_idx)
{
    if (!g_ctx.pause_events_enabled || g_ctx.paused || 0U == g_ctx.pause_cycles_cfg)
    {
        return 0U;
    }

    path_follow_sync_pause_cursor(target_idx);
    if (g_ctx.pause_cursor >= g_ctx.pause_count)
    {
        return 0U;
    }

    return (g_ctx.pause_indices[g_ctx.pause_cursor] == target_idx) ? 1U : 0U;
}

/**
 * @brief 进入一个“到点后静止等待”的暂停窗口。
 *
 * @param target_idx 当前触发暂停的 corner 索引，仅用于调试语义保持。
 */
static void path_follow_enter_pause(size_t target_idx)
{
    (void)target_idx;

    g_ctx.paused = 1U;
    g_ctx.pause_cycles_remaining = (g_ctx.pause_cycles_cfg > 0U) ? g_ctx.pause_cycles_cfg : 1U;
    car_direction = 0U;
    /* Pause resumes with a fresh segment profile, so clear the current motion state here. */
    path_follow_reset_control_state();
}

/**
 * @brief 处理暂停窗口的倒计时与恢复逻辑。
 *
 * @param out 输出结构体。暂停期间保持 active=1 且三轴速度为 0。
 * @return `1` 表示本周期已由暂停逻辑完全处理，`0` 表示当前不在暂停态。
 */
static uint8 path_follow_handle_pause(path_follow_output_t *out)
{
    if (!g_ctx.paused)
    {
        return 0U;
    }

    car_direction = 0U;
    path_follow_clear_debug_state();

    if (out != NULL)
    {
        out->active = 1U;
        out->reached = 0U;
        out->vx_cmd = 0.0f;
        out->vy_cmd = 0.0f;
        out->omega_cmd = 0.0f;
        out->target_idx = g_ctx.idx;
    }

    if (g_ctx.pause_cycles_remaining > 0U)
    {
        g_ctx.pause_cycles_remaining--;
    }
    if (g_ctx.pause_cycles_remaining > 0U)
    {
        return 1U;
    }

    g_ctx.paused = 0U;
    if (g_ctx.pause_cursor < g_ctx.pause_count &&
        g_ctx.pause_indices[g_ctx.pause_cursor] == g_ctx.idx)
    {
        g_ctx.pause_cursor++;
    }

    if ((g_ctx.idx + 1U) < g_ctx.steps)
    {
        g_ctx.idx++;
        /* The next segment must rebuild its own profile from standstill after pause release. */
        path_follow_reset_control_state();
        return 1U;
    }

    (void)path_follow_finish_path(out);
    if (out != NULL)
    {
        out->active = 0U;
        out->target_idx = g_ctx.idx;
    }
    return 1U;
}

/**
 * @brief 装载一条新的路径到路径跟随上下文。
 *
 * 该函数负责统一处理路径切换时的状态重置、网格分辨率切换、
 * 起点重置以及首目标点跳过策略。
 *
 * @param path 路径点数组指针。
 * @param steps 路径点数量。
 * @param grid_m 路径对应网格边长，单位 m。
 * @param reset_pose_to_first 是否将当前位姿重置到路径首点。
 * @param skip_first_target 是否跳过路径首点，直接跟随下一目标点。
 * @param pause_events_enabled 该路径是否允许触发预配置的暂停事件。
 */
static void path_follow_apply_path(const Position *path,
                                   size_t steps,
                                   float grid_m,
                                   uint8 reset_pose_to_first,
                                   uint8 skip_first_target,
                                   uint8 pause_events_enabled)
{
    path_follow_reset_control_state();
    path_follow_reset_pause_runtime();
    g_ctx.path = path;
    g_ctx.steps = steps;
    g_ctx.grid_m = (grid_m > 0.0f) ? grid_m : g_ctx.default_grid_m;
    g_ctx.idx = 0;
    g_ctx.pause_events_enabled = pause_events_enabled;
    g_ctx.rotate_only_active = 0U;

    if (path && steps > 0U)
    {
        if (reset_pose_to_first)
        {
            g_ctx.pose.x_m = path[0].row * g_ctx.grid_m;
            g_ctx.pose.y_m = path[0].col * g_ctx.grid_m;
        }

        if (skip_first_target && steps > 1U)
        {
            g_ctx.idx = 1U;
        }

        g_ctx.active = 1U;
    }
    else
    {
        g_ctx.active = 0U;
    }
}

/**
 * @brief 生成一条只包含水平/竖直两段的临时路径并启动跟随。
 *
 * 该函数主要用于发车前固定偏移和外部位姿修正场景，
 * 会自动根据位移方向决定先走 X 还是先走 Y。
 *
 * @param target_x_m 临时目标点 X 坐标，单位 m。
 * @param target_y_m 临时目标点 Y 坐标，单位 m。
 * @param path_buffer 临时路径缓存。
 * @param buffer_capacity 路径缓存容量。
 * @param grid_m 生成临时路径时使用的栅格分辨率，单位 m。
 */
static void path_follow_start_axis_move(float target_x_m,
                                        float target_y_m,
                                        Position *path_buffer,
                                        size_t buffer_capacity,
                                        float grid_m)
{
    const float eps_m = 0.001f;
    float start_x_m = g_ctx.pose.x_m;
    float start_y_m = g_ctx.pose.y_m;
    float delta_x_m = target_x_m - start_x_m;
    float delta_y_m = target_y_m - start_y_m;
    uint8 move_x_first = (fabsf(delta_x_m) >= fabsf(delta_y_m)) ? 1U : 0U;
    size_t steps = 1U;

    if (path_buffer == NULL || buffer_capacity < 3U)
    {
        return;
    }

    if (fabsf(delta_x_m) <= eps_m && fabsf(delta_y_m) <= eps_m)
    {
        path_follow_apply_path(NULL, 0U, grid_m, 0U, 0U, 0U);
        return;
    }

    path_buffer[0].row = (int)lroundf(start_x_m / grid_m);
    path_buffer[0].col = (int)lroundf(start_y_m / grid_m);

    if (move_x_first)
    {
        if (fabsf(delta_x_m) > eps_m)
        {
            path_buffer[steps].row = (int)lroundf(target_x_m / grid_m);
            path_buffer[steps].col = (int)lroundf(start_y_m / grid_m);
            steps++;
        }

        if (fabsf(delta_y_m) > eps_m)
        {
            path_buffer[steps].row = (int)lroundf(target_x_m / grid_m);
            path_buffer[steps].col = (int)lroundf(target_y_m / grid_m);
            steps++;
        }
    }
    else
    {
        if (fabsf(delta_y_m) > eps_m)
        {
            path_buffer[steps].row = (int)lroundf(start_x_m / grid_m);
            path_buffer[steps].col = (int)lroundf(target_y_m / grid_m);
            steps++;
        }

        if (fabsf(delta_x_m) > eps_m)
        {
            path_buffer[steps].row = (int)lroundf(target_x_m / grid_m);
            path_buffer[steps].col = (int)lroundf(target_y_m / grid_m);
            steps++;
        }
    }

    path_follow_apply_path(path_buffer, steps, grid_m, 0U, 1U, 0U);
}

/**
 * @brief 初始化路径跟随模块。
 *
 * 该函数完成上下文默认值、速度规划参数以及相关 PID 的初始化。
 *
 * @param grid_size_m 默认路径网格边长，单位 m。
 * @param pulses_per_meter 编码器每米脉冲数，用于里程计换算。
 */
void path_follow_init(float grid_size_m, float pulses_per_meter)
{
    // 初始化参数和 PID
    g_ctx.grid_m = grid_size_m;
    g_ctx.default_grid_m = grid_size_m;
    g_ctx.pulses_per_meter = pulses_per_meter;
    g_ctx.pos_tol_m = PATH_FOLLOW_POSITION_TOLERANCE_M;   // 位置容差，可按需要调整
    g_ctx.yaw_tol_deg = PATH_FOLLOW_YAW_TOLERANCE_DEG;  // 航向容差
    g_ctx.max_v_mps = PATH_FOLLOW_MAX_LINEAR_SPEED_MPS;    // 最大线速度
    g_ctx.max_w_rad = PATH_FOLLOW_MAX_ANGULAR_SPEED_LIMIT;    // 角速度上限
    g_ctx.target_yaw_deg = 0.0f;
    g_ctx.heading_mode = PATH_FOLLOW_HEADING_FIXED;
    g_ctx.pose.x_m = 0.0f;
    g_ctx.pose.y_m = 0.0f;
    g_ctx.pose.yaw_deg = 0.0f;
    g_ctx.speed_cfg.segment_end_speed_cmps = SCURVE_SEGMENT_END_SPEED_CMPS;
    g_ctx.speed_cfg.hold_release_distance_m = PATH_HOLD_TRIM_RELEASE_DISTANCE;
    g_ctx.path = NULL;
    g_ctx.steps = 0;
    g_ctx.idx = 0;
    g_ctx.rotate_only_active = 0U;
    g_ctx.active = 0;
    g_ctx.pause_events_enabled = 0U;
    path_follow_reset_scurve_band_defaults();
    path_follow_reset_motion_profile_state();
    path_follow_clear_debug_state();
    path_follow_clear_pause_config();

    pid_world_init.fKp = PATH_FOLLOW_PID_WORLD_KP;  // 位置误差到速度输出
    pid_world_init.fKi = PATH_FOLLOW_PID_WORLD_KI;
    pid_world_init.fKd = PATH_FOLLOW_PID_WORLD_KD;
    pid_world_init.fMax_Iout = g_ctx.max_v_mps*100.0f;
    pid_world_init.fMax_Out = g_ctx.max_v_mps*100.0f;
    pid_world_init.alpha = PATH_FOLLOW_PID_WORLD_ALPHA;

    pid_stay_init.fKp = PATH_FOLLOW_PID_STAY_KP;  // 位置误差到速度输出
    pid_stay_init.fKi = PATH_FOLLOW_PID_STAY_KI;
    pid_stay_init.fKd = PATH_FOLLOW_PID_STAY_KD;
    pid_stay_init.fMax_Iout = PATH_FOLLOW_PID_STAY_MAX_IOUT_CMPS;
    pid_stay_init.fMax_Out = PATH_FOLLOW_PID_STAY_MAX_OUT_CMPS;
    pid_stay_init.alpha = PATH_FOLLOW_PID_STAY_ALPHA;

    pid_yaw_init.fKp = PATH_FOLLOW_PID_YAW_KP;  // 航向误差到角速度输出
    pid_yaw_init.fKi = PATH_FOLLOW_PID_YAW_KI;
    pid_yaw_init.fKd = PATH_FOLLOW_PID_YAW_KD;
    pid_yaw_init.fMax_Iout = g_ctx.max_w_rad;
    pid_yaw_init.fMax_Out = g_ctx.max_w_rad;
    pid_yaw_init.alpha = PATH_FOLLOW_PID_YAW_ALPHA;

    pid_accel_yaw_init.fKp = PATH_FOLLOW_PID_ACCEL_YAW_KP;  // 航向误差到角速度输出
    pid_accel_yaw_init.fKi = PATH_FOLLOW_PID_ACCEL_YAW_KI;
    pid_accel_yaw_init.fKd = PATH_FOLLOW_PID_ACCEL_YAW_KD;
    pid_accel_yaw_init.fMax_Iout = g_ctx.max_w_rad;
    pid_accel_yaw_init.fMax_Out = g_ctx.max_w_rad;
    pid_accel_yaw_init.alpha = PATH_FOLLOW_PID_ACCEL_YAW_ALPHA;

    PID_Init(&pid_world_x, &pid_world_init);
    PID_Init(&pid_stay, &pid_stay_init);
    PID_Init(&pid_stay_y, &pid_stay_init);
    PID_Init(&pid_world_y, &pid_world_init);
    PID_Init(&pid_yaw, &pid_yaw_init);
    PID_Init(&pid_accel_yaw, &pid_accel_yaw_init);
}

/**
 * @brief 重置当前里程计位姿。
 *
 * 重置位姿时会同时清空平移控制状态，避免旧路径残留的 PID 历史量继续作用。
 *
 * @param x_m 新的 X 坐标，单位 m。
 * @param y_m 新的 Y 坐标，单位 m。
 * @param yaw_deg 新的航向角，单位 deg。
 */
void path_follow_reset_pose(float x_m, float y_m, float yaw_deg)
{
    g_ctx.pose.x_m = x_m;
    g_ctx.pose.y_m = y_m;
    g_ctx.pose.yaw_deg = path_follow_wrap_deg(yaw_deg);
    path_follow_reset_control_state();
}

/**
 * @brief 设置外部位姿输入接口。
 *
 * @param x_m 外部位置 X，单位 m。
 * @param y_m 外部位置 Y，单位 m。
 * @param valid 外部位置是否有效。
 *
 * @note 当前版本保留接口但未在本模块内部直接使用。
 */
void path_follow_set_external_position(float x_m, float y_m, uint8 valid)
{
    (void)x_m;
    (void)y_m;
    (void)valid;
}

/**
 * @brief 设置一条新的跟随路径。
 *
 * @param path 路径点数组指针。
 * @param steps 路径点数量。
 */
void path_follow_set_path(const Position *path, size_t steps)
{
    path_follow_set_path_pause_enabled(path, steps, 1U);
}

/**
 * @brief 设置一条新的跟随路径，并显式指定该路径是否允许触发 pause 事件。
 *
 * @param path 路径点数组指针。
 * @param steps 路径点数量。
 * @param pause_events_enabled `1` 表示允许使用已配置的 pause 索引，`0` 表示本路径禁用 pause。
 */
void path_follow_set_path_pause_enabled(const Position *path, size_t steps, uint8 pause_events_enabled)
{
    path_follow_apply_path(path, steps, g_ctx.default_grid_m, 0U, 1U, pause_events_enabled);
}

/**
 * @brief 配置普通地图 corner_path 上的暂停事件列表。
 *
 * @param pause_indices 需要暂停的 target corner 索引数组，可为 NULL。
 * @param pause_count pause_indices 中的元素个数。
 * @param pause_ms 每个暂停点统一停留时长，单位 ms；传 0 可清空配置。
 */
void path_follow_set_pause_indices(const size_t *pause_indices, size_t pause_count, uint32 pause_ms)
{
    size_t copy_count = 0U;
    size_t i;

    path_follow_clear_pause_config();
    if (pause_indices == NULL || pause_count == 0U || pause_ms == 0U)
    {
        return;
    }

    if (pause_count > PATH_FOLLOW_MAX_PAUSE_POINTS)
    {
        pause_count = PATH_FOLLOW_MAX_PAUSE_POINTS;
    }

    for (i = 0U; i < pause_count; ++i)
    {
        g_ctx.pause_indices[copy_count++] = pause_indices[i];
    }
    g_ctx.pause_count = copy_count;
    g_ctx.pause_cycles_cfg = (pause_ms * (uint32)PID_RATE + 999U) / 1000U;
    if (g_ctx.pause_cycles_cfg == 0U)
    {
        g_ctx.pause_cycles_cfg = 1U;
    }
}

/**
 * @brief 设置单个离散目标点并生成临时两点路径。
 *
 * @param target_row 目标点所在网格行号。
 * @param target_col 目标点所在网格列号。
 */
void path_follow_set_target(int target_row, int target_col)
{
    g_single_target_path[0].row = (int)lroundf(g_ctx.pose.x_m / g_ctx.default_grid_m);
    g_single_target_path[0].col = (int)lroundf(g_ctx.pose.y_m / g_ctx.default_grid_m);
    g_single_target_path[1].row = target_row;
    g_single_target_path[1].col = target_col;
    path_follow_apply_path(g_single_target_path, 2U, g_ctx.default_grid_m, 0U, 1U, 0U);
}

void path_follow_set_target_yaw(float target_yaw_deg)
{
    g_ctx.target_yaw_deg = path_follow_wrap_deg(target_yaw_deg);
}

void path_follow_hold_current_yaw(void)
{
    path_follow_set_target_yaw(g_ctx.pose.yaw_deg);
}

void path_follow_start_rotate_to_yaw(float target_yaw_deg)
{
    path_follow_reset_control_state();
    PID_Clear(&pid_yaw);
    g_ctx.path = NULL;
    g_ctx.steps = 0U;
    g_ctx.idx = 0U;
    g_ctx.active = 0U;
    g_ctx.paused = 0U;
    g_ctx.pause_cycles_remaining = 0U;
    g_ctx.rotate_only_active = 1U;
    g_ctx.pause_events_enabled = 0U;
    path_follow_set_target_yaw(target_yaw_deg);
}

/**
 * @brief 启动发车前固定偏移动作。
 *
 * @param delta_x_m X 方向偏移量，单位 m。
 * @param delta_y_m Y 方向偏移量，单位 m。
 */
void path_follow_start_offset_move(float delta_x_m, float delta_y_m)
{
    float start_x_m = g_ctx.pose.x_m;
    float start_y_m = g_ctx.pose.y_m;
    path_follow_start_axis_move(start_x_m + delta_x_m,
                                start_y_m + delta_y_m,
                                g_prestart_offset_path,
                                sizeof(g_prestart_offset_path) / sizeof(g_prestart_offset_path[0]),
                                PRESTART_OFFSET_RESOLUTION_M);
}

/**
 * @brief 启动位姿修正动作。
 *
 * 该函数会生成一条短临时路径，使底盘按当前位置缓慢靠近外部修正目标。
 *
 * @param target_x_m 修正目标 X 坐标，单位 m。
 * @param target_y_m 修正目标 Y 坐标，单位 m。
 */
void path_follow_start_pose_correction(float target_x_m, float target_y_m)
{
    path_follow_start_axis_move(target_x_m,
                                target_y_m,
                                g_pose_correction_path,
                                sizeof(g_pose_correction_path) / sizeof(g_pose_correction_path[0]),
                                PRESTART_OFFSET_RESOLUTION_M);
}
/**
 * @brief 根据四轮编码器和当前航向估算世界系位姿。
 *
 * @param yaw_deg 当前 IMU 航向角，单位 deg。
 */
static void update_odometry(float yaw_deg)
{
    if (g_ctx.pulses_per_meter <= 0.0f)
    {
        return;
    }

    float count_to_mps = ((float)PID_RATE) / g_ctx.pulses_per_meter;
    float w_ul = (float)up_L_all * count_to_mps;
    float w_ur = (float)up_R_all * count_to_mps;
    float w_dl = (float)down_L_all * count_to_mps;
    float w_dr = (float)down_R_all * count_to_mps;

    float vx_body = 0.25f * (w_ul + w_ur + w_dl + w_dr);
    float vy_body = 0.25f * (-w_ul + w_ur + w_dl - w_dr);
    float omega_body = (-w_ul + w_ur - w_dl + w_dr) / (2*D_X + 2*D_Y);
    // if (rx_plus_ry_cali != 0.0f)
    // {
    //     omega_body /= rx_plus_ry_cali;
    // }

    float dt = 1.0f / (float)PID_RATE;
    float yaw_rad = yaw_deg * ((float)M_PI / 180.0f);

    float cos_yaw = cosf(yaw_rad);
    float sin_yaw = sinf(yaw_rad);

    float vx_world = vx_body * cos_yaw - vy_body * sin_yaw;
    float vy_world = vx_body * sin_yaw + vy_body * cos_yaw;

    g_ctx.pose.x_m += vx_world * dt;
    g_ctx.pose.y_m += vy_world * dt;
    g_ctx.pose.yaw_deg = path_follow_wrap_deg(yaw_deg);
}

/**
 * @brief 根据目标点误差判断当前路径段主轴方向。
 *
 * @param delta_x_m 目标点相对当前位置的 X 方向误差，单位 m。
 * @param delta_y_m 目标点相对当前位置的 Y 方向误差，单位 m。
 * @return `0` 静止，`1` X 段，`2` Y 段，`3` 斜段。
 */
static uint8 path_follow_resolve_segment_axis(float delta_x_m, float delta_y_m)
{
    float delta_x_abs = fabsf(delta_x_m);
    float delta_y_abs = fabsf(delta_y_m);
    const float axis_eps_m = 0.001f;

    if (delta_x_abs <= axis_eps_m && delta_y_abs <= axis_eps_m)
    {
        return 0U;
    }
    if (delta_y_abs <= axis_eps_m)
    {
        return 1U;
    }
    if (delta_x_abs <= axis_eps_m)
    {
        return 2U;
    }
    return 3U;
}

/**
 * @brief 将路径跟随状态切换为完成。
 *
 * @param out 输出结构体指针，用于回写 `reached` 标志。
 * @return 固定返回 `0`，便于在调用点直接作为失败/结束分支使用。
 */
static uint8 path_follow_finish_path(path_follow_output_t *out)
{
    g_ctx.active = 0U;
    car_direction = 0U;
    path_follow_reset_control_state();
    if (out != NULL)
    {
        out->reached = 1U;
    }
    return 0U;
}

/**
 * @brief 为当前周期准备路径几何信息。
 *
 * 该函数会跳过已经到达的目标点，必要时自动推进到下一个拐点；
 * 若整条路径执行完毕，则在内部完成结束收尾。
 *
 * @param geometry 几何信息输出指针。
 * @param out 本周期控制输出指针，用于路径结束时回写状态。
 * @return `1` 表示成功得到有效几何信息，`0` 表示当前无有效控制目标。
 */
static uint8 path_follow_prepare_geometry(path_follow_geometry_t *geometry, path_follow_output_t *out)
{
    if (geometry == NULL)
    {
        return 0U;
    }

    while (g_ctx.active && g_ctx.path && g_ctx.idx < g_ctx.steps)
    {
        uint8 pause_target;

        Position target;

        path_follow_sync_pause_cursor(g_ctx.idx);
        target = g_ctx.path[g_ctx.idx];

        geometry->target_point = target;
        geometry->target_x_m = target.row * g_ctx.grid_m;
        geometry->target_y_m = target.col * g_ctx.grid_m;
        geometry->delta_x_m = geometry->target_x_m - g_ctx.pose.x_m;
        geometry->delta_y_m = geometry->target_y_m - g_ctx.pose.y_m;
        geometry->distance_m = sqrtf(geometry->delta_x_m * geometry->delta_x_m +
                                     geometry->delta_y_m * geometry->delta_y_m);
        pause_target = path_follow_target_requires_pause(g_ctx.idx);

        if (geometry->distance_m >= g_ctx.pos_tol_m)
        {
            geometry->dir_x = geometry->delta_x_m / geometry->distance_m;
            geometry->dir_y = geometry->delta_y_m / geometry->distance_m;
            geometry->segment_axis = path_follow_resolve_segment_axis(geometry->delta_x_m,
                                                                      geometry->delta_y_m);
            car_direction = geometry->segment_axis;
            return 1U;
        }

        if ((g_ctx.idx + 1U) < g_ctx.steps)
        {
            if (pause_target)
            {
                path_follow_enter_pause(g_ctx.idx);
                return 0U;
            }
#if PATH_FOLLOW_ENABLE_CORNER_HANDOVER
            path_follow_reset_corner_handover_state();
            path_follow_invalidate_active_profile();
#endif
            g_ctx.idx++;
            continue;
        }

        if (pause_target)
        {
            path_follow_enter_pause(g_ctx.idx);
            return 0U;
        }
        return path_follow_finish_path(out);
    }

    car_direction = 0U;
    return 0U;
}

/**
 * @brief 为当前路径段构造一次性的标准 S 曲线 profile。
 *
 * @note 该函数只在进入新路径段、切换目标点或 profile 被显式失效时调用，
 *       正常执行期不会重复重建 profile。
 *
 * @param geometry 当前周期路径几何信息。
 * @param speed_plan 速度规划结果输出，用于回填段末速度与安全速度上限。
 * @return `1` 表示当前段 profile 构造成功，`0` 表示构造失败。
 */
static uint8 path_follow_build_active_profile(const path_follow_geometry_t *geometry,
                                              path_follow_speed_plan_t *speed_plan)
{
    path_follow_scurve_profile_t profile = {0};
    path_follow_scurve_runtime_cfg_t selected_cfg = {0};
    float start_speed_cmps;

    if (geometry == NULL || speed_plan == NULL)
    {
        return 0U;
    }

    path_follow_select_scurve_band(geometry->distance_m, &selected_cfg);
    g_ctx.active_scurve_cfg = selected_cfg;

    speed_plan->end_speed_cmps = fminf(path_follow_compute_segment_end_speed(),
                                       g_ctx.active_scurve_cfg.max_speed_cmps);
    speed_plan->safety_cap_cmps = path_follow_compute_brake_speed_cap(geometry->distance_m,
                                                                      speed_plan->end_speed_cmps,
                                                                      g_ctx.active_scurve_cfg.max_speed_cmps,
                                                                      g_ctx.active_scurve_cfg.accel_cmpss);

    start_speed_cmps = fmaxf(g_ctx.last_ref_speed_cmps, 0.0f);
    start_speed_cmps = fminf(start_speed_cmps, speed_plan->safety_cap_cmps);

    if (!path_follow_build_scurve_profile(&profile,
                                          geometry->distance_m * 100.0f,
                                          start_speed_cmps,
                                          speed_plan->end_speed_cmps,
                                          g_ctx.active_scurve_cfg.max_speed_cmps,
                                          g_ctx.active_scurve_cfg.accel_cmpss,
                                          g_ctx.active_scurve_cfg.jerk_cmpsss))
    {
        path_follow_invalidate_active_profile();
        return 0U;
    }

    g_ctx.active_profile = profile;
    g_ctx.profile_time_s = 0.0f;
    g_ctx.profile_target_idx = g_ctx.idx;
    g_ctx.profile_active = 1U;
    speed_plan->end_speed_cmps = profile.v1;
    return 1U;
}

/**
 * @brief 规划当前周期的标量速度参考。
 *
 * 当前实现采用“整段一次规划 + 按累计时间连续采样”的标准 S 曲线执行方式：
 * 1. 仅在进入新路径段时构造一次完整 profile；
 * 2. profile 内固定该段的起点速度、终点速度与约束；
 * 3. 每个控制周期只推进该段的累计时间并采样速度；
 * 4. 当前剩余距离只作为安全监护层，不再参与主轨迹滚动重建；
 * 5. 只有 profile 构造失败时，才退化到异常保护速度。
 *
 * @param geometry 当前周期路径几何信息。
 * @param speed_plan 速度规划结果输出。
 */

static void path_follow_plan_speed(const path_follow_geometry_t *geometry,
                                   path_follow_speed_plan_t *speed_plan)
{
    const float sample_dt_s = 1.0f / (float)PID_RATE;
    uint8 build_ok = 1U;

    if (geometry == NULL || speed_plan == NULL)
    {
        return;
    }

    speed_plan->end_speed_cmps = path_follow_compute_segment_end_speed();
    speed_plan->safety_cap_cmps = 0.0f;
    speed_plan->ref_speed_cmps = 0.0f;

    if (!g_ctx.profile_active || g_ctx.profile_target_idx != g_ctx.idx)
    {
        build_ok = path_follow_build_active_profile(geometry, speed_plan);
    }
    else
    {
        speed_plan->end_speed_cmps = g_ctx.active_profile.v1;
        speed_plan->safety_cap_cmps = path_follow_compute_brake_speed_cap(geometry->distance_m,
                                                                          speed_plan->end_speed_cmps,
                                                                          g_ctx.active_scurve_cfg.max_speed_cmps,
                                                                          g_ctx.active_scurve_cfg.accel_cmpss);
    }

    if (g_ctx.profile_active)
    {
        g_ctx.profile_time_s += sample_dt_s;
        if (g_ctx.profile_time_s > g_ctx.active_profile.T)
        {
            g_ctx.profile_time_s = g_ctx.active_profile.T;
        }

        speed_plan->ref_speed_cmps = path_follow_sample_scurve_velocity(g_ctx.profile_time_s,
                                                                        &g_ctx.active_profile);
        speed_plan->end_speed_cmps = g_ctx.active_profile.v1;
        speed_plan->safety_cap_cmps = path_follow_compute_brake_speed_cap(geometry->distance_m,
                                                                          speed_plan->end_speed_cmps,
                                                                          g_ctx.active_scurve_cfg.max_speed_cmps,
                                                                          g_ctx.active_scurve_cfg.accel_cmpss);
    }
    else if (!build_ok)
    {
        speed_plan->ref_speed_cmps = path_follow_compute_profile_fault_speed(g_ctx.last_ref_speed_cmps,
                                                                             speed_plan->safety_cap_cmps,
                                                                             g_ctx.active_scurve_cfg.accel_cmpss);
    }

    speed_plan->ref_speed_cmps = clamp_sym(speed_plan->ref_speed_cmps,
                                           g_ctx.active_scurve_cfg.max_speed_cmps);
    speed_plan->ref_speed_cmps = fminf(speed_plan->ref_speed_cmps,
                                       speed_plan->safety_cap_cmps);
    speed_plan->ref_speed_cmps = fmaxf(speed_plan->ref_speed_cmps, 0.0f);

    if (geometry->distance_m <= g_ctx.pos_tol_m)
    {
        speed_plan->ref_speed_cmps = 0.0f;
    }

    g_ctx.last_ref_speed_cmps = speed_plan->ref_speed_cmps;
}

/**
 * @brief 计算定点保持修正的混合权重。
 *
 * @param distance_m 当前距目标点的剩余距离，单位 m。
 * @return `[0, 1]` 范围内的保持权重。
 */
static float path_follow_compute_hold_blend(float distance_m)
{
    float release_distance_m = g_ctx.speed_cfg.hold_release_distance_m;

    if (distance_m >= release_distance_m)
    {
        return 0.0f;
    }
    if (distance_m <= g_ctx.pos_tol_m)
    {
        return 1.0f;
    }
    if (release_distance_m <= g_ctx.pos_tol_m)
    {
        return 1.0f;
    }

    return 1.0f - ((distance_m - g_ctx.pos_tol_m) /
                   (release_distance_m - g_ctx.pos_tol_m));
}

/**
 * @brief 判断当前目标点是否属于普通路线中的中间拐点。
 *
 * @note 这里显式排除启动偏移/位姿修正使用的临时细网格路径，只在正常地图路径上生效。
 *
 * @return `1` 表示当前目标是普通路线中仍有后续段的拐点，`0` 表示不是。
 */
static uint8 path_follow_is_route_corner_target(void)
{
    const float grid_eps_m = 1e-6f;
    Position prev_point;
    Position curr_point;
    Position next_point;
    int d1_row;
    int d1_col;
    int d2_row;
    int d2_col;

    if (g_ctx.path == NULL || g_ctx.steps < 3U)
    {
        return 0U;
    }
    if (g_ctx.idx == 0U || (g_ctx.idx + 1U) >= g_ctx.steps)
    {
        return 0U;
    }
    if (fabsf(g_ctx.grid_m - g_ctx.default_grid_m) > grid_eps_m)
    {
        return 0U;
    }

    prev_point = g_ctx.path[g_ctx.idx - 1U];
    curr_point = g_ctx.path[g_ctx.idx];
    next_point = g_ctx.path[g_ctx.idx + 1U];

    d1_row = curr_point.row - prev_point.row;
    d1_col = curr_point.col - prev_point.col;
    d2_row = next_point.row - curr_point.row;
    d2_col = next_point.col - curr_point.col;

    return (d1_row != d2_row || d1_col != d2_col) ? 1U : 0U;
}


/**
 * @brief [CornerHandover移植] 将 0~1 线性进度整形成五次 smoothstep 进度。
 *
 * @param t 线性进度，范围期望为 [0, 1]。
 * @return S 形整形后的进度，范围为 [0, 1]。
 */
#if PATH_FOLLOW_ENABLE_CORNER_HANDOVER
static float path_follow_s_curve_01(float t)
{
    if (t <= 0.0f)
    {
        return 0.0f;
    }
    if (t >= 1.0f)
    {
        return 1.0f;
    }

    return t * t * t * (t * (t * 6.0f - 15.0f) + 10.0f);
}
  
static uint8 path_follow_check_corner_handover_timeout(void)
{
    static uint8 last_active = 0U;
    static uint32 handover_tick = 0U;
    static uint32 handover_idx = 0U;
    const uint32 timeout_tick = PATH_CORNER_HANDOVER_TIMEOUT_S * (uint32)PID_RATE;

    if (!g_ctx.corner_handover_active)
    {
        last_active = 0U;
        handover_tick = 0U;
        handover_idx = 0U;
        return 0U;
    }

    if (!last_active || handover_idx != (uint32)g_ctx.idx)
    {
        last_active = 1U;
        handover_tick = 0U;
        handover_idx = (uint32)g_ctx.idx;
        return 0U;
    }

    if (handover_tick < 0xFFFFFFFFU)
    {
        handover_tick++;
    }

    if (handover_tick >= timeout_tick)
    {
        path_follow_reset_corner_handover_state();
        last_active = 0U;
        handover_tick = 0U;
        handover_idx = 0U;
        return 1U;
    }

    return 0U;
}

/**
 * @brief [CornerHandover重构] 在普通中间拐点前平滑交接世界系速度方向。
 *
 * @note 这里不改变原有段级 S 曲线的标量速度规划，只在普通中间拐点上
 *       按上一段投影进度进入 handover，并在切段前保持该运行态锁存。
 *
 * @param geometry 当前周期路径几何信息。
 * @param ref_speed_cmps 当前周期标量速度参考，单位 cm/s。
 * @param vx_world_cmps 世界系 X 方向速度输出。
 * @param vy_world_cmps 世界系 Y 方向速度输出。
 * @return `1` 表示本周期已应用拐点交接，`0` 表示未应用。
 */
static uint8 path_follow_try_corner_handover(const path_follow_geometry_t *geometry,
                                             float ref_speed_cmps,
                                             float *vx_world_cmps,
                                             float *vy_world_cmps)
{
    const float eps = 1e-6f;
    const Position *prev_point = NULL;
    const Position *curr_point = NULL;
    const Position *next_point = NULL;
    float segment_progress_m = 0.0f;
    float segment_length_m = 0.0f;
    float lateral_error_m = 0.0f;
    float remaining_along_m;
    float prev_dx_m;
    float prev_dy_m;
    float next_dx_m;
    float next_dy_m;
    float prev_length_m;
    float next_length_m;
    float enter_distance_m;
    float commit_distance_m;
    float t;
    float k;
    float v_scale;
    float turn_cos;
    float turn_severity;
    float k_drop;
    float old_x;
    float old_y;
    float new_x;
    float new_y;
    float mix_x;
    float mix_y;
    float mix_norm;

    if (geometry == NULL || vx_world_cmps == NULL || vy_world_cmps == NULL)
    {
        return 0U;
    }
    if (!path_follow_is_route_corner_target())
    {
        return 0U;
    }
    if (path_follow_target_requires_pause(g_ctx.idx))
    {
        return 0U;
    }
    if (!path_follow_get_segment_points(&prev_point, &curr_point, &next_point) ||
        prev_point == NULL || curr_point == NULL || next_point == NULL)
    {
        return 0U;
    }
    if (!path_follow_compute_segment_progress_m(&g_ctx.pose,
                                                prev_point,
                                                curr_point,
                                                &segment_progress_m,
                                                &segment_length_m,
                                                &lateral_error_m))
    {
        return 0U;
    }


    prev_dx_m = (curr_point->row - prev_point->row) * g_ctx.grid_m;
    prev_dy_m = (curr_point->col - prev_point->col) * g_ctx.grid_m;
    next_dx_m = (next_point->row - curr_point->row) * g_ctx.grid_m;
    next_dy_m = (next_point->col - curr_point->col) * g_ctx.grid_m;
    prev_length_m = sqrtf(prev_dx_m * prev_dx_m + prev_dy_m * prev_dy_m);
    next_length_m = sqrtf(next_dx_m * next_dx_m + next_dy_m * next_dy_m);
    if (prev_length_m <= eps || next_length_m <= eps)
    {
        return 0U;
    }

/*     enter_distance_m = path_follow_compute_corner_enter_distance_m(prev_length_m, next_length_m);
    commit_distance_m = path_follow_compute_corner_commit_distance_m(prev_length_m, next_length_m); */
    enter_distance_m = PATH_CORNER_HANDOVER_ENTER_DISTANCE_M;
    commit_distance_m = PATH_CORNER_HANDOVER_COMMIT_DISTANCE_M;
    if (enter_distance_m <= (commit_distance_m + PATH_CORNER_HANDOVER_WINDOW_GAP_MIN_M))
    {
        enter_distance_m = commit_distance_m + PATH_CORNER_HANDOVER_WINDOW_GAP_MIN_M;
    }

    if (g_ctx.corner_handover_active)
    {
        if (g_ctx.corner_handover_idx != g_ctx.idx)
        {
            path_follow_reset_corner_handover_state();
        }
        else
        {
            if (g_ctx.corner_enter_distance_m > 0.0f)
            {
                enter_distance_m = g_ctx.corner_enter_distance_m;
            }
            if (g_ctx.corner_commit_distance_m > 0.0f)
            {
                commit_distance_m = g_ctx.corner_commit_distance_m;
            }
        }
    }
    if (!g_ctx.corner_handover_active)
    {
        g_ctx.corner_handover_idx = g_ctx.idx;
        g_ctx.corner_enter_distance_m = enter_distance_m;
        g_ctx.corner_commit_distance_m = commit_distance_m;
    }
    if (enter_distance_m <= (commit_distance_m + PATH_CORNER_HANDOVER_WINDOW_GAP_MIN_M))
    {
        enter_distance_m = commit_distance_m + PATH_CORNER_HANDOVER_WINDOW_GAP_MIN_M;
    }

    remaining_along_m = segment_length_m - segment_progress_m;
    if (remaining_along_m < 0.0f)
    {
        remaining_along_m = 0.0f;
    }

    if (!g_ctx.corner_handover_active)
    {
        if (remaining_along_m >= enter_distance_m)
        {
            return 0U;
        }
        g_ctx.corner_handover_active = 1U;
    }
        if (path_follow_check_corner_handover_timeout())
    {
        return 0U;
    }


    old_x = geometry->dir_x;
    old_y = geometry->dir_y;

    new_x = next_dx_m / next_length_m;
    new_y = next_dy_m / next_length_m;
    turn_cos = old_x * new_x + old_y * new_y;
    turn_cos = fminf(fmaxf(turn_cos, -1.0f), 1.0f);
    /* 直行时为 0，90 度及以上拐角按 1 处理。 */
    turn_severity = fminf(fmaxf(1.0f - turn_cos, 0.0f), 1.0f);

    t = (enter_distance_m - remaining_along_m) / (enter_distance_m - commit_distance_m);
    t = fminf(fmaxf(t, 0.0f), 1.0f);
    k = path_follow_s_curve_01(t);
    k_drop = PATH_CORNER_HANDOVER_K_DROP * turn_severity;
    v_scale = 1.0f - k_drop * 4.0f * k * (1.0f - k);
    v_scale = fmaxf(v_scale, 0.0f);

    mix_x = old_x * (1.0f - k) + new_x * k;
    mix_y = old_y * (1.0f - k) + new_y * k;
    mix_norm = sqrtf(mix_x * mix_x + mix_y * mix_y);
    if (mix_norm <= eps)
    {
        return 0U;
    }

    mix_x /= mix_norm;
    mix_y /= mix_norm;
    *vx_world_cmps = ref_speed_cmps * v_scale * mix_x;
    *vy_world_cmps = ref_speed_cmps * v_scale * mix_y;
    return 1U;
}
#endif

#if PATH_FOLLOW_ENABLE_LINE_GUIDE
static uint8 path_follow_apply_line_guidance(float ref_speed_cmps,
                                             float *vx_world_cmps,
                                             float *vy_world_cmps)
{
    const Position *prev_point = NULL;
    const Position *curr_point = NULL;
    float segment_progress_m = 0.0f;
    float segment_length_m = 0.0f;
    float lateral_error_m = 0.0f;
    float seg_dx_m;
    float seg_dy_m;
    float seg_norm_m;
    float tan_x;
    float tan_y;
    float normal_x;
    float normal_y;
    float effective_error_m;
    float abs_error_cm;
    float guide_sign;
    float trim_cmps;

    if (vx_world_cmps == NULL || vy_world_cmps == NULL)
    {
        return 0U;
    }
    if (!path_follow_get_segment_points(&prev_point, &curr_point, NULL) ||
        prev_point == NULL || curr_point == NULL)
    {
        return 0U;
    }
    if (!path_follow_compute_segment_progress_m(&g_ctx.pose,
                                                prev_point,
                                                curr_point,
                                                &segment_progress_m,
                                                &segment_length_m,
                                                &lateral_error_m))
    {
        return 0U;
    }

    seg_dx_m = (curr_point->row - prev_point->row) * g_ctx.grid_m;
    seg_dy_m = (curr_point->col - prev_point->col) * g_ctx.grid_m;
    seg_norm_m = sqrtf(seg_dx_m * seg_dx_m + seg_dy_m * seg_dy_m);
    if (seg_norm_m <= 1e-6f)
    {
        return 0U;
    }

    tan_x = seg_dx_m / seg_norm_m;
    tan_y = seg_dy_m / seg_norm_m;
    normal_x = -tan_y;
    normal_y = tan_x;

    if (fabsf(lateral_error_m) <= PATH_LINE_GUIDE_DEADBAND_M)
    {
        effective_error_m = 0.0f;
    }
    else if (lateral_error_m > 0.0f)
    {
        effective_error_m = lateral_error_m - PATH_LINE_GUIDE_DEADBAND_M;
    }
    else
    {
        effective_error_m = lateral_error_m + PATH_LINE_GUIDE_DEADBAND_M;
    }

    abs_error_cm = fabsf(effective_error_m * 100.0f);
    guide_sign = (effective_error_m > 0.0f) ? -1.0f : 1.0f;

    /*
     * 直线段法向纠偏采用“最小修正速度前馈 + Kp”：
     * - 误差一旦越过死区，先给一个最小侧向修正速度，帮助克服静摩擦；
     * - 再叠加与误差成正比的修正量，保持越偏越拉的趋势。
     */
    trim_cmps = PATH_LINE_GUIDE_KP * abs_error_cm;
    if (trim_cmps > 0.0f)
    {
        trim_cmps += PATH_LINE_GUIDE_MIN_CMPS;
    }
    trim_cmps *= guide_sign;
    trim_cmps = clamp_sym(trim_cmps, PATH_LINE_GUIDE_MAX_CMPS);

    *vx_world_cmps = ref_speed_cmps * tan_x + trim_cmps * normal_x;
    *vy_world_cmps = ref_speed_cmps * tan_y + trim_cmps * normal_y;
    return 1U;
}
#endif

/**
 * @brief 在接近目标点时叠加位置 PID 保持修正。
 *
 * @param geometry 当前周期路径几何信息。
 * @param motion_cmd 运动命令结构体，函数会直接在世界系平移速度上叠加修正量。
 */
static void path_follow_apply_hold_trim(const path_follow_geometry_t *geometry,
                                        path_follow_motion_cmd_t *motion_cmd)
{
    float hold_blend;

    if (geometry == NULL || motion_cmd == NULL)
    {
        return;
    }

    hold_blend = path_follow_compute_hold_blend(geometry->distance_m);
    if (hold_blend <= 0.0f)
    {
        return;
    }

#if !PATH_FOLLOW_ENABLE_CORNER_DUAL_AXIS_TRIM
    if (path_follow_is_route_corner_target())
    {
        return;
    }
#elif PATH_FOLLOW_ENABLE_CORNER_HANDOVER
    /*
     * handover 启用时，拐点附近仍保留双轴定点修正，但降低权重，
     * 避免对切向速度交接施加过强拉扯。
     */
    if (path_follow_is_route_corner_target())
    {
        hold_blend *= PATH_FOLLOW_CORNER_HOLD_TRIM_SCALE;
    }
#endif

    motion_cmd->vx_world_cmps += hold_blend *
                                 PID_Location_Calculate(&pid_stay,
                                                        g_ctx.pose.x_m * 100.0f,
                                                        geometry->target_x_m * 100.0f);
    motion_cmd->vy_world_cmps += hold_blend *
                                 PID_Location_Calculate(&pid_stay_y,
                                                        g_ctx.pose.y_m * 100.0f,
                                                        geometry->target_y_m * 100.0f);
}

/**
 * @brief 将世界系平移速度向量限制在允许的最大模长内。
 *
 * @param motion_cmd 运动命令结构体。
 */
static void path_follow_limit_world_speed(path_follow_motion_cmd_t *motion_cmd)
{
    float max_speed_cmps;
    float speed_norm;
    float scale;

    if (motion_cmd == NULL)
    {
        return;
    }

    max_speed_cmps = g_ctx.active_scurve_cfg.max_speed_cmps;
    if (max_speed_cmps <= 0.0f)
    {
        max_speed_cmps = path_follow_mps_to_cmps(g_path_follow_scurve_band_default_cfg[PATH_FOLLOW_SCURVE_BAND_COUNT - 1U].vmax_mps);
    }

    speed_norm = sqrtf(motion_cmd->vx_world_cmps * motion_cmd->vx_world_cmps +
                       motion_cmd->vy_world_cmps * motion_cmd->vy_world_cmps);
    if (speed_norm <= max_speed_cmps || speed_norm <= 0.0f)
    {
        return;
    }

    scale = max_speed_cmps / speed_norm;
    motion_cmd->vx_world_cmps *= scale;
    motion_cmd->vy_world_cmps *= scale;
}

/**
 * @brief 根据几何方向和标量速度生成运动命令。
 *
 * 该函数先在世界坐标系下生成平移速度，再叠加近目标保持修正，
 * 最后转换到车体系输出给底盘运动学逆解。
 *
 * @param geometry 当前周期路径几何信息。
 * @param speed_plan 当前周期速度规划结果。
 * @param yaw_deg 当前航向角，单位 deg。
 * @param motion_cmd 运动命令输出。
 */
static void path_follow_build_motion_command(const path_follow_geometry_t *geometry,
                                             const path_follow_speed_plan_t *speed_plan,
                                             float yaw_deg,
                                             path_follow_motion_cmd_t *motion_cmd)
{
    float yaw_rad;
    float cos_yaw;
    float sin_yaw;
    float hold_blend;
#if PATH_FOLLOW_ENABLE_CORNER_HANDOVER
    uint8 handover_active = 0U;
#endif

    if (geometry == NULL || speed_plan == NULL || motion_cmd == NULL)
    {
        return;
    }

    motion_cmd->vx_world_cmps = speed_plan->ref_speed_cmps * geometry->dir_x;
    motion_cmd->vy_world_cmps = speed_plan->ref_speed_cmps * geometry->dir_y;
    hold_blend = path_follow_compute_hold_blend(geometry->distance_m);

#if PATH_FOLLOW_ENABLE_CORNER_HANDOVER
    /*
     * [CornerHandover移植]
     * 先基于当前标量速度得到基础世界系速度，再在普通中间拐点前尝试做方向交接。
     * 后续仍保留原版的 hold trim、世界系限幅和车体系变换，不改变现有主链路能力。
     */
    handover_active = path_follow_try_corner_handover(geometry,
                                                      speed_plan->ref_speed_cmps,
                                                      &motion_cmd->vx_world_cmps,
                                                      &motion_cmd->vy_world_cmps);
#endif
#if PATH_FOLLOW_ENABLE_LINE_GUIDE
    if (
#if PATH_FOLLOW_ENABLE_CORNER_HANDOVER
        !handover_active &&
#endif
        hold_blend <= 0.0f)
    {
        (void)path_follow_apply_line_guidance(speed_plan->ref_speed_cmps,
                                              &motion_cmd->vx_world_cmps,
                                              &motion_cmd->vy_world_cmps);
    }
#endif

    if (hold_blend > 0.0f)
    {
        path_follow_apply_hold_trim(geometry, motion_cmd);
    }
    path_follow_limit_world_speed(motion_cmd);

    yaw_rad = yaw_deg * ((float)M_PI / 180.0f);
    cos_yaw = cosf(yaw_rad);
    sin_yaw = sinf(yaw_rad);

    motion_cmd->vx_body_cmps = motion_cmd->vx_world_cmps * cos_yaw +
                               motion_cmd->vy_world_cmps * sin_yaw;
    motion_cmd->vy_body_cmps = -motion_cmd->vx_world_cmps * sin_yaw +
                               motion_cmd->vy_world_cmps * cos_yaw;
}

/**
 * @brief 规划当前周期的姿态控制输出。
 *
 * @param yaw_deg 当前航向角，单位 deg。
 * @param attitude_plan 姿态规划结果输出。
 */
static void path_follow_plan_attitude(float yaw_deg, path_follow_attitude_plan_t *attitude_plan)
{
    float omega_cmd_degps;
    float yaw_error_deg;

    if (attitude_plan == NULL)
    {
        return;
    }

    attitude_plan->target_yaw_deg = g_ctx.target_yaw_deg;
    if (g_ctx.heading_mode != PATH_FOLLOW_HEADING_FIXED)
    {
        attitude_plan->target_yaw_deg = 0.0f;
    }

    attitude_plan->target_yaw_deg = path_follow_wrap_deg(attitude_plan->target_yaw_deg);
    yaw_error_deg = path_follow_yaw_error_deg(yaw_deg, attitude_plan->target_yaw_deg);
    omega_cmd_degps = PID_Location_Calculate(&pid_yaw, 0.0f, yaw_error_deg);
    omega_cmd_degps = clamp_sym(omega_cmd_degps, g_ctx.max_w_rad);
    attitude_plan->omega_cmd_radps = omega_cmd_degps * ((float)M_PI / 180.0f);
}

/**
 * @brief 保存当前周期的调试状态快照。
 *
 * @param geometry 当前周期路径几何信息。
 * @param speed_plan 当前周期速度规划结果。
 * @param attitude_plan 当前周期姿态规划结果。
 * @param motion_cmd 当前周期运动命令。
 */
static void path_follow_store_debug_state(const path_follow_geometry_t *geometry,
                                          const path_follow_speed_plan_t *speed_plan,
                                          const path_follow_attitude_plan_t *attitude_plan,
                                          const path_follow_motion_cmd_t *motion_cmd)
{
    if (geometry == NULL || speed_plan == NULL || attitude_plan == NULL || motion_cmd == NULL)
    {
        path_follow_clear_debug_state();
        return;
    }

    g_ctx.debug.distance_m = geometry->distance_m;
    g_ctx.debug.dir_x = geometry->dir_x;
    g_ctx.debug.dir_y = geometry->dir_y;
    g_ctx.debug.speed_ref_cmps = speed_plan->ref_speed_cmps;
    g_ctx.debug.target_yaw_deg = attitude_plan->target_yaw_deg;
    g_ctx.debug.vx_world_cmps = motion_cmd->vx_world_cmps;
    g_ctx.debug.vy_world_cmps = motion_cmd->vy_world_cmps;
    g_ctx.debug.segment_axis = geometry->segment_axis;
}

/**
 * @brief 路径跟随主更新函数。
 *
 * 该函数是路径跟随控制链的主入口，每个控制周期调用一次。
 * 它会完成位姿更新、目标点推进、几何分析、速度规划、姿态规划以及车体系输出合成。
 *
 * @param yaw_deg 当前航向角，单位 deg。
 * @param out 输出结构体，返回当前周期车体系速度命令与状态标志。
 */
void path_follow_update(float yaw_deg, path_follow_output_t *out)
{
    path_follow_geometry_t geometry = {0};
    path_follow_speed_plan_t speed_plan = {0};
    path_follow_attitude_plan_t attitude_plan = {0};
    path_follow_motion_cmd_t motion_cmd = {0};
    float yaw_error_deg = 0.0f;

    if (out)
    {
        out->active = 0;
        out->reached = 0;
        out->vx_cmd = 0;
        out->vy_cmd = 0;
        out->omega_cmd = 0;
        out->target_idx = g_ctx.idx;
    }

    if ((!g_ctx.active || NULL == g_ctx.path || 0 == g_ctx.steps) &&
        !g_ctx.rotate_only_active)
    {
        path_follow_clear_debug_state();
        car_direction = 0U;
        return;
    }

    update_odometry(yaw_deg);
    path_follow_sync_stay_pid_gains();

    if (g_ctx.rotate_only_active)
    {
        path_follow_plan_attitude(yaw_deg, &attitude_plan);
        yaw_error_deg = path_follow_yaw_error_deg(g_ctx.pose.yaw_deg,
                                                  attitude_plan.target_yaw_deg);

        path_follow_clear_debug_state();
        g_ctx.debug.target_yaw_deg = attitude_plan.target_yaw_deg;
        car_direction = 0U;

        if (fabsf(yaw_error_deg) <= g_ctx.yaw_tol_deg)
        {
            g_ctx.rotate_only_active = 0U;
            PID_Clear(&pid_yaw);
            path_follow_reset_control_state();
            if (out)
            {
                out->reached = 1U;
                out->active = 0U;
            }
            return;
        }

        if (out)
        {
            out->vx_cmd = 0.0f;
            out->vy_cmd = 0.0f;
            out->omega_cmd = attitude_plan.omega_cmd_radps;
            out->active = 1U;
        }
        return;
    }

    if (path_follow_handle_pause(out))
    {
        return;
    }

    if (!path_follow_prepare_geometry(&geometry, out))
    {
        if (path_follow_handle_pause(out))
        {
            return;
        }
        return;
    }

    path_follow_plan_speed(&geometry, &speed_plan);
    path_follow_build_motion_command(&geometry, &speed_plan, yaw_deg, &motion_cmd);
    path_follow_plan_attitude(yaw_deg, &attitude_plan);
    path_follow_store_debug_state(&geometry, &speed_plan, &attitude_plan, &motion_cmd);

    if (out)
    {
        out->vx_cmd = motion_cmd.vx_body_cmps;
        out->vy_cmd = motion_cmd.vy_body_cmps;
        out->omega_cmd = attitude_plan.omega_cmd_radps;
        out->active = 1U;
        out->target_idx = g_ctx.idx;
    }
}

/**
 * @brief 路径跟随测试入口。
 *
 * @param yaw_deg 当前航向角，单位 deg。
 * @param out 输出结构体。
 *
 * @note 当前实现直接复用正式更新函数。
 */
void path_follow_update_test(float yaw_deg, path_follow_output_t *out)
{
    path_follow_update(yaw_deg, out);
}

/**
 * @brief 将路径跟随输出接入到底盘运动学逆解。
 *
 * 该函数负责把 `path_follow_update()` 输出的三轴速度命令
 * 写入全局速度缓存，并进一步生成四轮目标编码器速度。
 */
void distance_speed_strategy(void)
{
    path_follow_output_t pf = {0};

    path_follow_update(eulerAngle.yaw, &pf);

    if (pf.active)
    {
        speed_three_array[0] = pf.vx_cmd;
        speed_three_array[1] = pf.vy_cmd;
        speed_three_array[2] = pf.omega_cmd;
    }
    else
    {
        speed_three_array[0] = 0.0f;
        speed_three_array[1] = 0.0f;
        speed_three_array[2] = 0.0f;
    }

    Kinematics_Inverse(speed_three_array, speed_encoder);
}

//void path_follow_request_bluetooth_report(void)
//{
//    g_path_follow_bt_report_pending = 1U;
//}

//void path_follow_service_bluetooth_report(void)
//{
//    path_follow_status_t st = {0};

//    if (!g_path_follow_bt_report_pending)
//    {
//        return;
//    }

//    g_path_follow_bt_report_pending = 0U;
//    path_follow_get_status(&st);
//    BlueSerial_Printf("POS x=%.3f y=%.3f yaw=%.1f\r\n",
//                      st.x_m,
//                      st.y_m,
//                      st.yaw_deg);
//}

/**
 * @brief 获取路径跟随模块当前状态。
 *
 * @param status 状态输出结构体。
 */
void path_follow_get_status(path_follow_status_t *status)
{
    if (NULL == status)
    {
        return;
    }

    status->x_m = g_ctx.pose.x_m;
    status->y_m = g_ctx.pose.y_m;
    status->yaw_deg = g_ctx.pose.yaw_deg;
    status->active = (g_ctx.active || g_ctx.rotate_only_active) ? 1U : 0U;
    status->reached = (status->active == 0U) ? 1U : 0U;
    status->paused = g_ctx.paused;
    status->yaw_only_active = g_ctx.rotate_only_active;
    status->target_idx = g_ctx.idx;
    status->distance_m = g_ctx.debug.distance_m;
    status->dir_x = g_ctx.debug.dir_x;
    status->dir_y = g_ctx.debug.dir_y;
    status->speed_ref_cmps = g_ctx.debug.speed_ref_cmps;
    status->target_yaw_deg = g_ctx.target_yaw_deg;
    status->segment_axis = g_ctx.debug.segment_axis;
    status->yaw_error_deg = path_follow_yaw_error_deg(g_ctx.pose.yaw_deg,
                                                      g_ctx.target_yaw_deg);

    if (g_ctx.path && g_ctx.idx < g_ctx.steps)
    {
        status->target_x_m = g_ctx.path[g_ctx.idx].row * g_ctx.grid_m;
        status->target_y_m = g_ctx.path[g_ctx.idx].col * g_ctx.grid_m;
    }
    else
    {
        status->target_x_m = 0.0f;
        status->target_y_m = 0.0f;
    }
}

/**
 * @brief 在 IPS200 屏幕上显示当前路径跟随状态。
 */
void path_follow_draw_status(void)
{
    path_follow_status_t st = {0};
    path_follow_get_status(&st);

    // ips200_show_string(x, y, "Pose x y yaw:");
    ips200_show_string(0,112,"st_x_m");
    ips200_show_float(70, 112, st.x_m, 2, 4);
    ips200_show_string(0,128,"st_y_m");
    ips200_show_float(70, 128, st.y_m, 2, 4);
    ips200_show_string(0,144,"st_yaw");
    ips200_show_float(70, 144, st.yaw_deg, 3, 2);

    // ips200_show_string(x, y + 16, "Target idx/x/y:");
    ips200_show_string(0,160,"target_idx");
    ips200_show_uint(90, 160, st.target_idx, 3);
    ips200_show_string(0,176,"target_x_m");
    ips200_show_float(90, 176, st.target_x_m, 2, 3);
    ips200_show_string(0,192,"target_y_m");
    ips200_show_float(90, 192, st.target_y_m, 2, 3);
    ips200_show_string(0,208,"speed_ref");
    ips200_show_float(90, 208, st.speed_ref_cmps, 3, 1);
    ips200_show_string(0,224,"seg_axis");
    ips200_show_uint(90, 224, st.segment_axis, 1);

}

/**
 * @brief 计算两个离散点之间的朝向角。
 *
 * @param from 起点。
 * @param to 终点。
 * @return 从起点指向终点的角度，单位 deg。
 */
float path_follow_heading_deg(Position from, Position to)
{
    float dx = (float)(to.row - from.row);
    float dy = (float)(to.col - from.col);
    if (dx == 0.0f && dy == 0.0f)
    {
        return 0.0f;
    }
    float angle_rad = atan2f(dy, dx);
    return angle_rad * (180.0f / (float)M_PI);
}

/**
 * @brief 从完整离散路径中提取拐点序列。
 *
 * @param path 原始离散路径。
 * @param path_steps 原始路径点数量。
 * @param corner_buffer 拐点输出缓存。
 * @param corner_capacity 拐点缓存容量。
 * @return 成功提取到的拐点数量。
 */
size_t path_follow_extract_corners(const Position *path, size_t path_steps,
                                   Position *corner_buffer, size_t corner_capacity)
{
    if (NULL == path || 0 == path_steps || NULL == corner_buffer || 0 == corner_capacity)
    {
        return 0;
    }

    // 仅一个点时直接返回
    if (path_steps == 1)
    {
        if (corner_capacity >= 1)
        {
            corner_buffer[0] = path[0];
            return 1;
        }
        return 0;
    }

    size_t corner_count = 0;

    // 起点一定是拐点
    if (corner_count >= corner_capacity)
    {
        return 0;  // 缓冲区不足
    }
    corner_buffer[corner_count++] = path[0];

    // 检查中间点是否拐弯
    for (size_t i = 1; i < path_steps - 1; i++)
    {
        // 上一段方向向量
        int dx1 = path[i].col - path[i - 1].col;
        int dy1 = path[i].row - path[i - 1].row;

        // 下一段方向向量
        int dx2 = path[i + 1].col - path[i].col;
        int dy2 = path[i + 1].row - path[i].row;

        // 方向变化则记为拐点
        if (dx1 != dx2 || dy1 != dy2)
        {
            if (corner_count >= corner_capacity)
            {
                return 0;  // 缓冲区不足
            }
            corner_buffer[corner_count++] = path[i];
        }
    }

    // 终点一定是拐点
    if (corner_count >= corner_capacity)
    {
        return 0;  // 缓冲区不足
    }
    corner_buffer[corner_count++] = path[path_steps - 1];

    return corner_count;
}
