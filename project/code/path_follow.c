
#include "path_follow.h"
#include "Motor.h"
#include "pid.h"
#include "Attitude.h"
#include "zf_device_ips200.h"

#include <math.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846 /* math.h 未提供时使用的圆周率常量。 */
#endif

#define PF_POSITION_TOLERANCE_M          0.015f  /* 到点位置容差，单位 m。 */
#define PF_YAW_TOLERANCE_DEG             0.30f   /* 航向到点容差，单位 deg。 */
#define PF_MAX_LINEAR_SPEED_MPS          3.0f    /* 模块允许的最大平移速度，单位 m/s。 */
#define PF_MAX_ANGULAR_SPEED_DEGPS       300.0f  /* 模块允许的最大角速度，单位 deg/s。 */
#define PF_TEMP_PATH_GRID_M              0.01f   /* 临时点路径的栅格尺寸，单位 m。 */
#define PF_MIN_VALID_GRID_M              0.0001f /* 有效距离/栅格的最小值，单位 m。 */
#define PF_INVALID_INDEX                 ((size_t)-1) /* 无效路径下标。 */
#define PF_INVALID_BAND                  0xFFU   /* 无效 S 曲线速度档位。 */
#define PF_DT_S                          (1.0f / (float)PID_RATE) /* 单次控制周期，单位 s。 */
#define PF_SEGMENT_END_SPEED_CMPS        0.0f    /* 目标中心处的规划终速度，单位 cm/s。 */
#define PF_POSITION_LOOP_RELEASE_M       0.75f   /* 近点接管距离上限；高速档按理论制动距离动态接管，单位 m。 */
#define PF_ARRIVAL_MAX_SPEED_CMPS        3.0f    /* 完成目标允许的最大实测平移速度，单位 cm/s。 */
#define PF_CROSSED_TARGET_MAX_SPEED_CMPS 3.0f    /* 越过目标平面后的切段沿程速度上限，单位 cm/s。 */
#define PF_YAW_FEEDFORWARD_DEADBAND_DEG  0.25f   /* 克服 0.25~0.5 deg 残差死区，极小误差再由浮点 PID 细调。 */
#define PF_APPROACH_DECEL_CMPS2          120.0f  /* S 曲线参数无效时的接近减速度，单位 cm/s^2。 */
#define PF_POSITION_SPEED_FACTOR_MIN     0.10f   /* 手动制动速度包络系数下限。 */
#define PF_POSITION_SPEED_FACTOR_MAX     1.00f   /* 制动速度包络系数上限。 */
#define PF_POSITION_X_SPEED_FACTOR_DEFAULT 0.95f /* X轴实时剩余距离速度上限系数。 */
#define PF_POSITION_Y_SPEED_FACTOR_DEFAULT 0.90f /* Y轴实时剩余距离速度上限系数。 */
#define PF_POSITION_RELEASE_MARGIN_M     0.05f   /* 扩大的滑移、执行及位置环接管余量，单位 m。 */
#define PF_SPEED_TEST_SETTLE_COUNTS      2       /* 纯S曲线结束后每轮允许的静止计数，count/10ms。 */
#define PF_SPEED_TEST_SETTLE_TICKS       5U      /* 四轮连续静止50ms后结束测试。 */
#define PF_SPEED_TEST_SETTLE_TIMEOUT     500U    /* 最长保留5s零速闭环尾段。 */

/* Y-to-X crosstalk feedforward boot defaults.
 * Unit: X correction / absolute Y command. The gains are signed and
 * independent: +Y is left, -Y is right. X-only commands are unaffected. */
#define PF_Y_CROSSTALK_LEFT_X_COMP_K      0.005f /* 多组左移原始串轴均值接近 0，先不补偿。 */
#define PF_Y_CROSSTALK_RIGHT_X_COMP_K     -0.016f /* 多组右移原始串轴均值接近 0，先不补偿。 */
#define PF_LATERAL_LEFT_ODOMETRY_SCALE    1.0090f /* 三地图区域五组往返均值：左移里程约多 0.9%。 */

#define PF_POSITION_X_KP                 3.7f    /* 车体 X 前进直走位置环。 */
#define PF_POSITION_X_KI                 0.0f
#define PF_POSITION_X_KD                 5.9f
#define PF_POSITION_Y_KP                 5.4f    /* 车体 Y 左右侧移位置环。 */
#define PF_POSITION_Y_KI                 0.0f
#define PF_POSITION_Y_KD                 7.7f
#define PF_POSITION_BACKWARD_X_KP        PF_POSITION_X_KP /* 后退 X 位置环独立默认值。 */
#define PF_POSITION_BACKWARD_X_KI        PF_POSITION_X_KI
#define PF_POSITION_BACKWARD_X_KD        PF_POSITION_X_KD
#define PF_POSITION_X_MAX_IOUT_CMPS      200.0f  /* X轴位置环积分输出上限，单位 cm/s。 */
#define PF_POSITION_X_MAX_OUT_CMPS       200.0f  /* X轴位置环总输出上限，单位 cm/s。 */
#define PF_POSITION_Y_MAX_IOUT_CMPS      200.0f  /* Y轴位置环积分输出上限，单位 cm/s。 */
#define PF_POSITION_Y_MAX_OUT_CMPS       200.0f  /* Y轴位置环总输出上限，单位 cm/s。 */
#define PF_POSITION_FILTER_ALPHA         0.9f    /* 位置环微分滤波系数。 */
#define PF_LINE_GUIDE_KP                 1.0f    /* 默认启用法向纠偏，单位 (cm/s)/cm。 */
#define PF_LINE_GUIDE_MIN_CMPS           4.0f    /* 越过死区后的最小法向纠偏速度，单位 cm/s。 */
#define PF_LINE_GUIDE_MAX_CMPS           12.0f   /* 法向纠偏速度上限，单位 cm/s。 */
#define PF_LINE_GUIDE_DEADBAND_M         0.0025f /* 法向偏差死区，单位 m。 */
#define PF_LINE_GUIDE_START_BOOST        2.0f    /* 首段刚起步时的法向纠偏倍率。 */
#define PF_LINE_GUIDE_START_BOOST_M      0.03f   /* 前 20 cm 内将倍率平滑退回 1。 */

#define PF_YAW_KP                        7.0f    /* 高加速度平移保持环：提高负载扰动抑制。 */
#define PF_YAW_KI                        0.00f  /* 平移保持环积分：抵消持续横移负载转矩。 */
#define PF_YAW_KD                        16.0f   /* 抑制高速横移偏航，避免更高增益下的末段反摆。 */
#define PF_YAW_FILTER_ALPHA              0.9f    /* 航向环微分滤波系数。 */
#define PF_YAW_FEEDFORWARD_MIN_DEGPS     8.0f    /* 克服低速转向死区的最小角速度，单位 deg/s。 */
#define PF_ROTATE_YAW_KP                 6.0f    /* 原地旋转专用比例系数。 */
#define PF_ROTATE_YAW_KI                 0.0f    /* 原地旋转专用积分系数。 */
#define PF_ROTATE_YAW_KD                 8.5f    /* 原地旋转专用微分系数。 */
#define PF_ROTATE_YAW_FEEDFORWARD_DEGPS  8.0f    /* 原地旋转低速前馈，单位 deg/s。 */
#define PF_ROTATE_POSITION_HOLD_KP_CMPS_PER_CM 1.5f /* 每偏移 1 cm 产生的反向平移速度，单位 cm/s。 */
#define PF_ROTATE_POSITION_HOLD_MAX_CMPS 12.0f  /* 旋转位置保持最大平移纠偏速度，单位 cm/s。 */
#define PF_ROTATE_POSITION_HOLD_DEADBAND_CM 0.3f /* 旋转位置保持死区，避免编码器噪声引起抖动。 */

typedef struct
{
    float x_m;
    float y_m;
    float yaw_deg;
} pf_pose_t;

typedef struct
{
    float target_x_m;
    float target_y_m;
    float dx_m;
    float dy_m;
    float distance_m;
    float dir_x;
    float dir_y;
    float planned_distance_m;
    float along_track_remaining_m;
    uint8 within_tolerance;
    uint8 target_plane_crossed;
    uint8 segment_axis;
} pf_geometry_t;

typedef struct
{
    float distance_m;
    float dir_x;
    float dir_y;
    float speed_ref_cmps;
    float speed_cap_cmps;
    float target_x_m;
    float target_y_m;
    float position_speed_limit_cmps;
    float actual_along_speed_cmps;
    float position_blend;
    uint8 speed_limit_active;
    uint8 overspeed_guard_active;
    uint8 position_reverse_requested;
    uint8 target_plane_crossed;
    uint8 position_loop_active;
    uint8 line_guidance_active;
    uint8 segment_axis;
} pf_debug_t;

typedef struct
{
    uint8 band_idx;
    float max_speed_cmps;
    float accel_cmpss;
    float jerk_cmpsss;
} pf_scurve_runtime_cfg_t;

typedef struct
{
    float s;
    float x0;
    float x1;
    float v0;
    float v1;
    float vmax;
    float amax;
    float jmax;
    float vlim;
    float alima;
    float alimd;
    float Tj1;
    float Ta;
    float Tv;
    float Tj2;
    float Td;
    float T;
    uint8 valid;
} pf_scurve_profile_t;

typedef struct
{
    float safety_cap_cmps;
    float end_speed_cmps;
    float ref_speed_cmps;
} pf_speed_plan_t;

typedef struct
{
    const Position *path;
    size_t steps;
    size_t target_idx;

    float default_grid_m;
    float path_grid_m;
    float path_origin_x_m;
    float path_origin_y_m;
    float pulses_per_meter;
    float target_yaw_deg;
    float position_tolerance_m;
    float yaw_tolerance_deg;
    float max_linear_speed_mps;
    float max_angular_speed_degps;
    float linear_speed_cmps;
    float velocity_x_world_cmps;
    float velocity_y_world_cmps;

    pf_pose_t pose;
    pf_debug_t debug;
    pf_scurve_runtime_cfg_t active_scurve_cfg;
    pf_scurve_profile_t active_profile;
    float profile_time_s;
    float last_ref_speed_cmps;
    size_t profile_target_idx;

    float position_blend;

    size_t pause_indices[PATH_FOLLOW_MAX_PAUSE_POINTS];
    size_t pause_count;
    size_t pause_cursor;
    uint32 pause_cycles_cfg;
    uint32 pause_cycles_left;
    uint8 active;
    uint8 paused;
    uint8 pause_events_enabled;
    uint8 rotate_only_active;
    float rotate_hold_x_m;
    float rotate_hold_y_m;
    uint8 profile_active;
    uint8 segment_axis_override;
} pf_context_t;

static pf_context_t g_pf;
static Position g_single_target_path[2];
static Position g_offset_path[3];
static Position g_pose_move_path[3];
static uint8 g_stationary_yaw_hold_enabled;
static volatile uint8 g_bluetooth_report_pending;
static uint8 g_speed_test_enabled;
static uint8 g_speed_test_yaw_enabled;
static uint8 g_speed_test_settling;
static uint16 g_speed_test_stable_ticks;
static uint16 g_speed_test_settle_ticks;
static uint8 g_speed_test_settle_timeout;
static uint8 g_speed_test_profile_fault;
static float g_speed_test_frame_yaw_deg;

/* Public compatibility objects declared by path_follow.h. */
tagPID_T pid_world_x;
tagPID_T pid_world_y;
tagPID_T pid_stay;
tagPID_T pid_stay_y;
tagPID_T pid_stay_backward;
tagPID_T pid_yaw;
static tagPID_T pid_yaw_rotate;
tagPID_T pid_accel_yaw;

uint8 car_direction = 0U;
uint8 wait_stop = 0U;

float prestart_move_left_m = 0.12f;
float prestart_move_right_m = 0.00f;
float prestart_move_forward_m = 0.35f;
float prestart_move_backward_m = 0.00f;

/* Position-loop tuning entry retained from the original module. */
float path_corner_commit_lateral_gate_min_m = PF_POSITION_TOLERANCE_M;
float path_hold_trim_release_distance = PF_POSITION_LOOP_RELEASE_M;

/* Runtime-tunable line guidance and yaw low-speed feedforward. */
float path_line_guide_kp = PF_LINE_GUIDE_KP;
float path_line_guide_min_cmps = PF_LINE_GUIDE_MIN_CMPS;
static float path_position_speed_limit_factor_x = PF_POSITION_X_SPEED_FACTOR_DEFAULT;
static float path_position_speed_limit_factor_y = PF_POSITION_Y_SPEED_FACTOR_DEFAULT;
float path_yaw_feedforward_min_degps = PF_YAW_FEEDFORWARD_MIN_DEGPS;
float path_yaw_feedforward_deadband_deg = PF_YAW_FEEDFORWARD_DEADBAND_DEG;
static float path_rotate_yaw_feedforward_degps = PF_ROTATE_YAW_FEEDFORWARD_DEGPS;

/* Runtime copies of the boot defaults; BlueSerial sliders tune these. */
float path_y_crosstalk_left_x_comp_k = PF_Y_CROSSTALK_LEFT_X_COMP_K;
float path_y_crosstalk_right_x_comp_k = PF_Y_CROSSTALK_RIGHT_X_COMP_K;

/* Other legacy compensation variables remain link-compatible but unused. */
float path_yaw_target_base_comp_deg = 0.0f;
float path_yaw_target_error_comp_k = 0.0f;
float path_rotate_center_offset_x_cm = 0.0f;
float path_rotate_center_offset_y_cm = 0.0f;

static const path_follow_scurve_band_cfg_t g_default_speed_bands[PATH_FOLLOW_SCURVE_BAND_COUNT] =
{
    {0.30f,   0.40f, 1.20f, 5.00f},
    {0.50f,   0.60f, 1.20f, 5.00f},
    {0.70f,   0.80f, 1.20f, 5.00f},
    {0.90f,   0.95f, 1.20f, 5.00f},
    {130.0f, 1.30f, 1.20f, 5.00f}
};

path_follow_scurve_band_cfg_t g_path_follow_scurve_band_cfg[PATH_FOLLOW_SCURVE_BAND_COUNT] =
{ {0.30f,   0.40f, 1.20f, 5.00f},
    {0.50f,   0.60f, 1.20f, 5.00f},
    {0.70f,   0.80f, 1.20f, 5.00f},
    {0.90f,   0.95f, 1.20f, 5.00f},
    {130.0f, 1.30f, 1.20f, 5.00f}
};

static float pf_clamp(float value, float min_value, float max_value)
{
    if (value < min_value)
    {
        return min_value;
    }
    if (value > max_value)
    {
        return max_value;
    }
    return value;
}

static float pf_y_crosstalk_x_comp(float vy)
{
    if (vy > 0.0f)
    {
        return path_y_crosstalk_left_x_comp_k * fabsf(vy);
    }
    if (vy < 0.0f)
    {
        return path_y_crosstalk_right_x_comp_k * fabsf(vy);
    }
    return 0.0f;
}

static float pf_wrap_deg(float angle_deg)
{
    while (angle_deg > 180.0f)
    {
        angle_deg -= 360.0f;
    }
    while (angle_deg < -180.0f)
    {
        angle_deg += 360.0f;
    }
    return angle_deg;
}

static float pf_cardinal_target_deg(float angle_deg)
{
    uint8 quarter;

    angle_deg = fmodf(angle_deg, 360.0f);
    if (angle_deg < 0.0f)
    {
        angle_deg += 360.0f;
    }
    quarter = (uint8)floorf((angle_deg + 45.0f) / 90.0f);
    if (quarter >= 4U)
    {
        quarter = 0U;
    }
    return (float)quarter * 90.0f;
}

static float pf_yaw_error_deg(float current_deg, float target_deg)
{
    return pf_wrap_deg(target_deg - current_deg);
}

static void pf_clear_output(path_follow_output_t *out)
{
    if (out == NULL)
    {
        return;
    }

    memset(out, 0, sizeof(*out));
    out->target_idx = g_pf.target_idx;
}

static void pf_clear_debug(void)
{
    memset(&g_pf.debug, 0, sizeof(g_pf.debug));
}

static void pf_reset_position_transition(void)
{
    g_pf.position_blend = 0.0f;
    PID_Clear(&pid_stay);
    PID_Clear(&pid_stay_y);
}

static void pf_reset_speed(void)
{
    memset(&g_pf.active_profile, 0, sizeof(g_pf.active_profile));
    memset(&g_pf.active_scurve_cfg, 0, sizeof(g_pf.active_scurve_cfg));
    g_pf.active_scurve_cfg.band_idx = PF_INVALID_BAND;
    g_pf.profile_time_s = 0.0f;
    g_pf.last_ref_speed_cmps = 0.0f;
    g_pf.profile_target_idx = PF_INVALID_INDEX;
    g_pf.profile_active = 0U;
    pf_reset_position_transition();
}

static void pf_invalidate_active_profile(void)
{
    memset(&g_pf.active_profile, 0, sizeof(g_pf.active_profile));
    g_pf.profile_time_s = 0.0f;
    g_pf.profile_target_idx = PF_INVALID_INDEX;
    g_pf.profile_active = 0U;
}

static void pf_reset_motion_state(void)
{
    pf_reset_speed();
    pf_clear_debug();
    g_speed_test_settling = 0U;
    g_speed_test_stable_ticks = 0U;
    g_speed_test_settle_ticks = 0U;
    PID_Clear(&pid_world_x);
    PID_Clear(&pid_world_y);
    PID_Clear(&pid_stay);
    PID_Clear(&pid_stay_y);
    PID_Clear(&pid_stay_backward);
    car_direction = 0U;
}

static void pf_point_to_world(Position point, float grid_m, float *x_m, float *y_m)
{
    if (x_m != NULL)
    {
        *x_m = g_pf.path_origin_x_m + (float)point.row * grid_m;
    }
    if (y_m != NULL)
    {
        *y_m = g_pf.path_origin_y_m + (float)point.col * grid_m;
    }
}

static uint8 pf_segment_axis(float dx_world_m, float dy_world_m)
{
    const float axis_epsilon_m = 0.001f;
    float yaw_rad = g_pf.target_yaw_deg * ((float)M_PI / 180.0f);
    float dx_body_m = dx_world_m * cosf(yaw_rad) +
                      dy_world_m * sinf(yaw_rad);
    float dy_body_m = -dx_world_m * sinf(yaw_rad) +
                       dy_world_m * cosf(yaw_rad);

    if (fabsf(dx_body_m) <= axis_epsilon_m &&
        fabsf(dy_body_m) <= axis_epsilon_m)
    {
        return 0U;
    }
    if (fabsf(dy_body_m) <= axis_epsilon_m)
    {
        return PATH_FOLLOW_AXIS_X;
    }
    if (fabsf(dx_body_m) <= axis_epsilon_m)
    {
        return PATH_FOLLOW_AXIS_Y;
    }
    return 3U;
}

static uint32 pf_ms_to_cycles(uint32 duration_ms)
{
    uint32 cycles = (duration_ms * (uint32)PID_RATE + 999U) / 1000U;
    return (cycles > 0U) ? cycles : 1U;
}

void path_follow_reset_scurve_band_defaults(void)
{
    size_t i;

    for (i = 0U; i < PATH_FOLLOW_SCURVE_BAND_COUNT; ++i)
    {
        g_path_follow_scurve_band_cfg[i] = g_default_speed_bands[i];
    }
}

void path_follow_sanitize_scurve_band_cfg(void)
{
    uint8 i;

    for (i = 0U; i < PATH_FOLLOW_SCURVE_BAND_COUNT; ++i)
    {
        path_follow_scurve_band_cfg_t *band = &g_path_follow_scurve_band_cfg[i];

        band->distance_upper_m = g_default_speed_bands[i].distance_upper_m;
        if (!(band->vmax_mps > 0.0f))
        {
            band->vmax_mps = g_default_speed_bands[i].vmax_mps;
        }
        if (!(band->amax_mps2 > 0.0f))
        {
            band->amax_mps2 = g_default_speed_bands[i].amax_mps2;
        }
        if (!(band->jmax_mps3 > 0.0f))
        {
            band->jmax_mps3 = g_default_speed_bands[i].jmax_mps3;
        }
    }
}

static float pf_mps_to_cmps(float value_mps)
{
    return value_mps * 100.0f;
}

static float pf_mps2_to_cmpss(float value_mps2)
{
    return value_mps2 * 100.0f;
}

static float pf_mps3_to_cmpsss(float value_mps3)
{
    return value_mps3 * 100.0f;
}

static void pf_select_scurve_band(float distance_m,
                                  pf_scurve_runtime_cfg_t *runtime_cfg)
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
            runtime_cfg->max_speed_cmps = pf_mps_to_cmps(g_path_follow_scurve_band_cfg[i].vmax_mps);
            runtime_cfg->accel_cmpss = pf_mps2_to_cmpss(g_path_follow_scurve_band_cfg[i].amax_mps2);
            runtime_cfg->jerk_cmpsss = pf_mps3_to_cmpsss(g_path_follow_scurve_band_cfg[i].jmax_mps3);
            return;
        }
    }

    runtime_cfg->band_idx = (uint8)(PATH_FOLLOW_SCURVE_BAND_COUNT - 1U);
    runtime_cfg->max_speed_cmps = pf_mps_to_cmps(g_path_follow_scurve_band_cfg[PATH_FOLLOW_SCURVE_BAND_COUNT - 1U].vmax_mps);
    runtime_cfg->accel_cmpss = pf_mps2_to_cmpss(g_path_follow_scurve_band_cfg[PATH_FOLLOW_SCURVE_BAND_COUNT - 1U].amax_mps2);
    runtime_cfg->jerk_cmpsss = pf_mps3_to_cmpsss(g_path_follow_scurve_band_cfg[PATH_FOLLOW_SCURVE_BAND_COUNT - 1U].jmax_mps3);
}

/* Original seven-segment jerk-limited S-curve equations. */
static uint8 pf_build_scurve_profile(pf_scurve_profile_t *profile,
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
    uint16 iteration_count;

    if (profile == NULL)
    {
        return 0U;
    }

    memset(profile, 0, sizeof(*profile));
    if (distance_cm <= 0.0f || v_max_cmps <= 0.0f ||
        a_max_cmpss <= 0.0f || j_max_cmpsss <= 0.0f)
    {
        return 0U;
    }

    v0_cmps = fminf(fmaxf(v0_cmps, 0.0f), v_max_cmps);
    v1_cmps = fminf(fmaxf(v1_cmps, 0.0f), v_max_cmps);

    T1 = sqrtf(fabsf(v1_cmps - v0_cmps) / j_max_cmpsss);
    T2 = v_max_cmps / j_max_cmpsss;
    Tjs = fminf(T1, T2);
    if ((T1 <= T2 && distance_cm < Tjs * (v0_cmps + v1_cmps)) ||
        (T1 > T2 && distance_cm < 0.5f * (v0_cmps + v1_cmps) *
                                   (Tjs + fabsf(v1_cmps - v0_cmps) / a_max_cmpss)))
    {
        return 0U;
    }

    if ((v_max_cmps - v0_cmps) * j_max_cmpsss < a_max_cmpss * a_max_cmpss)
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

    if ((v_max_cmps - v1_cmps) * j_max_cmpsss < a_max_cmpss * a_max_cmpss)
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
    iteration_count = 0U;
    while ((Ta < 2.0f * Tj || Td < 2.0f * Tj) && iteration_count < 1000U)
    {
        ++iteration_count;
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
                    sqrtf(fmaxf(j_max_cmpsss *
                                (j_max_cmpsss * distance_cm * distance_cm +
                                 (v1_cmps + v0_cmps) * (v1_cmps + v0_cmps) *
                                 (v1_cmps - v0_cmps)), 0.0f));
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
                    sqrtf(fmaxf(j_max_cmpsss *
                                (j_max_cmpsss * distance_cm * distance_cm -
                                 (v1_cmps + v0_cmps) * (v1_cmps + v0_cmps) *
                                 (v1_cmps - v0_cmps)), 0.0f));
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

    if (Ta < 2.0f * Tj || Td < 2.0f * Tj)
    {
        return 0U;
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

static float pf_sample_scurve_velocity(float t_s,
                                       const pf_scurve_profile_t *profile)
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
    if (t_s < profile->Ta - profile->Tj1)
    {
        return profile->v0 + profile->alima * (t_s - profile->Tj1 * 0.5f);
    }
    if (t_s < profile->Ta)
    {
        float dt = profile->Ta - t_s;
        return profile->vlim - profile->jmax * dt * dt * 0.5f;
    }
    if (t_s < profile->Ta + profile->Tv)
    {
        return profile->vlim;
    }
    if (t_s < profile->T - profile->Td + profile->Tj2)
    {
        float dt = t_s - profile->T + profile->Td;
        return profile->vlim - profile->jmax * dt * dt * 0.5f;
    }
    if (t_s < profile->T - profile->Tj2)
    {
        return profile->vlim - profile->alimd *
               (t_s - profile->T + profile->Td - profile->Tj2 * 0.5f);
    }
    {
        float dt = profile->T - t_s;
        return profile->v1 + profile->jmax * dt * dt * 0.5f;
    }
}

static float pf_profile_fault_speed(float current_speed_cmps,
                                    float safety_speed_cmps,
                                    float accel_cmpss)
{
    float accel_step = fmaxf(accel_cmpss, 1.0f) / (float)PID_RATE;

    if (current_speed_cmps < safety_speed_cmps)
    {
        current_speed_cmps = fminf(current_speed_cmps + accel_step,
                                   safety_speed_cmps);
    }
    else
    {
        current_speed_cmps = fmaxf(current_speed_cmps - accel_step,
                                   safety_speed_cmps);
    }
    return fmaxf(current_speed_cmps, 0.0f);
}

static uint8 pf_target_needs_pause(void);
static void pf_apply_rotate_position_hold(float yaw_deg,
                                          float *vx_body_cmps,
                                          float *vy_body_cmps);

static float pf_segment_end_speed_cmps(void)
{
    if ((g_pf.target_idx + 1U) >= g_pf.steps || pf_target_needs_pause())
    {
        return 0.0f;
    }
    return PF_SEGMENT_END_SPEED_CMPS;
}

static uint8 pf_build_active_profile(const pf_geometry_t *geometry,
                                     pf_speed_plan_t *speed_plan)
{
    pf_scurve_profile_t profile = {0};
    pf_scurve_runtime_cfg_t selected_cfg = {0};
    float start_speed_cmps;
    float profile_distance_m;

    if (geometry == NULL || speed_plan == NULL)
    {
        return 0U;
    }

    /* Build from the distance that is actually left, so a recovery profile
     * does not repeat the original full-segment timing. */
    profile_distance_m = g_speed_test_enabled ?
                         geometry->planned_distance_m : geometry->distance_m;
    if (profile_distance_m <= PF_MIN_VALID_GRID_M)
    {
        profile_distance_m = geometry->distance_m;
    }
    pf_select_scurve_band(profile_distance_m, &selected_cfg);
    g_pf.active_scurve_cfg = selected_cfg;

    speed_plan->end_speed_cmps = fminf(pf_segment_end_speed_cmps(),
                                       g_pf.active_scurve_cfg.max_speed_cmps);
    /* The S-curve is feedforward only.  The single real-time distance limit is
     * applied after S/position blending, so no second braking envelope acts
     * here. */
    speed_plan->safety_cap_cmps = g_pf.active_scurve_cfg.max_speed_cmps;
    start_speed_cmps = g_speed_test_enabled ? 0.0f :
        fminf(fmaxf(g_pf.last_ref_speed_cmps, 0.0f),
              speed_plan->safety_cap_cmps);

    if (!pf_build_scurve_profile(&profile,
                                 profile_distance_m * 100.0f,
                                 start_speed_cmps,
                                 speed_plan->end_speed_cmps,
                                 g_pf.active_scurve_cfg.max_speed_cmps,
                                 g_pf.active_scurve_cfg.accel_cmpss,
                                 g_pf.active_scurve_cfg.jerk_cmpsss))
    {
        pf_invalidate_active_profile();
        return 0U;
    }

    g_pf.active_profile = profile;
    g_pf.profile_time_s = 0.0f;
    g_pf.profile_target_idx = g_pf.target_idx;
    g_pf.profile_active = 1U;
    speed_plan->end_speed_cmps = profile.v1;
    return 1U;
}

static void pf_plan_scurve_speed(const pf_geometry_t *geometry,
                                 pf_speed_plan_t *speed_plan)
{
    uint8 build_ok = 1U;

    if (geometry == NULL || speed_plan == NULL)
    {
        return;
    }

    memset(speed_plan, 0, sizeof(*speed_plan));
    speed_plan->end_speed_cmps = pf_segment_end_speed_cmps();
    if (!g_pf.profile_active || g_pf.profile_target_idx != g_pf.target_idx)
    {
        build_ok = pf_build_active_profile(geometry, speed_plan);
    }
    else
    {
        speed_plan->end_speed_cmps = g_pf.active_profile.v1;
        speed_plan->safety_cap_cmps = g_pf.active_scurve_cfg.max_speed_cmps;
    }

    if (g_pf.profile_active)
    {
        g_pf.profile_time_s += PF_DT_S;
        if (g_pf.profile_time_s > g_pf.active_profile.T)
        {
            g_pf.profile_time_s = g_pf.active_profile.T;
        }
        speed_plan->ref_speed_cmps = pf_sample_scurve_velocity(g_pf.profile_time_s,
                                                                &g_pf.active_profile);
        speed_plan->end_speed_cmps = g_pf.active_profile.v1;
        speed_plan->safety_cap_cmps = g_pf.active_scurve_cfg.max_speed_cmps;
    }
    else if (!build_ok)
    {
        if (g_speed_test_enabled)
        {
            g_speed_test_profile_fault = 1U;
            g_pf.active = 0U;
            speed_plan->ref_speed_cmps = 0.0f;
            speed_plan->safety_cap_cmps = 0.0f;
            g_pf.last_ref_speed_cmps = 0.0f;
            return;
        }
        speed_plan->ref_speed_cmps = pf_profile_fault_speed(g_pf.last_ref_speed_cmps,
                                                             speed_plan->safety_cap_cmps,
                                                             g_pf.active_scurve_cfg.accel_cmpss);
    }

    speed_plan->ref_speed_cmps = pf_clamp(speed_plan->ref_speed_cmps,
                                           -g_pf.active_scurve_cfg.max_speed_cmps,
                                           g_pf.active_scurve_cfg.max_speed_cmps);
    if (!g_speed_test_enabled)
    {
        speed_plan->ref_speed_cmps = fminf(speed_plan->ref_speed_cmps,
                                            speed_plan->safety_cap_cmps);
    }
    speed_plan->ref_speed_cmps = fmaxf(speed_plan->ref_speed_cmps, 0.0f);

    if (!g_speed_test_enabled &&
        g_pf.profile_active && g_pf.active_profile.T > 0.0f &&
        g_pf.profile_time_s >= g_pf.active_profile.T &&
        geometry->distance_m > fmaxf(g_pf.position_tolerance_m,
                                     path_hold_trim_release_distance))
    {
        /* The timed profile ended too early (slip/load/odometry mismatch).
         * Invalidate it; the next control tick replans from the remaining
         * distance instead of staying at a zero speed reference. */
        g_pf.profile_active = 0U;
        g_pf.active_profile.valid = 0U;
        g_pf.profile_time_s = 0.0f;
        g_pf.last_ref_speed_cmps = 0.0f;
        speed_plan->ref_speed_cmps = 0.0f;
        return;
    }
    if (!g_speed_test_enabled &&
        geometry->distance_m <= g_pf.position_tolerance_m)
    {
        speed_plan->ref_speed_cmps = 0.0f;
    }
    g_pf.last_ref_speed_cmps = speed_plan->ref_speed_cmps;
}

static float pf_position_speed_factor_for_axis(uint8 segment_axis)
{
    float factor;

    if (segment_axis == 2U)
    {
        factor = path_position_speed_limit_factor_y;
    }
    else if (segment_axis == 3U)
    {
        /* A diagonal segment is limited by the more conservative axis. */
        factor = fminf(path_position_speed_limit_factor_x,
                       path_position_speed_limit_factor_y);
    }
    else
    {
        factor = path_position_speed_limit_factor_x;
    }
    return pf_clamp(factor,
                    PF_POSITION_SPEED_FACTOR_MIN,
                    PF_POSITION_SPEED_FACTOR_MAX);
}

static float pf_position_speed_limit_cmps(float distance_m,
                                          uint8 segment_axis)
{
    float decel_cmpss;
    float braking_distance_cm;
    float physical_limit_cmps;
    float max_speed_cmps;
    float retained_speed_cmps;
    float factor;

    decel_cmpss = (g_pf.active_scurve_cfg.accel_cmpss > 0.0f) ?
                  g_pf.active_scurve_cfg.accel_cmpss :
                  PF_APPROACH_DECEL_CMPS2;
    braking_distance_cm = fmaxf(distance_m, 0.0f) * 100.0f;
    retained_speed_cmps = fmaxf(pf_segment_end_speed_cmps(), 0.0f);
    physical_limit_cmps = sqrtf(retained_speed_cmps * retained_speed_cmps +
                                2.0f * fmaxf(decel_cmpss, 1.0f) *
                                braking_distance_cm);

    max_speed_cmps = (g_pf.active_scurve_cfg.max_speed_cmps > 0.0f) ?
                     g_pf.active_scurve_cfg.max_speed_cmps :
                     g_pf.max_linear_speed_mps * 100.0f;
    physical_limit_cmps = fminf(physical_limit_cmps,
                                fmaxf(max_speed_cmps, retained_speed_cmps));
    factor = pf_position_speed_factor_for_axis(segment_axis);

    /* Scaling only the braking headroom keeps the retained speed exact when
     * a non-zero pass-through speed is enabled later.  It is zero for now. */
    return retained_speed_cmps +
           factor * (physical_limit_cmps - retained_speed_cmps);
}

static void pf_init_pid_object(tagPID_T *pid,
                               float kp,
                               float ki,
                               float kd,
                               float max_output,
                               float alpha)
{
    PIDInitStruct init = {0};

    init.fKp = kp;
    init.fKi = ki;
    init.fKd = kd;
    init.fMax_Iout = max_output;
    init.fMax_Out = max_output;
    init.alpha = alpha;
    PID_Init(pid, &init);
}

static float pf_position_loop_release_m(const pf_geometry_t *geometry)
{
    float max_speed_cmps;
    float decel_cmpss;
    float braking_distance_m;
    float release_m;

    if (geometry == NULL)
    {
        return path_hold_trim_release_distance;
    }

    max_speed_cmps = fmaxf(g_pf.active_scurve_cfg.max_speed_cmps, 0.0f);
    decel_cmpss = (g_pf.active_scurve_cfg.accel_cmpss > 0.0f) ?
                  g_pf.active_scurve_cfg.accel_cmpss :
                  PF_APPROACH_DECEL_CMPS2;
    braking_distance_m = (max_speed_cmps * max_speed_cmps) /
                         (2.0f * decel_cmpss) * 0.01f;
    release_m = braking_distance_m + PF_POSITION_RELEASE_MARGIN_M;
    release_m = fminf(release_m, path_hold_trim_release_distance);
    if (geometry->planned_distance_m > PF_MIN_VALID_GRID_M)
    {
        release_m = fminf(release_m, geometry->planned_distance_m);
    }
    return fmaxf(release_m, g_pf.position_tolerance_m);
}

static float pf_position_loop_blend(const pf_geometry_t *geometry)
{
    float distance_m;
    float release_m;
    float blend;

    if (geometry == NULL)
    {
        return 0.0f;
    }
    distance_m = fmaxf(geometry->along_track_remaining_m, 0.0f);
    release_m = pf_position_loop_release_m(geometry);

    if (distance_m >= release_m)
    {
        blend = 0.0f;
    }
    else if (distance_m <= g_pf.position_tolerance_m)
    {
        blend = 1.0f;
    }
    else if (release_m <= g_pf.position_tolerance_m)
    {
        blend = 1.0f;
    }
    else
    {
        blend = 1.0f - ((distance_m - g_pf.position_tolerance_m) /
                        (release_m - g_pf.position_tolerance_m));
    }

    /* Once position control starts taking ownership of a segment it must not
     * hand control back to the timed profile because of odometry noise. */
    g_pf.position_blend = fmaxf(g_pf.position_blend,
                                pf_clamp(blend, 0.0f, 1.0f));
    return g_pf.position_blend;
}

static void pf_apply_position_loop(const pf_geometry_t *geometry,
                                   float *vx_world_cmps,
                                   float *vy_world_cmps,
                                   float blend)
{
    tagPID_T *position_pid_x;
    float error_x_world_cm;
    float error_y_world_cm;
    float error_x_body_cm;
    float error_y_body_cm;
    float position_vx_body_cmps;
    float position_vy_body_cmps;
    float position_vx_cmps;
    float position_vy_cmps;
    float speed_weight;
    float target_yaw_rad;
    float pose_yaw_rad;
    float cos_pose_yaw;
    float sin_pose_yaw;
    float forward_projection;
    uint8 backward_segment;

    if (geometry == NULL || vx_world_cmps == NULL || vy_world_cmps == NULL)
    {
        return;
    }

    if (blend <= 0.0f)
    {
        return;
    }

    target_yaw_rad = g_pf.target_yaw_deg * ((float)M_PI / 180.0f);
    forward_projection = geometry->dir_x * cosf(target_yaw_rad) +
                         geometry->dir_y * sinf(target_yaw_rad);
    backward_segment = (forward_projection < -0.5f) ? 1U : 0U;
    position_pid_x = backward_segment ? &pid_stay_backward : &pid_stay;

    /* Position is stored in the world frame, but X/Y tuning represents the
     * mecanum body axes: X is straight motion and Y is lateral motion. */
    pose_yaw_rad = g_pf.pose.yaw_deg * ((float)M_PI / 180.0f);
    cos_pose_yaw = cosf(pose_yaw_rad);
    sin_pose_yaw = sinf(pose_yaw_rad);
    error_x_world_cm = geometry->dx_m * 100.0f;
    error_y_world_cm = geometry->dy_m * 100.0f;
    error_x_body_cm = error_x_world_cm * cos_pose_yaw +
                      error_y_world_cm * sin_pose_yaw;
    error_y_body_cm = -error_x_world_cm * sin_pose_yaw +
                       error_y_world_cm * cos_pose_yaw;

    /* PID_Location_Calculate() keeps its historical int return type for
     * compatibility with the rest of the project.  Its internal controller
     * output is float, however, and the near-target position loop needs that
     * resolution: truncating here used to discard every correction below
     * 1 cm/s and produced a measurable final-position quantisation. */
    (void)PID_Location_Calculate(position_pid_x,
                                 0.0f,
                                 error_x_body_cm);
    position_vx_body_cmps = position_pid_x->fCtrl_Out;

    (void)PID_Location_Calculate(&pid_stay_y,
                                 0.0f,
                                 error_y_body_cm);
    position_vy_body_cmps = pid_stay_y.fCtrl_Out;

    /* The downstream blend and one-way limiter operate in the world frame. */
    position_vx_cmps = position_vx_body_cmps * cos_pose_yaw -
                       position_vy_body_cmps * sin_pose_yaw;
    position_vy_cmps = position_vx_body_cmps * sin_pose_yaw +
                       position_vy_body_cmps * cos_pose_yaw;

    blend = pf_clamp(blend, 0.0f, 1.0f);
    speed_weight = 1.0f - blend;

    /* Position PID outputs are velocity targets.  Cross-fading them with the
     * S-curve avoids the old double-command behavior where stronger position
     * gains could add speed on top of an already moving S reference. */
    *vx_world_cmps = speed_weight * *vx_world_cmps +
                     blend * position_vx_cmps;
    *vy_world_cmps = speed_weight * *vy_world_cmps +
                     blend * position_vy_cmps;
}

static void pf_limit_along_track_command(const pf_geometry_t *geometry,
                                         float max_along_speed_cmps,
                                         float *vx_world_cmps,
                                         float *vy_world_cmps,
                                         uint8 *speed_limit_active,
                                         uint8 *overspeed_guard_active,
                                         uint8 *reverse_requested)
{
    float candidate_along_cmps;
    float actual_along_cmps;
    float limited_along_cmps;
    float delta_along_cmps;

    if (geometry == NULL || vx_world_cmps == NULL || vy_world_cmps == NULL)
    {
        return;
    }

    max_along_speed_cmps = fmaxf(max_along_speed_cmps, 0.0f);
    candidate_along_cmps = *vx_world_cmps * geometry->dir_x +
                           *vy_world_cmps * geometry->dir_y;
    actual_along_cmps = g_pf.velocity_x_world_cmps * geometry->dir_x +
                        g_pf.velocity_y_world_cmps * geometry->dir_y;

    if (speed_limit_active != NULL &&
        candidate_along_cmps > max_along_speed_cmps)
    {
        *speed_limit_active = 1U;
    }
    if (overspeed_guard_active != NULL &&
        actual_along_cmps > max_along_speed_cmps)
    {
        *overspeed_guard_active = 1U;
    }
    if (reverse_requested != NULL && candidate_along_cmps < 0.0f)
    {
        *reverse_requested = 1U;
    }

    /* Reverse is forbidden only along the fixed segment tangent.  The normal
     * component keeps both signs so line/position correction can still move
     * toward the path from either side. */
    limited_along_cmps = pf_clamp(candidate_along_cmps,
                                  0.0f,
                                  max_along_speed_cmps);
    delta_along_cmps = limited_along_cmps - candidate_along_cmps;
    *vx_world_cmps += delta_along_cmps * geometry->dir_x;
    *vy_world_cmps += delta_along_cmps * geometry->dir_y;
}

static uint8 pf_apply_line_guidance(const pf_geometry_t *geometry,
                                    float *vx_world_cmps,
                                    float *vy_world_cmps)
{
    float start_x_m;
    float start_y_m;
    float segment_dx_m;
    float segment_dy_m;
    float segment_length_m;
    float tangent_x;
    float tangent_y;
    float normal_x;
    float normal_y;
    float relative_x_m;
    float relative_y_m;
    float normal_error_m;
    float effective_error_m;
    float trim_cmps;
    float start_boost = 1.0f;

    if (geometry == NULL || vx_world_cmps == NULL || vy_world_cmps == NULL ||
        g_pf.path == NULL || g_pf.target_idx == 0U ||
        g_pf.target_idx >= g_pf.steps ||
        (path_line_guide_kp <= 0.0f && path_line_guide_min_cmps <= 0.0f))
    {
        return 0U;
    }

    pf_point_to_world(g_pf.path[g_pf.target_idx - 1U],
                      g_pf.path_grid_m,
                      &start_x_m,
                      &start_y_m);
    segment_dx_m = geometry->target_x_m - start_x_m;
    segment_dy_m = geometry->target_y_m - start_y_m;
    segment_length_m = sqrtf(segment_dx_m * segment_dx_m +
                             segment_dy_m * segment_dy_m);
    if (segment_length_m <= 1.0e-6f)
    {
        return 0U;
    }

    tangent_x = segment_dx_m / segment_length_m;
    tangent_y = segment_dy_m / segment_length_m;
    normal_x = -tangent_y;
    normal_y = tangent_x;
    relative_x_m = g_pf.pose.x_m - start_x_m;
    relative_y_m = g_pf.pose.y_m - start_y_m;
    normal_error_m = relative_x_m * normal_x + relative_y_m * normal_y;

    if (fabsf(normal_error_m) <= PF_LINE_GUIDE_DEADBAND_M)
    {
        effective_error_m = 0.0f;
    }
    else if (normal_error_m > 0.0f)
    {
        effective_error_m = normal_error_m - PF_LINE_GUIDE_DEADBAND_M;
    }
    else
    {
        effective_error_m = normal_error_m + PF_LINE_GUIDE_DEADBAND_M;
    }

    trim_cmps = path_line_guide_kp * fabsf(effective_error_m * 100.0f);
    if (trim_cmps > 0.0f)
    {
        trim_cmps += fmaxf(path_line_guide_min_cmps, 0.0f);

        /* Boost only the first segment's launch.  The gain fades with
         * along-track travel, so later corners do not receive a fresh kick. */
        if (g_pf.target_idx == 1U && PF_LINE_GUIDE_START_BOOST_M > 0.0f)
        {
            float traveled_m = relative_x_m * tangent_x +
                               relative_y_m * tangent_y;
            float boost_blend = 1.0f -
                pf_clamp(traveled_m / PF_LINE_GUIDE_START_BOOST_M,
                         0.0f,
                         1.0f);

            start_boost += (fmaxf(PF_LINE_GUIDE_START_BOOST, 1.0f) - 1.0f) *
                           boost_blend;
        }
        trim_cmps *= start_boost;
    }
    trim_cmps = fminf(trim_cmps, PF_LINE_GUIDE_MAX_CMPS);
    if (effective_error_m > 0.0f)
    {
        trim_cmps = -trim_cmps;
    }

    /* Only add the cross-track correction.  The fixed segment tangent owns
     * longitudinal motion; line guidance must not add a second longitudinal
     * command or change the one-way arrival direction. */
    *vx_world_cmps += trim_cmps * normal_x;
    *vy_world_cmps += trim_cmps * normal_y;
    return 1U;
}

static void pf_limit_world_speed(float *vx_world_cmps,
                                 float *vy_world_cmps,
                                 float max_speed_cmps)
{
    float speed_norm_cmps;
    float scale;

    if (vx_world_cmps == NULL || vy_world_cmps == NULL)
    {
        return;
    }

    if (max_speed_cmps < 0.0f)
    {
        max_speed_cmps =
            g_default_speed_bands[PATH_FOLLOW_SCURVE_BAND_COUNT - 1U].vmax_mps * 100.0f;
    }

    speed_norm_cmps = sqrtf(*vx_world_cmps * *vx_world_cmps +
                            *vy_world_cmps * *vy_world_cmps);
    if (speed_norm_cmps <= max_speed_cmps || speed_norm_cmps <= 0.0f)
    {
        return;
    }

    if (max_speed_cmps <= 0.0f)
    {
        *vx_world_cmps = 0.0f;
        *vy_world_cmps = 0.0f;
        return;
    }

    scale = max_speed_cmps / speed_norm_cmps;
    *vx_world_cmps *= scale;
    *vy_world_cmps *= scale;
}

static float pf_yaw_command_radps(float yaw_deg,
                                  tagPID_T *controller,
                                  float feedforward_min_degps)
{
    float error_deg = pf_yaw_error_deg(yaw_deg, g_pf.target_yaw_deg);
    float output_degps;
    float feedforward_deadband_deg;

    if (controller == NULL || fabsf(error_deg) <= g_pf.yaw_tolerance_deg)
    {
        return 0.0f;
    }

    /* Keep sub-degree control resolution instead of truncating the generic
     * PID helper's historical int return value. */
    (void)PID_Location_Calculate(controller, 0.0f, error_deg);
    output_degps = controller->fCtrl_Out;
    feedforward_deadband_deg = fmaxf(path_yaw_feedforward_deadband_deg,
                                    g_pf.yaw_tolerance_deg);
    if (feedforward_min_degps > 0.0f &&
        fabsf(error_deg) > feedforward_deadband_deg &&
        fabsf(output_degps) < feedforward_min_degps)
    {
        output_degps = (error_deg > 0.0f) ?
                       feedforward_min_degps :
                       -feedforward_min_degps;
    }
    output_degps = pf_clamp(output_degps,
                            -g_pf.max_angular_speed_degps,
                            g_pf.max_angular_speed_degps);
    return output_degps * ((float)M_PI / 180.0f);
}

static void pf_update_odometry(float yaw_deg)
{
    float count_to_mps;
    float wheel_ul_mps;
    float wheel_ur_mps;
    float wheel_dl_mps;
    float wheel_dr_mps;
    float vx_body_mps;
    float vy_body_mps;
    float yaw_rad;
    float cos_yaw;
    float sin_yaw;

    g_pf.pose.yaw_deg = pf_wrap_deg(yaw_deg);
    if (g_pf.pulses_per_meter <= 0.0f)
    {
        g_pf.linear_speed_cmps = 0.0f;
        g_pf.velocity_x_world_cmps = 0.0f;
        g_pf.velocity_y_world_cmps = 0.0f;
        return;
    }

    count_to_mps = (float)PID_RATE / g_pf.pulses_per_meter;
    wheel_ul_mps = (float)up_L_all * count_to_mps;
    wheel_ur_mps = (float)up_R_all * count_to_mps;
    wheel_dl_mps = (float)down_L_all * count_to_mps;
    wheel_dr_mps = (float)down_R_all * count_to_mps;

    vx_body_mps = 0.25f * (wheel_ul_mps + wheel_ur_mps +
                           wheel_dl_mps + wheel_dr_mps);
    vy_body_mps = 0.25f * (-wheel_ul_mps + wheel_ur_mps +
                           wheel_dl_mps - wheel_dr_mps) *
                  LATERAL_CORRECTION_FACTOR;
    if (vy_body_mps > 0.0f)
    {
        vy_body_mps *= PF_LATERAL_LEFT_ODOMETRY_SCALE;
    }
    /* The speed-loop test observes raw wheel coupling.  Normal operation
     * still removes deliberate command feedforward from internal odometry. */
    if (!g_speed_test_enabled)
    {
        vx_body_mps -= pf_y_crosstalk_x_comp(vy_body_mps);
    }
    g_pf.linear_speed_cmps =
        sqrtf(vx_body_mps * vx_body_mps + vy_body_mps * vy_body_mps) * 100.0f;

    yaw_rad = yaw_deg * ((float)M_PI / 180.0f);
    cos_yaw = cosf(yaw_rad);
    sin_yaw = sinf(yaw_rad);

    g_pf.velocity_x_world_cmps =
        (vx_body_mps * cos_yaw - vy_body_mps * sin_yaw) * 100.0f;
    g_pf.velocity_y_world_cmps =
        (vx_body_mps * sin_yaw + vy_body_mps * cos_yaw) * 100.0f;
    g_pf.pose.x_m += g_pf.velocity_x_world_cmps * 0.01f * PF_DT_S;
    g_pf.pose.y_m += g_pf.velocity_y_world_cmps * 0.01f * PF_DT_S;
}

static void pf_reset_pause_runtime(void)
{
    g_pf.pause_cursor = 0U;
    g_pf.pause_cycles_left = 0U;
    g_pf.paused = 0U;
}

static void pf_clear_pause_config(void)
{
    size_t i;

    for (i = 0U; i < PATH_FOLLOW_MAX_PAUSE_POINTS; ++i)
    {
        g_pf.pause_indices[i] = PF_INVALID_INDEX;
    }
    g_pf.pause_count = 0U;
    g_pf.pause_cycles_cfg = 0U;
    pf_reset_pause_runtime();
}

static void pf_sync_pause_cursor(void)
{
    while (g_pf.pause_cursor < g_pf.pause_count &&
           g_pf.pause_indices[g_pf.pause_cursor] < g_pf.target_idx)
    {
        ++g_pf.pause_cursor;
    }
}

static uint8 pf_target_needs_pause(void)
{
    if (!g_pf.pause_events_enabled ||
        g_pf.pause_cycles_cfg == 0U ||
        g_pf.pause_cursor >= g_pf.pause_count)
    {
        return 0U;
    }

    pf_sync_pause_cursor();
    return (g_pf.pause_cursor < g_pf.pause_count &&
            g_pf.pause_indices[g_pf.pause_cursor] == g_pf.target_idx) ? 1U : 0U;
}

static void pf_finish_path(path_follow_output_t *out)
{
    g_pf.active = 0U;
    g_pf.paused = 0U;
    pf_reset_motion_state();
    PID_Clear(&pid_yaw);

    if (out != NULL)
    {
        out->active = 0U;
        out->reached = 1U;
        out->target_idx = g_pf.target_idx;
    }
}

static void pf_enter_pause(void)
{
    g_pf.paused = 1U;
    g_pf.pause_cycles_left = g_pf.pause_cycles_cfg;
    pf_reset_speed();
    car_direction = 0U;
}

static void pf_output_yaw_hold(float yaw_deg, path_follow_output_t *out)
{
    pf_clear_debug();
    car_direction = 0U;

    if (out != NULL)
    {
        out->active = 1U;
        out->reached = 0U;
        out->vx_cmd = 0.0f;
        out->vy_cmd = 0.0f;
        out->omega_cmd = pf_yaw_command_radps(yaw_deg,
                                              &pid_yaw,
                                              path_yaw_feedforward_min_degps);
        out->target_idx = g_pf.target_idx;
    }
}

static void pf_output_rotate(float yaw_deg, path_follow_output_t *out)
{
    pf_clear_debug();
    car_direction = 0U;

    if (out != NULL)
    {
        out->active = 1U;
        out->reached = 0U;
        out->vx_cmd = 0.0f;
        out->vy_cmd = 0.0f;
        pf_apply_rotate_position_hold(yaw_deg,
                                      &out->vx_cmd,
                                      &out->vy_cmd);
        out->omega_cmd = pf_yaw_command_radps(yaw_deg,
                                              &pid_yaw_rotate,
                                              path_rotate_yaw_feedforward_degps);
        out->target_idx = g_pf.target_idx;
    }
}

static uint8 pf_handle_pause(float yaw_deg, path_follow_output_t *out)
{
    if (!g_pf.paused)
    {
        return 0U;
    }

    pf_output_yaw_hold(yaw_deg, out);
    if (g_pf.pause_cycles_left > 0U)
    {
        --g_pf.pause_cycles_left;
    }
    if (g_pf.pause_cycles_left > 0U)
    {
        return 1U;
    }

    g_pf.paused = 0U;
    if (g_pf.pause_cursor < g_pf.pause_count &&
        g_pf.pause_indices[g_pf.pause_cursor] == g_pf.target_idx)
    {
        ++g_pf.pause_cursor;
    }

    if ((g_pf.target_idx + 1U) < g_pf.steps)
    {
        ++g_pf.target_idx;
        /* A configured pause restarts the next segment from standstill. */
        pf_reset_speed();
        return 1U;
    }

    pf_finish_path(out);
    return 1U;
}

static uint8 pf_advance_reached_target(path_follow_output_t *out)
{
    pf_reset_position_transition();

    if (pf_target_needs_pause())
    {
        pf_enter_pause();
        return 0U;
    }

    if ((g_pf.target_idx + 1U) < g_pf.steps)
    {
        ++g_pf.target_idx;
        /* Preserve last_ref_speed_cmps so the next S-curve keeps v0 continuity. */
        return 1U;
    }

    pf_finish_path(out);
    return 0U;
}

static uint8 pf_prepare_geometry(pf_geometry_t *geometry, path_follow_output_t *out)
{
    if (geometry == NULL)
    {
        return 0U;
    }

    while (g_pf.active && g_pf.path != NULL && g_pf.target_idx < g_pf.steps)
    {
        Position target = g_pf.path[g_pf.target_idx];
        float segment_start_x_m = g_pf.pose.x_m;
        float segment_start_y_m = g_pf.pose.y_m;
        float segment_dx_m;
        float segment_dy_m;
        float segment_dir_x = 0.0f;
        float segment_dir_y = 0.0f;
        float actual_along_speed_cmps = 0.0f;
        uint8 position_ready;

        pf_sync_pause_cursor();
        pf_point_to_world(target,
                          g_pf.path_grid_m,
                          &geometry->target_x_m,
                          &geometry->target_y_m);

        geometry->dx_m = geometry->target_x_m - g_pf.pose.x_m;
        geometry->dy_m = geometry->target_y_m - g_pf.pose.y_m;
        geometry->distance_m = sqrtf(geometry->dx_m * geometry->dx_m +
                                     geometry->dy_m * geometry->dy_m);

        if (g_pf.target_idx > 0U)
        {
            pf_point_to_world(g_pf.path[g_pf.target_idx - 1U],
                              g_pf.path_grid_m,
                              &segment_start_x_m,
                              &segment_start_y_m);
        }
        segment_dx_m = geometry->target_x_m - segment_start_x_m;
        segment_dy_m = geometry->target_y_m - segment_start_y_m;
        geometry->planned_distance_m = sqrtf(segment_dx_m * segment_dx_m +
                                             segment_dy_m * segment_dy_m);
        geometry->along_track_remaining_m = geometry->distance_m;

        /* Signed remaining distance and the fixed segment direction are used
         * by the one-way real-time limiter. */
        if (geometry->planned_distance_m > PF_MIN_VALID_GRID_M)
        {
            segment_dir_x = segment_dx_m / geometry->planned_distance_m;
            segment_dir_y = segment_dy_m / geometry->planned_distance_m;
            geometry->along_track_remaining_m =
                geometry->dx_m * segment_dir_x +
                geometry->dy_m * segment_dir_y;
            geometry->target_plane_crossed =
                (geometry->along_track_remaining_m <= 0.0f) ? 1U : 0U;
            actual_along_speed_cmps =
                g_pf.velocity_x_world_cmps * segment_dir_x +
                g_pf.velocity_y_world_cmps * segment_dir_y;
        }

        geometry->within_tolerance =
            (geometry->distance_m <= g_pf.position_tolerance_m) ? 1U : 0U;
        position_ready =
            ((geometry->within_tolerance &&
              g_pf.linear_speed_cmps <= PF_ARRIVAL_MAX_SPEED_CMPS) ||
             (geometry->target_plane_crossed &&
              fabsf(actual_along_speed_cmps) <=
                  PF_CROSSED_TARGET_MAX_SPEED_CMPS)) ? 1U : 0U;

        if (g_speed_test_enabled)
        {
            uint8 profile_done =
                (g_pf.profile_active && g_pf.active_profile.T > 0.0f &&
                 g_pf.profile_target_idx == g_pf.target_idx &&
                 g_pf.profile_time_s >= g_pf.active_profile.T) ? 1U : 0U;

            if (profile_done)
            {
                uint8 wheels_still =
                    (fabsf((float)up_L_all) <= PF_SPEED_TEST_SETTLE_COUNTS &&
                     fabsf((float)up_R_all) <= PF_SPEED_TEST_SETTLE_COUNTS &&
                     fabsf((float)down_L_all) <= PF_SPEED_TEST_SETTLE_COUNTS &&
                     fabsf((float)down_R_all) <= PF_SPEED_TEST_SETTLE_COUNTS &&
                     fabsf((float)encoder_data_quaddec1) <= PF_SPEED_TEST_SETTLE_COUNTS &&
                     fabsf((float)-encoder_data_quaddec2) <= PF_SPEED_TEST_SETTLE_COUNTS &&
                     fabsf((float)encoder_data_quaddec3) <= PF_SPEED_TEST_SETTLE_COUNTS &&
                     fabsf((float)-encoder_data_quaddec4) <= PF_SPEED_TEST_SETTLE_COUNTS) ? 1U : 0U;

                g_speed_test_settling = 1U;
                g_speed_test_settle_ticks++;
                g_speed_test_stable_ticks = wheels_still ?
                    (uint16)(g_speed_test_stable_ticks + 1U) : 0U;

                if (g_speed_test_stable_ticks >= PF_SPEED_TEST_SETTLE_TICKS ||
                    g_speed_test_settle_ticks >= PF_SPEED_TEST_SETTLE_TIMEOUT)
                {
                    if (g_speed_test_settle_ticks >= PF_SPEED_TEST_SETTLE_TIMEOUT &&
                        g_speed_test_stable_ticks < PF_SPEED_TEST_SETTLE_TICKS)
                    {
                        g_speed_test_settle_timeout = 1U;
                        pf_finish_path(out);
                        return 0U;
                    }
                    g_speed_test_settling = 0U;
                    g_speed_test_stable_ticks = 0U;
                    g_speed_test_settle_ticks = 0U;
                    if (!pf_advance_reached_target(out))
                    {
                        return 0U;
                    }
                    if (!g_pf.active || g_pf.target_idx >= g_pf.steps)
                    {
                        return 0U;
                    }
                    continue;
                }
            }
            else
            {
                g_speed_test_settling = 0U;
                g_speed_test_stable_ticks = 0U;
                g_speed_test_settle_ticks = 0U;
            }
        }

        if (!g_speed_test_enabled && position_ready)
        {
            /* Accept either inside 1.5 cm below 5 cm/s, or after crossing the
             * fixed target plane once along-track speed is below 3 cm/s. */
            if (!pf_advance_reached_target(out))
            {
                return 0U;
            }
            if (!g_pf.active || g_pf.target_idx >= g_pf.steps)
            {
                return 0U;
            }
            continue;
        }

        if (geometry->planned_distance_m > PF_MIN_VALID_GRID_M)
        {
            /* During line-guidance mode the base velocity follows only the
             * fixed segment tangent.  Cross-track motion is exclusively
             * supplied by pf_apply_line_guidance(); near-target position
             * motion is exclusively supplied by the position loop. */
            geometry->dir_x = segment_dir_x;
            geometry->dir_y = segment_dir_y;
        }
        else if (geometry->distance_m > PF_MIN_VALID_GRID_M)
        {
            geometry->dir_x = geometry->dx_m / geometry->distance_m;
            geometry->dir_y = geometry->dy_m / geometry->distance_m;
        }
        else
        {
            geometry->dir_x = 0.0f;
            geometry->dir_y = 0.0f;
        }
        geometry->segment_axis =
            (g_pf.segment_axis_override != PATH_FOLLOW_AXIS_AUTO) ?
            g_pf.segment_axis_override :
            pf_segment_axis(geometry->dir_x, geometry->dir_y);
        car_direction = geometry->segment_axis;
        return 1U;
    }

    return 0U;
}

static void pf_world_to_body(float vx_world_cmps,
                             float vy_world_cmps,
                             float yaw_deg,
                             float *vx_body_cmps,
                             float *vy_body_cmps)
{
    float yaw_rad = yaw_deg * ((float)M_PI / 180.0f);
    float cos_yaw = cosf(yaw_rad);
    float sin_yaw = sinf(yaw_rad);

    if (vx_body_cmps != NULL)
    {
        *vx_body_cmps = vx_world_cmps * cos_yaw + vy_world_cmps * sin_yaw;
    }
    if (vy_body_cmps != NULL)
    {
        *vy_body_cmps = -vx_world_cmps * sin_yaw + vy_world_cmps * cos_yaw;
    }
}

static void pf_apply_path(const Position *path,
                          size_t steps,
                          float grid_m,
                          uint8 pause_events_enabled,
                          uint8 segment_axis_override)
{
    g_pf.path = path;
    g_pf.steps = steps;
    g_pf.target_idx = 0U;
    g_pf.path_grid_m = (grid_m > PF_MIN_VALID_GRID_M) ? grid_m : g_pf.default_grid_m;
    g_pf.path_origin_x_m = 0.0f;
    g_pf.path_origin_y_m = 0.0f;
    g_pf.pause_events_enabled = pause_events_enabled ? 1U : 0U;
    g_pf.rotate_only_active = 0U;
    g_pf.segment_axis_override =
        (segment_axis_override == PATH_FOLLOW_AXIS_X ||
         segment_axis_override == PATH_FOLLOW_AXIS_Y) ?
        segment_axis_override : PATH_FOLLOW_AXIS_AUTO;
    g_pf.active = (path != NULL && steps > 0U) ? 1U : 0U;

    pf_reset_pause_runtime();
    pf_reset_motion_state();
    g_speed_test_settle_timeout = 0U;
    g_speed_test_profile_fault = 0U;
    g_speed_test_frame_yaw_deg = g_pf.pose.yaw_deg;
    PID_Clear(&pid_yaw);
    PID_Clear(&pid_yaw_rotate);

    /* A normal planned path usually includes the current cell as point zero. */
    if (g_pf.active && steps > 1U)
    {
        g_pf.target_idx = 1U;
    }
}

static void pf_start_axis_move(float target_x_m,
                               float target_y_m,
                               Position *buffer,
                               size_t capacity)
{
    const float epsilon_m = 0.001f;
    const float local_center = 127.0f;
    const float local_half_span = 120.0f;
    float dx_m = target_x_m - g_pf.pose.x_m;
    float dy_m = target_y_m - g_pf.pose.y_m;
    float max_delta_m = fmaxf(fabsf(dx_m), fabsf(dy_m));
    float local_grid_m = fmaxf(PF_TEMP_PATH_GRID_M,
                               max_delta_m / local_half_span);
    int target_row;
    int target_col;

    if (buffer == NULL || capacity < 2U)
    {
        return;
    }

    if (fabsf(dx_m) <= epsilon_m && fabsf(dy_m) <= epsilon_m)
    {
        pf_apply_path(NULL, 0U, PF_TEMP_PATH_GRID_M, 0U, PATH_FOLLOW_AXIS_AUTO);
        return;
    }

    buffer[0].row = (uint8)local_center;
    buffer[0].col = (uint8)local_center;
    target_row = (int)local_center + (int)lroundf(dx_m / local_grid_m);
    target_col = (int)local_center + (int)lroundf(dy_m / local_grid_m);

    /* local_half_span leaves guard cells on both sides, but keep the cast
     * protected if the constants are changed later. */
    target_row = (int)pf_clamp((float)target_row, 0.0f, 255.0f);
    target_col = (int)pf_clamp((float)target_col, 0.0f, 255.0f);

    /* A relative/point move is one straight segment in world coordinates.
     * Splitting a slightly diagonal command into X then Y created a false
     * arrival between the two axes: the car stopped after the long leg and
     * restarted for a tiny yaw-induced tail segment. */
    buffer[1].row = (uint8)target_row;
    buffer[1].col = (uint8)target_col;

    pf_apply_path(buffer, 2U, local_grid_m, 0U, PATH_FOLLOW_AXIS_AUTO);
    g_pf.path_origin_x_m = g_pf.pose.x_m - local_center * local_grid_m;
    g_pf.path_origin_y_m = g_pf.pose.y_m - local_center * local_grid_m;
}

void path_follow_init(float grid_size_m, float pulses_per_meter)
{
    memset(&g_pf, 0, sizeof(g_pf));

    g_pf.default_grid_m = (grid_size_m > PF_MIN_VALID_GRID_M) ? grid_size_m : 1.0f;
    g_pf.path_grid_m = g_pf.default_grid_m;
    g_pf.pulses_per_meter = pulses_per_meter;
    g_pf.position_tolerance_m = PF_POSITION_TOLERANCE_M;
    g_pf.yaw_tolerance_deg = PF_YAW_TOLERANCE_DEG;
    g_pf.max_linear_speed_mps = PF_MAX_LINEAR_SPEED_MPS;
    g_pf.max_angular_speed_degps = PF_MAX_ANGULAR_SPEED_DEGPS;
    g_pf.target_yaw_deg = 0.0f;

    g_stationary_yaw_hold_enabled = 0U;
    g_bluetooth_report_pending = 0U;
    g_speed_test_enabled = 0U;
    g_speed_test_yaw_enabled = 0U;
    g_speed_test_frame_yaw_deg = 0.0f;
    g_speed_test_settle_timeout = 0U;
    g_speed_test_profile_fault = 0U;
    path_follow_reset_scurve_band_defaults();
    pf_clear_pause_config();
    pf_reset_motion_state();

    /* Heading loop. */
    pf_init_pid_object(&pid_yaw,
                       PF_YAW_KP,
                       PF_YAW_KI,
                       PF_YAW_KD,
                       PF_MAX_ANGULAR_SPEED_DEGPS,
                       PF_YAW_FILTER_ALPHA);
    pf_init_pid_object(&pid_yaw_rotate,
                       PF_ROTATE_YAW_KP,
                       PF_ROTATE_YAW_KI,
                       PF_ROTATE_YAW_KD,
                       PF_MAX_ANGULAR_SPEED_DEGPS,
                       PF_YAW_FILTER_ALPHA);

    /* Body X forward/backward and body Y lateral motion keep independent
     * gains and controller histories. */
    pf_init_pid_object(&pid_stay,
                       PF_POSITION_X_KP,
                       PF_POSITION_X_KI,
                       PF_POSITION_X_KD,
                       PF_POSITION_X_MAX_OUT_CMPS,
                       PF_POSITION_FILTER_ALPHA);
    pid_stay.fMax_Iout = PF_POSITION_X_MAX_IOUT_CMPS;
    pf_init_pid_object(&pid_stay_y,
                       PF_POSITION_Y_KP,
                       PF_POSITION_Y_KI,
                       PF_POSITION_Y_KD,
                       PF_POSITION_Y_MAX_OUT_CMPS,
                       PF_POSITION_FILTER_ALPHA);
    pid_stay_y.fMax_Iout = PF_POSITION_Y_MAX_IOUT_CMPS;
    pf_init_pid_object(&pid_stay_backward,
                       PF_POSITION_BACKWARD_X_KP,
                       PF_POSITION_BACKWARD_X_KI,
                       PF_POSITION_BACKWARD_X_KD,
                       PF_POSITION_X_MAX_OUT_CMPS,
                       PF_POSITION_FILTER_ALPHA);
    pid_stay_backward.fMax_Iout = PF_POSITION_X_MAX_IOUT_CMPS;

    /* Historical world PID objects remain link-compatible but inactive. */
    pf_init_pid_object(&pid_world_x, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    pf_init_pid_object(&pid_world_y, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    pf_init_pid_object(&pid_accel_yaw, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
}

void path_follow_set_position_pid_gains(float kp, float ki, float kd)
{
    path_follow_set_position_pid_gains_x(kp, ki, kd);
    path_follow_set_position_pid_gains_y(kp, ki, kd);
    path_follow_set_position_pid_gains_backward(kp, ki, kd);
}

void path_follow_set_position_pid_gains_x(float kp, float ki, float kd)
{
    pid_stay.fKp = fmaxf(kp, 0.0f);
    pid_stay.fKi = fmaxf(ki, 0.0f);
    pid_stay.fKd = fmaxf(kd, 0.0f);
    PID_Clear(&pid_stay);
}

void path_follow_set_position_pid_gains_y(float kp, float ki, float kd)
{
    pid_stay_y.fKp = fmaxf(kp, 0.0f);
    pid_stay_y.fKi = fmaxf(ki, 0.0f);
    pid_stay_y.fKd = fmaxf(kd, 0.0f);
    PID_Clear(&pid_stay_y);
}

void path_follow_set_position_pid_gains_forward(float kp, float ki, float kd)
{
    path_follow_set_position_pid_gains_x(kp, ki, kd);
}

void path_follow_set_position_pid_gains_backward(float kp, float ki, float kd)
{
    pid_stay_backward.fKp = fmaxf(kp, 0.0f);
    pid_stay_backward.fKi = fmaxf(ki, 0.0f);
    pid_stay_backward.fKd = fmaxf(kd, 0.0f);
    PID_Clear(&pid_stay_backward);
}

void path_follow_set_rotate_yaw_pid_gains(float kp, float ki, float kd)
{
    pid_yaw_rotate.fKp = fmaxf(kp, 0.0f);
    pid_yaw_rotate.fKi = fmaxf(ki, 0.0f);
    pid_yaw_rotate.fKd = fmaxf(kd, 0.0f);
    PID_Clear(&pid_yaw_rotate);
}

void path_follow_get_rotate_yaw_pid_gains(float *kp, float *ki, float *kd)
{
    if (kp != NULL)
    {
        *kp = pid_yaw_rotate.fKp;
    }
    if (ki != NULL)
    {
        *ki = pid_yaw_rotate.fKi;
    }
    if (kd != NULL)
    {
        *kd = pid_yaw_rotate.fKd;
    }
}

static void pf_apply_rotate_position_hold(float yaw_deg,
                                          float *vx_body_cmps,
                                          float *vy_body_cmps)
{
    float error_x_cm;
    float error_y_cm;
    float error_norm_cm;
    float vx_world_cmps;
    float vy_world_cmps;
    float speed_norm_cmps;
    float scale;
    float vx_correction_body_cmps;
    float vy_correction_body_cmps;

    if (vx_body_cmps == NULL || vy_body_cmps == NULL)
    {
        return;
    }

    error_x_cm = (g_pf.rotate_hold_x_m - g_pf.pose.x_m) * 100.0f;
    error_y_cm = (g_pf.rotate_hold_y_m - g_pf.pose.y_m) * 100.0f;
    if (fabsf(error_x_cm) <= PF_ROTATE_POSITION_HOLD_DEADBAND_CM)
    {
        error_x_cm = 0.0f;
    }
    if (fabsf(error_y_cm) <= PF_ROTATE_POSITION_HOLD_DEADBAND_CM)
    {
        error_y_cm = 0.0f;
    }

    vx_world_cmps = error_x_cm * PF_ROTATE_POSITION_HOLD_KP_CMPS_PER_CM;
    vy_world_cmps = error_y_cm * PF_ROTATE_POSITION_HOLD_KP_CMPS_PER_CM;
    speed_norm_cmps = sqrtf(vx_world_cmps * vx_world_cmps +
                            vy_world_cmps * vy_world_cmps);
    if (speed_norm_cmps > PF_ROTATE_POSITION_HOLD_MAX_CMPS &&
        speed_norm_cmps > 0.0f)
    {
        scale = PF_ROTATE_POSITION_HOLD_MAX_CMPS / speed_norm_cmps;
        vx_world_cmps *= scale;
        vy_world_cmps *= scale;
    }

    pf_world_to_body(vx_world_cmps,
                     vy_world_cmps,
                     yaw_deg,
                     &vx_correction_body_cmps,
                     &vy_correction_body_cmps);
    *vx_body_cmps += vx_correction_body_cmps;
    *vy_body_cmps += vy_correction_body_cmps;

    error_norm_cm = sqrtf(error_x_cm * error_x_cm + error_y_cm * error_y_cm);
    g_pf.debug.distance_m = error_norm_cm * 0.01f;
    g_pf.debug.target_x_m = g_pf.rotate_hold_x_m;
    g_pf.debug.target_y_m = g_pf.rotate_hold_y_m;
    if (error_norm_cm > 0.0f)
    {
        g_pf.debug.dir_x = error_x_cm / error_norm_cm;
        g_pf.debug.dir_y = error_y_cm / error_norm_cm;
    }
}

void path_follow_set_rotate_yaw_feedforward(float feedforward_degps)
{
    path_rotate_yaw_feedforward_degps = fmaxf(feedforward_degps, 0.0f);
}

float path_follow_get_rotate_yaw_feedforward(void)
{
    return path_rotate_yaw_feedforward_degps;
}

void path_follow_set_position_speed_factor(float factor)
{
    path_follow_set_position_speed_factor_x(factor);
    path_follow_set_position_speed_factor_y(factor);
}

void path_follow_set_position_speed_factor_x(float factor)
{
    path_position_speed_limit_factor_x =
        pf_clamp(factor,
                 PF_POSITION_SPEED_FACTOR_MIN,
                 PF_POSITION_SPEED_FACTOR_MAX);
}

void path_follow_set_position_speed_factor_y(float factor)
{
    path_position_speed_limit_factor_y =
        pf_clamp(factor,
                 PF_POSITION_SPEED_FACTOR_MIN,
                 PF_POSITION_SPEED_FACTOR_MAX);
}

float path_follow_get_position_speed_factor(void)
{
    return fminf(path_follow_get_position_speed_factor_x(),
                 path_follow_get_position_speed_factor_y());
}

float path_follow_get_position_speed_factor_x(void)
{
    return pf_position_speed_factor_for_axis(1U);
}

float path_follow_get_position_speed_factor_y(void)
{
    return pf_position_speed_factor_for_axis(2U);
}

void path_follow_set_speed_test_mode(uint8 enabled, uint8 yaw_enabled)
{
    g_speed_test_enabled = enabled ? 1U : 0U;
    g_speed_test_yaw_enabled = yaw_enabled ? 1U : 0U;
    g_speed_test_frame_yaw_deg = g_pf.pose.yaw_deg;
    g_speed_test_settling = 0U;
    g_speed_test_stable_ticks = 0U;
    g_speed_test_settle_ticks = 0U;
    if (g_speed_test_enabled)
    {
        g_speed_test_settle_timeout = 0U;
        g_speed_test_profile_fault = 0U;
    }

    if (!g_speed_test_yaw_enabled)
    {
        PID_Clear(&pid_yaw);
    }
}

void path_follow_reset_pose(float x_m, float y_m, float yaw_deg)
{
    g_pf.pose.x_m = x_m;
    g_pf.pose.y_m = y_m;
    g_pf.pose.yaw_deg = pf_wrap_deg(yaw_deg);
    pf_reset_motion_state();
    PID_Clear(&pid_yaw);
    PID_Clear(&pid_yaw_rotate);
}

void path_follow_set_external_position(float x_m, float y_m, uint8 valid)
{
    /* Compatibility stub. Automatic external-position correction is removed. */
    (void)x_m;
    (void)y_m;
    (void)valid;
}

void path_follow_set_path(const Position *path, size_t steps)
{
    path_follow_set_path_pause_enabled(path, steps, 1U);
}

void path_follow_set_path_pause_enabled(const Position *path,
                                        size_t steps,
                                        uint8 pause_events_enabled)
{
    pf_apply_path(path, steps, g_pf.default_grid_m, pause_events_enabled,
                  PATH_FOLLOW_AXIS_AUTO);
}

void path_follow_set_path_with_grid(const Position *path,
                                    size_t steps,
                                    float grid_m,
                                    uint8 pause_events_enabled)
{
    pf_apply_path(path, steps, grid_m, pause_events_enabled,
                  PATH_FOLLOW_AXIS_AUTO);
}

void path_follow_set_path_with_grid_axis(const Position *path,
                                         size_t steps,
                                         float grid_m,
                                         uint8 pause_events_enabled,
                                         uint8 segment_axis)
{
    pf_apply_path(path, steps, grid_m, pause_events_enabled, segment_axis);
}

void path_follow_set_pause_indices(const size_t *pause_indices,
                                   size_t pause_count,
                                   uint32 pause_ms)
{
    size_t i;

    pf_clear_pause_config();
    if (pause_indices == NULL || pause_count == 0U || pause_ms == 0U)
    {
        return;
    }

    g_pf.pause_count = (pause_count < PATH_FOLLOW_MAX_PAUSE_POINTS) ?
                       pause_count : PATH_FOLLOW_MAX_PAUSE_POINTS;
    for (i = 0U; i < g_pf.pause_count; ++i)
    {
        g_pf.pause_indices[i] = pause_indices[i];
    }
    g_pf.pause_cycles_cfg = pf_ms_to_cycles(pause_ms);
}

void path_follow_set_target(int target_row, int target_col)
{
    g_single_target_path[0].row = (int)lroundf(g_pf.pose.x_m / g_pf.default_grid_m);
    g_single_target_path[0].col = (int)lroundf(g_pf.pose.y_m / g_pf.default_grid_m);
    g_single_target_path[1].row = target_row;
    g_single_target_path[1].col = target_col;
    pf_apply_path(g_single_target_path, 2U, g_pf.default_grid_m, 0U,
                  PATH_FOLLOW_AXIS_AUTO);
}

void path_follow_set_target_yaw(float target_yaw_deg)
{
    /* Global invariant: a commanded heading is always a map-cardinal angle.
     * Measured yaw is never allowed to become the next target. */
    g_pf.target_yaw_deg = pf_cardinal_target_deg(target_yaw_deg);
}

void path_follow_set_stationary_yaw_hold_enabled(uint8 enabled)
{
    g_stationary_yaw_hold_enabled = enabled ? 1U : 0U;
    if (!g_stationary_yaw_hold_enabled)
    {
        PID_Clear(&pid_yaw);
    }
}

uint8 path_follow_get_stationary_yaw_hold_enabled(void)
{
    return g_stationary_yaw_hold_enabled;
}

void path_follow_start_rotate_to_yaw(float target_yaw_deg)
{
    pf_apply_path(NULL, 0U, g_pf.default_grid_m, 0U, PATH_FOLLOW_AXIS_AUTO);
    g_pf.rotate_hold_x_m = g_pf.pose.x_m;
    g_pf.rotate_hold_y_m = g_pf.pose.y_m;
    path_follow_set_target_yaw(target_yaw_deg);
    PID_Clear(&pid_yaw_rotate);
    g_pf.rotate_only_active = 1U;
}

void path_follow_start_offset_move(float delta_x_m, float delta_y_m)
{
    pf_start_axis_move(g_pf.pose.x_m + delta_x_m,
                       g_pf.pose.y_m + delta_y_m,
                       g_offset_path,
                       sizeof(g_offset_path) / sizeof(g_offset_path[0]));
}

void path_follow_start_pose_correction(float target_x_m, float target_y_m)
{
    /* Compatibility name: this is only an explicit basic point move. */
    pf_start_axis_move(target_x_m,
                       target_y_m,
                       g_pose_move_path,
                       sizeof(g_pose_move_path) / sizeof(g_pose_move_path[0]));
}

void path_follow_update(float yaw_deg, path_follow_output_t *out)
{
    pf_geometry_t geometry = {0};
    pf_speed_plan_t speed_plan = {0};
    float vx_world_cmps;
    float vy_world_cmps;
    float yaw_error_deg;
    float command_frame_yaw_deg;
    float position_blend;
    float position_speed_limit_cmps;
    float actual_along_speed_cmps;
    uint8 speed_limit_active = 0U;
    uint8 overspeed_guard_active = 0U;
    uint8 position_reverse_requested = 0U;
    uint8 use_position_loop;
    uint8 use_line_guidance;

    pf_clear_output(out);
    pf_update_odometry(yaw_deg);

    if (g_pf.rotate_only_active)
    {
        yaw_error_deg = pf_yaw_error_deg(yaw_deg, g_pf.target_yaw_deg);
        if (fabsf(yaw_error_deg) <= g_pf.yaw_tolerance_deg)
        {
            g_pf.rotate_only_active = 0U;
            PID_Clear(&pid_yaw_rotate);
            if (out != NULL)
            {
                out->reached = 1U;
            }
            return;
        }

        pf_output_rotate(yaw_deg, out);
        return;
    }

    if (pf_handle_pause(yaw_deg, out))
    {
        return;
    }

    if (!g_pf.active || g_pf.path == NULL || g_pf.steps == 0U)
    {
        if (g_stationary_yaw_hold_enabled)
        {
            pf_output_yaw_hold(yaw_deg, out);
        }
        else
        {
            pf_clear_debug();
            car_direction = 0U;
        }
        return;
    }

    if (!pf_prepare_geometry(&geometry, out))
    {
        if (g_pf.paused)
        {
            (void)pf_handle_pause(yaw_deg, out);
        }
        return;
    }

    position_blend = g_speed_test_enabled ? 0.0f :
        pf_position_loop_blend(&geometry);
    position_speed_limit_cmps = g_speed_test_enabled ? 0.0f :
        pf_position_speed_limit_cmps(
            fmaxf(geometry.along_track_remaining_m, 0.0f),
            geometry.segment_axis);
    actual_along_speed_cmps =
        g_pf.velocity_x_world_cmps * geometry.dir_x +
        g_pf.velocity_y_world_cmps * geometry.dir_y;

    pf_plan_scurve_speed(&geometry, &speed_plan);
    if (g_speed_test_enabled && g_speed_test_profile_fault)
    {
        return;
    }

    vx_world_cmps = speed_plan.ref_speed_cmps * geometry.dir_x;
    vy_world_cmps = speed_plan.ref_speed_cmps * geometry.dir_y;
    use_position_loop = (!g_speed_test_enabled && position_blend > 0.0f) ? 1U : 0U;
    use_line_guidance = (!g_speed_test_enabled && !use_position_loop) ? 1U : 0U;
    if (use_position_loop)
    {
        pf_apply_position_loop(&geometry,
                               &vx_world_cmps,
                               &vy_world_cmps,
                               position_blend);
    }
    else if (use_line_guidance)
    {
        (void)pf_apply_line_guidance(&geometry,
                                     &vx_world_cmps,
                                      &vy_world_cmps);
    }
    if (!g_speed_test_enabled)
    {
        /* One continuous limiter owns normal endpoint deceleration.  It caps
         * the already blended command every tick and records measured
         * overspeed without entering a latched braking state. */
        pf_limit_along_track_command(&geometry,
                                     position_speed_limit_cmps,
                                     &vx_world_cmps,
                                     &vy_world_cmps,
                                     &speed_limit_active,
                                     &overspeed_guard_active,
                                     &position_reverse_requested);
    }
    pf_limit_world_speed(&vx_world_cmps,
                          &vy_world_cmps,
                          speed_plan.safety_cap_cmps);

    g_pf.debug.distance_m = geometry.distance_m;
    g_pf.debug.dir_x = geometry.dir_x;
    g_pf.debug.dir_y = geometry.dir_y;
    g_pf.debug.speed_ref_cmps = sqrtf(vx_world_cmps * vx_world_cmps +
                                      vy_world_cmps * vy_world_cmps);
    g_pf.debug.speed_cap_cmps = g_speed_test_enabled ?
                                speed_plan.safety_cap_cmps :
                                fminf(speed_plan.safety_cap_cmps,
                                      position_speed_limit_cmps);
    g_pf.debug.target_x_m = geometry.target_x_m;
    g_pf.debug.target_y_m = geometry.target_y_m;
    g_pf.debug.position_speed_limit_cmps = position_speed_limit_cmps;
    g_pf.debug.actual_along_speed_cmps = actual_along_speed_cmps;
    g_pf.debug.position_blend = position_blend;
    g_pf.debug.speed_limit_active = speed_limit_active;
    g_pf.debug.overspeed_guard_active = overspeed_guard_active;
    g_pf.debug.position_reverse_requested = position_reverse_requested;
    g_pf.debug.target_plane_crossed = geometry.target_plane_crossed;
    g_pf.debug.position_loop_active = use_position_loop;
    g_pf.debug.line_guidance_active = use_line_guidance;
    g_pf.debug.segment_axis = geometry.segment_axis;

    if (out != NULL)
    {
        command_frame_yaw_deg = g_speed_test_enabled ?
                                g_speed_test_frame_yaw_deg : yaw_deg;
        pf_world_to_body(vx_world_cmps,
                         vy_world_cmps,
                         command_frame_yaw_deg,
                         &out->vx_cmd,
                         &out->vy_cmd);
        if (!g_speed_test_enabled)
        {
            out->vx_cmd += pf_y_crosstalk_x_comp(out->vy_cmd);
        }
        if (g_speed_test_enabled &&
            (!g_speed_test_yaw_enabled || g_speed_test_settling))
        {
            out->omega_cmd = 0.0f;
        }
        else
        {
            out->omega_cmd = pf_yaw_command_radps(yaw_deg,
                                                  &pid_yaw,
                                                  path_yaw_feedforward_min_degps);
        }
        out->active = 1U;
        out->reached = 0U;
        out->target_idx = g_pf.target_idx;
    }
}

void path_follow_update_yaw_hold(float yaw_deg, path_follow_output_t *out)
{
    pf_clear_output(out);
    pf_update_odometry(yaw_deg);
    pf_output_yaw_hold(yaw_deg, out);
}

/* Historical test entry kept for source-level compatibility. */
void path_follow_update_test(float yaw_deg, path_follow_output_t *out)
{
    path_follow_update(yaw_deg, out);
}

void distance_speed_strategy(void)
{
    path_follow_output_t output = {0};

    path_follow_update(eulerAngle.yaw, &output);
    if (output.active)
    {
        speed_three_array[0] = output.vx_cmd;
        speed_three_array[1] = output.vy_cmd;
        speed_three_array[2] = output.omega_cmd;
    }
    else
    {
        speed_three_array[0] = 0.0f;
        speed_three_array[1] = 0.0f;
        speed_three_array[2] = 0.0f;
    }

    Kinematics_Inverse(speed_three_array, speed_encoder);
}

void path_follow_get_status(path_follow_status_t *status)
{
    if (status == NULL)
    {
        return;
    }

    memset(status, 0, sizeof(*status));
    status->x_m = g_pf.pose.x_m;
    status->y_m = g_pf.pose.y_m;
    status->yaw_deg = g_pf.pose.yaw_deg;
    status->target_x_m = g_pf.debug.target_x_m;
    status->target_y_m = g_pf.debug.target_y_m;
    status->distance_m = g_pf.debug.distance_m;
    status->dir_x = g_pf.debug.dir_x;
    status->dir_y = g_pf.debug.dir_y;
    status->speed_ref_cmps = g_pf.debug.speed_ref_cmps;
    status->actual_speed_cmps = g_pf.linear_speed_cmps;
    status->speed_cap_cmps = g_pf.debug.speed_cap_cmps;
    status->position_speed_limit_cmps =
        g_pf.debug.position_speed_limit_cmps;
    status->actual_along_speed_cmps = g_pf.debug.actual_along_speed_cmps;
    status->position_blend = g_pf.debug.position_blend;
    status->position_speed_factor =
        pf_position_speed_factor_for_axis(g_pf.debug.segment_axis);
    status->position_speed_factor_x = path_follow_get_position_speed_factor_x();
    status->position_speed_factor_y = path_follow_get_position_speed_factor_y();
    status->target_yaw_deg = g_pf.target_yaw_deg;
    status->speed_test_profile_time_s = g_pf.profile_time_s;
    status->speed_test_profile_total_s = g_pf.active_profile.T;
    status->active = (g_pf.active || g_pf.rotate_only_active) ? 1U : 0U;
    status->reached = status->active ? 0U : 1U;
    status->paused = g_pf.paused;
    status->yaw_only_active = g_pf.rotate_only_active;
    status->speed_limit_active = g_pf.debug.speed_limit_active;
    status->overspeed_guard_active = g_pf.debug.overspeed_guard_active;
    status->approach_braking_active =
        g_pf.debug.overspeed_guard_active;
    status->position_reverse_requested =
        g_pf.debug.position_reverse_requested;
    status->target_plane_crossed = g_pf.debug.target_plane_crossed;
    status->position_loop_active = g_pf.debug.position_loop_active;
    status->line_guidance_active = g_pf.debug.line_guidance_active;
    status->speed_test_enabled = g_speed_test_enabled;
    status->speed_test_yaw_enabled = g_speed_test_yaw_enabled;
    status->speed_test_settling = g_speed_test_settling;
    status->speed_test_settle_timeout = g_speed_test_settle_timeout;
    status->speed_test_profile_fault = g_speed_test_profile_fault;
    status->segment_axis = g_pf.debug.segment_axis;
    status->target_idx = g_pf.target_idx;
    status->yaw_error_deg = pf_yaw_error_deg(g_pf.pose.yaw_deg,
                                              g_pf.target_yaw_deg);

    if (g_pf.path != NULL && g_pf.target_idx < g_pf.steps)
    {
        status->target_grid_valid = 1U;
        status->target_row = g_pf.path[g_pf.target_idx].row;
        status->target_col = g_pf.path[g_pf.target_idx].col;
        pf_point_to_world(g_pf.path[g_pf.target_idx],
                          g_pf.path_grid_m,
                          &status->target_x_m,
                          &status->target_y_m);
    }
}

void path_follow_draw_status(void)
{
    path_follow_status_t status = {0};

    path_follow_get_status(&status);
    ips200_show_string(0, 112, "st_x_m");
    ips200_show_float(70, 112, status.x_m, 2, 4);
    ips200_show_string(0, 128, "st_y_m");
    ips200_show_float(70, 128, status.y_m, 2, 4);
    ips200_show_string(0, 144, "st_yaw");
    ips200_show_float(70, 144, status.yaw_deg, 3, 2);
    ips200_show_string(0, 160, "target_idx");
    ips200_show_uint(90, 160, status.target_idx, 3);
    ips200_show_string(0, 176, "target_x_m");
    ips200_show_float(90, 176, status.target_x_m, 2, 3);
    ips200_show_string(0, 192, "target_y_m");
    ips200_show_float(90, 192, status.target_y_m, 2, 3);
    ips200_show_string(0, 208, "speed_ref");
    ips200_show_float(90, 208, status.speed_ref_cmps, 3, 1);
    ips200_show_string(0, 224, "seg_axis");
    ips200_show_uint(90, 224, status.segment_axis, 1);
}

void path_follow_request_bluetooth_report(void)
{
    g_bluetooth_report_pending = 1U;
}

void path_follow_service_bluetooth_report(void)
{
    /* Kept as a link-compatible hook; reporting belongs in BlueSerial.c. */
    g_bluetooth_report_pending = 0U;
}

float path_follow_heading_deg(Position from, Position to)
{
    float dx = (float)(to.row - from.row);
    float dy = (float)(to.col - from.col);

    if (dx == 0.0f && dy == 0.0f)
    {
        return 0.0f;
    }
    return atan2f(dy, dx) * (180.0f / (float)M_PI);
}

size_t path_follow_extract_corners(const Position *path,
                                   size_t path_steps,
                                   Position *corner_buffer,
                                   size_t corner_capacity)
{
    size_t i;
    size_t corner_count = 0U;

    if (path == NULL || path_steps == 0U ||
        corner_buffer == NULL || corner_capacity == 0U)
    {
        return 0U;
    }

    corner_buffer[corner_count++] = path[0];
    if (path_steps == 1U)
    {
        return corner_count;
    }

    for (i = 1U; i + 1U < path_steps; ++i)
    {
        int dx_before = path[i].row - path[i - 1U].row;
        int dy_before = path[i].col - path[i - 1U].col;
        int dx_after = path[i + 1U].row - path[i].row;
        int dy_after = path[i + 1U].col - path[i].col;

        if (dx_before != dx_after || dy_before != dy_after)
        {
            if (corner_count >= corner_capacity)
            {
                return corner_count;
            }
            corner_buffer[corner_count++] = path[i];
        }
    }

    if (corner_count < corner_capacity)
    {
        corner_buffer[corner_count++] = path[path_steps - 1U];
    }
    return corner_count;
}
