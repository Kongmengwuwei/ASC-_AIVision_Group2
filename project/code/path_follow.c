

#include "path_follow.h"
#include "Motor.h"
#include "pid.h"
#include "Attitude.h"
#include "zf_device_ips200.h"

#include <math.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define PF_POSITION_TOLERANCE_M          0.015f
#define PF_YAW_TOLERANCE_DEG             0.5f
#define PF_MAX_LINEAR_SPEED_MPS          3.0f
#define PF_MAX_ANGULAR_SPEED_DEGPS       300.0f
#define PF_TEMP_PATH_GRID_M              0.01f
#define PF_MIN_VALID_GRID_M              0.0001f
#define PF_INVALID_INDEX                 ((size_t)-1)
#define PF_INVALID_BAND                  0xFFU
#define PF_DT_S                          (1.0f / (float)PID_RATE)
#define PF_PROFILE_DONE_MIN_SPEED_CMPS   0.0f
#define PF_SEGMENT_END_SPEED_CMPS        0.0f
#define PF_POSITION_LOOP_RELEASE_M       0.20f
#define PF_POSITION_KP                   1.1f
#define PF_POSITION_KI                   0.0f
#define PF_POSITION_KD                   0.25f
#define PF_POSITION_MAX_IOUT_CMPS        200.0f
#define PF_POSITION_MAX_OUT_CMPS         200.0f
#define PF_POSITION_FILTER_ALPHA         0.9f
#define PF_LINE_GUIDE_MAX_CMPS           12.0f
#define PF_LINE_GUIDE_DEADBAND_M         0.0025f

#define PF_YAW_KP                        6.0f
#define PF_YAW_KI                        0.0f
#define PF_YAW_KD                        10.5f
#define PF_YAW_FILTER_ALPHA              0.9f
#define PF_YAW_FEEDFORWARD_MIN_DEGPS     5.0f

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
    uint8 segment_axis;
} pf_geometry_t;

typedef struct
{
    float distance_m;
    float dir_x;
    float dir_y;
    float speed_ref_cmps;
    float target_x_m;
    float target_y_m;
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

    pf_pose_t pose;
    pf_debug_t debug;
    pf_scurve_runtime_cfg_t active_scurve_cfg;
    pf_scurve_profile_t active_profile;
    float profile_time_s;
    float last_ref_speed_cmps;
    size_t profile_target_idx;

    size_t pause_indices[PATH_FOLLOW_MAX_PAUSE_POINTS];
    size_t pause_count;
    size_t pause_cursor;
    uint32 pause_cycles_cfg;
    uint32 pause_cycles_left;

    uint8 active;
    uint8 paused;
    uint8 pause_events_enabled;
    uint8 rotate_only_active;
    uint8 profile_active;
} pf_context_t;

static pf_context_t g_pf;
static Position g_single_target_path[2];
static Position g_offset_path[3];
static Position g_pose_move_path[3];
static uint8 g_stationary_yaw_hold_enabled;
static volatile uint8 g_bluetooth_report_pending;

/* Public compatibility objects declared by path_follow.h. */
tagPID_T pid_world_x;
tagPID_T pid_world_y;
tagPID_T pid_stay;
static tagPID_T pid_stay_y;
tagPID_T pid_yaw;
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
float path_line_guide_kp = 0.0f;
float path_line_guide_min_cmps = 0.0f;
float path_yaw_feedforward_min_degps = PF_YAW_FEEDFORWARD_MIN_DEGPS;
float path_yaw_feedforward_deadband_deg = PF_YAW_TOLERANCE_DEG;

/* Other legacy compensation variables remain link-compatible but unused. */
float path_yaw_target_base_comp_deg = 0.0f;
float path_yaw_target_error_comp_k = 0.0f;
float path_rotate_center_offset_x_cm = 0.0f;
float path_rotate_center_offset_y_cm = 0.0f;

static const path_follow_scurve_band_cfg_t g_default_speed_bands[PATH_FOLLOW_SCURVE_BAND_COUNT] =
{
    {0.30f,   0.40f, 1.05f, 3.50f},
    {0.50f,   0.50f, 1.05f, 3.50f},
    {0.70f,   0.65f, 1.05f, 3.50f},
    {0.90f,   0.75f, 1.05f, 3.50f},
    {1000.0f, 1.50f, 1.05f, 3.50f}
};

path_follow_scurve_band_cfg_t g_path_follow_scurve_band_cfg[PATH_FOLLOW_SCURVE_BAND_COUNT] =
{
    {0.30f,   0.40f, 1.05f, 3.50f},
    {0.50f,   0.50f, 1.05f, 3.50f},
    {0.70f,   0.65f, 1.05f, 3.50f},
    {0.90f,   0.75f, 1.05f, 3.50f},
    {1000.0f, 1.50f, 1.05f, 3.50f}
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

static void pf_reset_speed(void)
{
    memset(&g_pf.active_profile, 0, sizeof(g_pf.active_profile));
    memset(&g_pf.active_scurve_cfg, 0, sizeof(g_pf.active_scurve_cfg));
    g_pf.active_scurve_cfg.band_idx = PF_INVALID_BAND;
    g_pf.profile_time_s = 0.0f;
    g_pf.last_ref_speed_cmps = 0.0f;
    g_pf.profile_target_idx = PF_INVALID_INDEX;
    g_pf.profile_active = 0U;
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
    PID_Clear(&pid_world_x);
    PID_Clear(&pid_world_y);
    PID_Clear(&pid_stay);
    PID_Clear(&pid_stay_y);
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

static uint8 pf_segment_axis(float dx_m, float dy_m)
{
    const float axis_epsilon_m = 0.001f;

    if (fabsf(dx_m) <= axis_epsilon_m && fabsf(dy_m) <= axis_epsilon_m)
    {
        return 0U;
    }
    if (fabsf(dy_m) <= axis_epsilon_m)
    {
        return 1U;
    }
    if (fabsf(dx_m) <= axis_epsilon_m)
    {
        return 2U;
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

static float pf_brake_speed_cap(float distance_m,
                                float end_speed_cmps,
                                float max_speed_cmps,
                                float accel_cmpss)
{
    float distance_cm = fmaxf(distance_m, 0.0f) * 100.0f;
    float speed_sq;

    accel_cmpss = fmaxf(accel_cmpss, 1.0f);
    speed_sq = end_speed_cmps * end_speed_cmps +
               2.0f * accel_cmpss * distance_cm;
    return fminf(max_speed_cmps, sqrtf(fmaxf(speed_sq, 0.0f)));
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
    profile_distance_m = geometry->distance_m;
    pf_select_scurve_band(profile_distance_m, &selected_cfg);
    g_pf.active_scurve_cfg = selected_cfg;

    speed_plan->end_speed_cmps = fminf(pf_segment_end_speed_cmps(),
                                       g_pf.active_scurve_cfg.max_speed_cmps);
    speed_plan->safety_cap_cmps = pf_brake_speed_cap(geometry->distance_m,
                                                      speed_plan->end_speed_cmps,
                                                      g_pf.active_scurve_cfg.max_speed_cmps,
                                                      g_pf.active_scurve_cfg.accel_cmpss);
    start_speed_cmps = fminf(fmaxf(g_pf.last_ref_speed_cmps, 0.0f),
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
        speed_plan->safety_cap_cmps = pf_brake_speed_cap(geometry->distance_m,
                                                          speed_plan->end_speed_cmps,
                                                          g_pf.active_scurve_cfg.max_speed_cmps,
                                                          g_pf.active_scurve_cfg.accel_cmpss);
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
        speed_plan->safety_cap_cmps = pf_brake_speed_cap(geometry->distance_m,
                                                          speed_plan->end_speed_cmps,
                                                          g_pf.active_scurve_cfg.max_speed_cmps,
                                                          g_pf.active_scurve_cfg.accel_cmpss);
    }
    else if (!build_ok)
    {
        speed_plan->ref_speed_cmps = pf_profile_fault_speed(g_pf.last_ref_speed_cmps,
                                                             speed_plan->safety_cap_cmps,
                                                             g_pf.active_scurve_cfg.accel_cmpss);
    }

    speed_plan->ref_speed_cmps = pf_clamp(speed_plan->ref_speed_cmps,
                                           -g_pf.active_scurve_cfg.max_speed_cmps,
                                           g_pf.active_scurve_cfg.max_speed_cmps);
    speed_plan->ref_speed_cmps = fminf(speed_plan->ref_speed_cmps,
                                       speed_plan->safety_cap_cmps);
    speed_plan->ref_speed_cmps = fmaxf(speed_plan->ref_speed_cmps, 0.0f);

    if (g_pf.profile_active && g_pf.active_profile.T > 0.0f &&
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
    if (geometry->distance_m <= g_pf.position_tolerance_m)
    {
        speed_plan->ref_speed_cmps = 0.0f;
    }
    g_pf.last_ref_speed_cmps = speed_plan->ref_speed_cmps;
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

static void pf_sync_position_pid_gains(void)
{
    /* pid_stay is the public tuning handle; Y keeps independent history. */
    pid_stay_y.fKp = pid_stay.fKp;
    pid_stay_y.fKi = pid_stay.fKi;
    pid_stay_y.fKd = pid_stay.fKd;
    pid_stay_y.fMax_Iout = pid_stay.fMax_Iout;
    pid_stay_y.fMax_Out = pid_stay.fMax_Out;
    pid_stay_y.alpha = pid_stay.alpha;
}

static float pf_position_loop_blend(float distance_m)
{
    float release_m = path_hold_trim_release_distance;

    if (distance_m >= release_m)
    {
        return 0.0f;
    }
    if (distance_m <= g_pf.position_tolerance_m)
    {
        return 1.0f;
    }
    if (release_m <= g_pf.position_tolerance_m)
    {
        return 1.0f;
    }
    return 1.0f - ((distance_m - g_pf.position_tolerance_m) /
                   (release_m - g_pf.position_tolerance_m));
}

static void pf_apply_position_loop(const pf_geometry_t *geometry,
                                   float *vx_world_cmps,
                                   float *vy_world_cmps)
{
    float blend;

    if (geometry == NULL || vx_world_cmps == NULL || vy_world_cmps == NULL)
    {
        return;
    }

    blend = pf_position_loop_blend(geometry->distance_m);
    if (blend <= 0.0f)
    {
        return;
    }

    *vx_world_cmps += blend *
        (float)PID_Location_Calculate(&pid_stay,
                                      g_pf.pose.x_m * 100.0f,
                                      geometry->target_x_m * 100.0f);
    *vy_world_cmps += blend *
        (float)PID_Location_Calculate(&pid_stay_y,
                                      g_pf.pose.y_m * 100.0f,
                                      geometry->target_y_m * 100.0f);
}

static uint8 pf_apply_line_guidance(const pf_geometry_t *geometry,
                                    float ref_speed_cmps,
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
    }
    trim_cmps = fminf(trim_cmps, PF_LINE_GUIDE_MAX_CMPS);
    if (effective_error_m > 0.0f)
    {
        trim_cmps = -trim_cmps;
    }

    *vx_world_cmps = ref_speed_cmps * tangent_x + trim_cmps * normal_x;
    *vy_world_cmps = ref_speed_cmps * tangent_y + trim_cmps * normal_y;
    return 1U;
}

static void pf_limit_world_speed(float *vx_world_cmps, float *vy_world_cmps)
{
    float max_speed_cmps;
    float speed_norm_cmps;
    float scale;

    if (vx_world_cmps == NULL || vy_world_cmps == NULL)
    {
        return;
    }

    max_speed_cmps = g_pf.active_scurve_cfg.max_speed_cmps;
    if (max_speed_cmps <= 0.0f)
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

    scale = max_speed_cmps / speed_norm_cmps;
    *vx_world_cmps *= scale;
    *vy_world_cmps *= scale;
}

static float pf_yaw_command_radps(float yaw_deg)
{
    float error_deg = pf_yaw_error_deg(yaw_deg, g_pf.target_yaw_deg);
    float output_degps;
    float feedforward_deadband_deg;

    if (fabsf(error_deg) <= g_pf.yaw_tolerance_deg)
    {
        return 0.0f;
    }

    output_degps = (float)PID_Location_Calculate(&pid_yaw, 0.0f, error_deg);
    feedforward_deadband_deg = fmaxf(path_yaw_feedforward_deadband_deg,
                                    g_pf.yaw_tolerance_deg);
    if (path_yaw_feedforward_min_degps > 0.0f &&
        fabsf(error_deg) > feedforward_deadband_deg &&
        fabsf(output_degps) < path_yaw_feedforward_min_degps)
    {
        output_degps = (error_deg > 0.0f) ?
                       path_yaw_feedforward_min_degps :
                       -path_yaw_feedforward_min_degps;
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
    vx_body_mps += LATERAL_TO_LONGITUDINAL_COUPLING_FACTOR * vy_body_mps;

    yaw_rad = yaw_deg * ((float)M_PI / 180.0f);
    cos_yaw = cosf(yaw_rad);
    sin_yaw = sinf(yaw_rad);

    g_pf.pose.x_m += (vx_body_mps * cos_yaw - vy_body_mps * sin_yaw) * PF_DT_S;
    g_pf.pose.y_m += (vx_body_mps * sin_yaw + vy_body_mps * cos_yaw) * PF_DT_S;
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
        out->omega_cmd = pf_yaw_command_radps(yaw_deg);
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
        geometry->planned_distance_m =
            sqrtf((geometry->target_x_m - segment_start_x_m) *
                  (geometry->target_x_m - segment_start_x_m) +
                  (geometry->target_y_m - segment_start_y_m) *
                  (geometry->target_y_m - segment_start_y_m));

        if (geometry->distance_m <= g_pf.position_tolerance_m)
        {
            if (!pf_advance_reached_target(out))
            {
                return 0U;
            }
            continue;
        }

        geometry->dir_x = geometry->dx_m / geometry->distance_m;
        geometry->dir_y = geometry->dy_m / geometry->distance_m;
        geometry->segment_axis = pf_segment_axis(geometry->dx_m, geometry->dy_m);
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
                          uint8 pause_events_enabled)
{
    g_pf.path = path;
    g_pf.steps = steps;
    g_pf.target_idx = 0U;
    g_pf.path_grid_m = (grid_m > PF_MIN_VALID_GRID_M) ? grid_m : g_pf.default_grid_m;
    g_pf.path_origin_x_m = 0.0f;
    g_pf.path_origin_y_m = 0.0f;
    g_pf.pause_events_enabled = pause_events_enabled ? 1U : 0U;
    g_pf.rotate_only_active = 0U;
    g_pf.active = (path != NULL && steps > 0U) ? 1U : 0U;

    pf_reset_pause_runtime();
    pf_reset_motion_state();
    PID_Clear(&pid_yaw);

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
    uint8 x_first = (fabsf(dx_m) >= fabsf(dy_m)) ? 1U : 0U;
    size_t count = 1U;

    if (buffer == NULL || capacity < 3U)
    {
        return;
    }

    if (fabsf(dx_m) <= epsilon_m && fabsf(dy_m) <= epsilon_m)
    {
        pf_apply_path(NULL, 0U, PF_TEMP_PATH_GRID_M, 0U);
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

    if (x_first)
    {
        if (fabsf(dx_m) > epsilon_m)
        {
            buffer[count].row = (uint8)target_row;
            buffer[count].col = buffer[0].col;
            ++count;
        }
        if (fabsf(dy_m) > epsilon_m)
        {
            buffer[count].row = (uint8)target_row;
            buffer[count].col = (uint8)target_col;
            ++count;
        }
    }
    else
    {
        if (fabsf(dy_m) > epsilon_m)
        {
            buffer[count].row = buffer[0].row;
            buffer[count].col = (uint8)target_col;
            ++count;
        }
        if (fabsf(dx_m) > epsilon_m)
        {
            buffer[count].row = (uint8)target_row;
            buffer[count].col = (uint8)target_col;
            ++count;
        }
    }

    pf_apply_path(buffer, count, local_grid_m, 0U);
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

    /* Near-target X/Y position loop restored from the original controller. */
    pf_init_pid_object(&pid_stay,
                       PF_POSITION_KP,
                       PF_POSITION_KI,
                       PF_POSITION_KD,
                       PF_POSITION_MAX_OUT_CMPS,
                       PF_POSITION_FILTER_ALPHA);
    pid_stay.fMax_Iout = PF_POSITION_MAX_IOUT_CMPS;
    pf_init_pid_object(&pid_stay_y,
                       PF_POSITION_KP,
                       PF_POSITION_KI,
                       PF_POSITION_KD,
                       PF_POSITION_MAX_OUT_CMPS,
                       PF_POSITION_FILTER_ALPHA);
    pid_stay_y.fMax_Iout = PF_POSITION_MAX_IOUT_CMPS;

    /* Historical world PID objects remain link-compatible but inactive. */
    pf_init_pid_object(&pid_world_x, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    pf_init_pid_object(&pid_world_y, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    pf_init_pid_object(&pid_accel_yaw, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
}

void path_follow_reset_pose(float x_m, float y_m, float yaw_deg)
{
    g_pf.pose.x_m = x_m;
    g_pf.pose.y_m = y_m;
    g_pf.pose.yaw_deg = pf_wrap_deg(yaw_deg);
    pf_reset_motion_state();
    PID_Clear(&pid_yaw);
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
    pf_apply_path(path, steps, g_pf.default_grid_m, pause_events_enabled);
}

void path_follow_set_path_with_grid(const Position *path,
                                    size_t steps,
                                    float grid_m,
                                    uint8 pause_events_enabled)
{
    pf_apply_path(path, steps, grid_m, pause_events_enabled);
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
    pf_apply_path(g_single_target_path, 2U, g_pf.default_grid_m, 0U);
}

void path_follow_set_target_yaw(float target_yaw_deg)
{
    g_pf.target_yaw_deg = pf_wrap_deg(target_yaw_deg);
}

void path_follow_hold_current_yaw(void)
{
    path_follow_set_target_yaw(g_pf.pose.yaw_deg);
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
    pf_apply_path(NULL, 0U, g_pf.default_grid_m, 0U);
    path_follow_set_target_yaw(target_yaw_deg);
    PID_Clear(&pid_yaw);
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

    pf_clear_output(out);
    pf_update_odometry(yaw_deg);

    if (g_pf.rotate_only_active)
    {
        yaw_error_deg = pf_yaw_error_deg(yaw_deg, g_pf.target_yaw_deg);
        if (fabsf(yaw_error_deg) <= g_pf.yaw_tolerance_deg)
        {
            g_pf.rotate_only_active = 0U;
            PID_Clear(&pid_yaw);
            if (out != NULL)
            {
                out->reached = 1U;
            }
            return;
        }

        pf_output_yaw_hold(yaw_deg, out);
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

    pf_sync_position_pid_gains();
    pf_plan_scurve_speed(&geometry, &speed_plan);
    vx_world_cmps = speed_plan.ref_speed_cmps * geometry.dir_x;
    vy_world_cmps = speed_plan.ref_speed_cmps * geometry.dir_y;
    if (pf_position_loop_blend(geometry.distance_m) <= 0.0f)
    {
        (void)pf_apply_line_guidance(&geometry,
                                     speed_plan.ref_speed_cmps,
                                     &vx_world_cmps,
                                     &vy_world_cmps);
    }
    pf_apply_position_loop(&geometry, &vx_world_cmps, &vy_world_cmps);
    pf_limit_world_speed(&vx_world_cmps, &vy_world_cmps);

    g_pf.debug.distance_m = geometry.distance_m;
    g_pf.debug.dir_x = geometry.dir_x;
    g_pf.debug.dir_y = geometry.dir_y;
    g_pf.debug.speed_ref_cmps = speed_plan.ref_speed_cmps;
    g_pf.debug.target_x_m = geometry.target_x_m;
    g_pf.debug.target_y_m = geometry.target_y_m;
    g_pf.debug.segment_axis = geometry.segment_axis;

    if (out != NULL)
    {
        pf_world_to_body(vx_world_cmps,
                         vy_world_cmps,
                         yaw_deg,
                         &out->vx_cmd,
                         &out->vy_cmd);
        out->omega_cmd = pf_yaw_command_radps(yaw_deg);
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
    status->target_yaw_deg = g_pf.target_yaw_deg;
    status->active = (g_pf.active || g_pf.rotate_only_active) ? 1U : 0U;
    status->reached = status->active ? 0U : 1U;
    status->paused = g_pf.paused;
    status->yaw_only_active = g_pf.rotate_only_active;
    status->segment_axis = g_pf.debug.segment_axis;
    status->target_idx = g_pf.target_idx;
    status->yaw_error_deg = pf_yaw_error_deg(g_pf.pose.yaw_deg,
                                              g_pf.target_yaw_deg);

    if (g_pf.path != NULL && g_pf.target_idx < g_pf.steps)
    {
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
