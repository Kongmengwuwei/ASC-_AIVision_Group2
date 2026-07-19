#ifndef PATH_FOLLOW_H
#define PATH_FOLLOW_H

#include <stddef.h>
#include "zf_common_typedef.h"
#include "Map_Path_Data.h"
#include "PID.h"

#define PATH_FOLLOW_SCURVE_BAND_COUNT 5U
#define PATH_FOLLOW_MAX_PAUSE_POINTS 20U
#define PATH_FOLLOW_AXIS_AUTO 0U
#define PATH_FOLLOW_AXIS_X    1U
#define PATH_FOLLOW_AXIS_Y    2U

typedef struct
{
    float distance_upper_m;
    float vmax_mps;
    float amax_mps2;
    float jmax_mps3;
} path_follow_scurve_band_cfg_t;

typedef struct
{
    float vx_cmd;
    float vy_cmd;
    float omega_cmd;
    uint8 active;
    uint8 reached;
    size_t target_idx;
} path_follow_output_t;

typedef struct
{
    float x_m;
    float y_m;
    float yaw_deg;
    float target_x_m;
    float target_y_m;
    float distance_m;
    float dir_x;
    float dir_y;
    float speed_ref_cmps;
    float actual_speed_cmps;
    float speed_cap_cmps;
    float position_speed_limit_cmps;
    float actual_along_speed_cmps;
    float position_blend;
    float position_speed_factor;
    float position_speed_factor_x;
    float position_speed_factor_y;
    float target_yaw_deg;
    float speed_test_profile_time_s;
    float speed_test_profile_total_s;
    uint8 active;
    uint8 reached;
    uint8 paused;
    uint8 yaw_only_active;
    uint8 speed_limit_active;
    uint8 overspeed_guard_active;
    uint8 approach_braking_active;
    uint8 position_reverse_requested;
    uint8 target_plane_crossed;
    uint8 position_loop_active;
    uint8 line_guidance_active;
    uint8 speed_test_enabled;
    uint8 speed_test_yaw_enabled;
    uint8 speed_test_settling;
    uint8 speed_test_settle_timeout;
    uint8 speed_test_profile_fault;
    uint8 segment_axis;
    uint8 target_grid_valid;
    uint8 target_row;
    uint8 target_col;
    size_t target_idx;
    float yaw_error_deg;
} path_follow_status_t;

void path_follow_init(float grid_size_m, float pulses_per_meter);
void path_follow_set_position_pid_gains(float kp, float ki, float kd);
void path_follow_set_position_pid_gains_x(float kp, float ki, float kd);
void path_follow_set_position_pid_gains_y(float kp, float ki, float kd);
void path_follow_set_position_pid_gains_forward(float kp, float ki, float kd);
void path_follow_set_position_pid_gains_backward(float kp, float ki, float kd);
void path_follow_set_rotate_yaw_pid_gains(float kp, float ki, float kd);
void path_follow_get_rotate_yaw_pid_gains(float *kp, float *ki, float *kd);
void path_follow_set_rotate_yaw_feedforward(float feedforward_degps);
float path_follow_get_rotate_yaw_feedforward(void);
void path_follow_set_position_speed_factor(float factor);
void path_follow_set_position_speed_factor_x(float factor);
void path_follow_set_position_speed_factor_y(float factor);
float path_follow_get_position_speed_factor(void);
float path_follow_get_position_speed_factor_x(void);
float path_follow_get_position_speed_factor_y(void);
void path_follow_set_speed_test_mode(uint8 enabled, uint8 yaw_enabled);
void path_follow_set_path(const Position *path, size_t steps);
void path_follow_set_path_pause_enabled(const Position *path, size_t steps, uint8 pause_events_enabled);
void path_follow_set_path_with_grid(const Position *path, size_t steps, float grid_m, uint8 pause_events_enabled);
void path_follow_set_path_with_grid_axis(const Position *path, size_t steps, float grid_m,
                                         uint8 pause_events_enabled, uint8 segment_axis);
void path_follow_set_target(int target_row, int target_col);
void path_follow_set_target_yaw(float target_yaw_deg);
void path_follow_set_stationary_yaw_hold_enabled(uint8 enabled);
uint8 path_follow_get_stationary_yaw_hold_enabled(void);
void path_follow_start_rotate_to_yaw(float target_yaw_deg);
void path_follow_start_offset_move(float delta_x_m, float delta_y_m);
void path_follow_start_pose_correction(float target_x_m, float target_y_m);
void path_follow_set_pause_indices(const size_t *pause_indices, size_t pause_count, uint32 pause_ms);
void path_follow_reset_pose(float x_m, float y_m, float yaw_deg);
void path_follow_set_external_position(float x_m, float y_m, uint8 valid);
void path_follow_update(float yaw_deg, path_follow_output_t *out);
void path_follow_update_yaw_hold(float yaw_deg, path_follow_output_t *out);
void path_follow_get_status(path_follow_status_t *status);
void path_follow_draw_status(void);
void path_follow_request_bluetooth_report(void);
void path_follow_service_bluetooth_report(void);
float path_follow_heading_deg(Position from, Position to);
void distance_speed_strategy(void);
void path_follow_reset_scurve_band_defaults(void);
void path_follow_sanitize_scurve_band_cfg(void);

size_t path_follow_extract_corners(const Position *path, size_t path_steps,
                                   Position *corner_buffer, size_t corner_capacity);

extern tagPID_T pid_world_x;
extern tagPID_T pid_world_y;
extern tagPID_T pid_stay;
extern tagPID_T pid_stay_y;
extern tagPID_T pid_stay_backward;
extern tagPID_T pid_yaw;
extern tagPID_T pid_accel_yaw;
extern uint8 wait_stop;
extern float prestart_move_left_m;
extern float prestart_move_right_m;
extern float prestart_move_forward_m;
extern float prestart_move_backward_m;
extern float path_corner_commit_lateral_gate_min_m;
extern float path_hold_trim_release_distance;
extern float path_line_guide_kp;
extern float path_line_guide_min_cmps;
extern float path_yaw_feedforward_min_degps;
extern float path_yaw_feedforward_deadband_deg;
extern float path_y_crosstalk_left_x_comp_k;
extern float path_y_crosstalk_right_x_comp_k;
extern float path_yaw_target_base_comp_deg;
extern float path_yaw_target_error_comp_k;
extern float path_rotate_center_offset_x_cm;
extern float path_rotate_center_offset_y_cm;
extern path_follow_scurve_band_cfg_t g_path_follow_scurve_band_cfg[PATH_FOLLOW_SCURVE_BAND_COUNT];

#endif
