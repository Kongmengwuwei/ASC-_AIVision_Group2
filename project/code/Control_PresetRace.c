#include "Control_PresetRace.h"
#include "path.h"
#include "path_follow.h"
#include "Attitude.h"
#include <math.h>
#include <string.h>

#define CONTROL_PRESTART_OFFSET_M 0.30f
#define CONTROL_DEG_TO_RAD 0.01745329251994329577f
#define CONTROL_PRESET_LOOP_HZ 250U

/* Fixed launch direction for the school-match preset flow. */
#define CONTROL_PRESET_PRESTART_FACE PRESET_FACE_MAP_UP

extern uint8 car_go_flag;
extern uint8 car_stop_flag;

control_stage_t g_control_stage = CONTROL_STAGE_IDLE;

static volatile uint8 g_control_start_enabled = 0U;
static volatile uint8 g_path_plan_paused = 0U;

static Position g_exec_path[MAX_CAR_PATH] = {{0}};
static Position g_segment_path[MAX_CAR_PATH] = {{0}};
static size_t g_exec_steps = 0U;
static size_t g_segment_steps = 0U;

static uint8 g_start_route_index = 1U;
static uint8 g_current_route_index = 1U;
static uint8 g_prestart_move_started = 0U;
static uint8 g_initial_yaw_ready = 0U;
static float g_initial_yaw_deg = 0.0f;
static uint8 g_rotate_started = 0U;
static uint8 g_return_rotate_started = 0U;
static uint8 g_return_start_zone_started = 0U;

static size_t g_segment_start_idx = 0U;
static size_t g_segment_end_idx = 0U;
static uint32 g_wait_cycles_remaining = 0U;
static control_stage_t g_wait_next_stage = CONTROL_STAGE_IDLE;

static float clampf_local(float v, float min_v, float max_v)
{
    if (v < min_v)
        return min_v;
    if (v > max_v)
        return max_v;
    return v;
}

static float wrap_yaw_deg_local(float yaw_deg)
{
    while (yaw_deg > 180.0f)
        yaw_deg -= 360.0f;
    while (yaw_deg < -180.0f)
        yaw_deg += 360.0f;
    return yaw_deg;
}

static uint32 ms_to_loop_cycles(uint32 ms)
{
    uint32 cycles = (ms * CONTROL_PRESET_LOOP_HZ + 999U) / 1000U;
    return (cycles == 0U) ? 1U : cycles;
}

static void begin_path_plan_pause(void)
{
    g_path_plan_paused = 1U;
}

static void end_path_plan_pause(void)
{
    g_path_plan_paused = 0U;
}

static const preset_route_t *get_current_route(void)
{
    if (g_current_route_index < 1U || g_current_route_index > PRESET_ROUTE_COUNT)
    {
        return NULL;
    }
    return &g_preset_routes[g_current_route_index - 1U];
}

static uint8 point_is_valid(const preset_point_t *p)
{
    if (p == NULL)
        return 0U;
    return (p->row < MAP_ROWS && p->col < MAP_COLS) ? 1U : 0U;
}

static uint8 same_grid_point(Position p, const preset_point_t *q)
{
    if (q == NULL)
        return 0U;
    return (p.row == q->row && p.col == q->col) ? 1U : 0U;
}

static Position preset_point_to_position(const preset_point_t *p)
{
    Position out = {0U, 0U, 0U};
    if (p != NULL)
    {
        out.row = p->row;
        out.col = p->col;
        out.id = p->action;
    }
    return out;
}

static void remap_position_to_exec_pose(Position map_point, float *x_m, float *y_m)
{
    Position exec_point = map_point;
    path_remap_exec_point(&exec_point);
    if (x_m != NULL)
        *x_m = (float)exec_point.row * GRID_SIZE_M;
    if (y_m != NULL)
        *y_m = (float)exec_point.col * GRID_SIZE_M;
}

static float face_to_yaw_deg(uint8 face)
{
    float delta_yaw = 0.0f;

    switch (face)
    {
    case PRESET_FACE_MAP_UP:
        delta_yaw = 90.0f;
        break;
    case PRESET_FACE_MAP_LEFT:
        delta_yaw = 180.0f;
        break;
    case PRESET_FACE_MAP_DOWN:
        delta_yaw = -90.0f;
        break;
    case PRESET_FACE_MAP_RIGHT:
    case PRESET_FACE_KEEP:
    default:
        delta_yaw = 0.0f;
        break;
    }

    return wrap_yaw_deg_local(g_initial_yaw_deg + delta_yaw);
}

static uint8 validate_route(const preset_route_t *route)
{
    size_t i;

    if (route == NULL || route->points == NULL || route->count < 2U ||
        route->count > MAX_CAR_PATH)
    {
        return 0U;
    }
    if (!same_grid_point(route->landing_grid, &route->points[0]) ||
        !same_grid_point(route->landing_grid, &route->points[route->count - 1U]))
    {
        return 0U;
    }

    for (i = 0U; i < route->count; i++)
    {
        if (!point_is_valid(&route->points[i]))
        {
            return 0U;
        }
    }

    return 1U;
}

static void rebuild_full_exec_path_for_display(const preset_route_t *route)
{
    size_t i;

    g_exec_steps = 0U;
    memset(g_exec_path, 0, sizeof(g_exec_path));
    memset(car_path, 0, sizeof(car_path));
    Car_path_count = 0U;

    if (route == NULL || route->points == NULL)
        return;

    for (i = 0U; i < route->count && i < MAX_CAR_PATH; i++)
    {
        Position map_point = preset_point_to_position(&route->points[i]);
        Position exec_point = map_point;
        path_remap_exec_point(&exec_point);
        g_exec_path[g_exec_steps++] = exec_point;
        car_path[Car_path_count++] = map_point;
    }
}

static void begin_wait(uint32 wait_ms, control_stage_t wait_stage, control_stage_t next_stage)
{
    g_wait_cycles_remaining = ms_to_loop_cycles(wait_ms);
    g_wait_next_stage = next_stage;
    car_go_flag = 1U;
    car_stop_flag = 1U;
    g_control_stage = wait_stage;
}

static void service_wait_state(void)
{
    if (g_wait_cycles_remaining > 0U)
    {
        g_wait_cycles_remaining--;
    }
    if (g_wait_cycles_remaining == 0U)
    {
        car_stop_flag = 0U;
        g_control_stage = g_wait_next_stage;
    }
}

static void reset_segment_runtime(void)
{
    g_segment_start_idx = 0U;
    g_segment_end_idx = 0U;
    g_segment_steps = 0U;
    g_rotate_started = 0U;
    g_return_rotate_started = 0U;
    g_return_start_zone_started = 0U;
    memset(g_segment_path, 0, sizeof(g_segment_path));
}

static void reset_control_runtime_state(void)
{
    g_control_start_enabled = 0U;
    g_path_plan_paused = 0U;
    g_control_stage = CONTROL_STAGE_IDLE;
    g_current_route_index = g_start_route_index;
    g_prestart_move_started = 0U;
    g_initial_yaw_ready = 0U;
    g_initial_yaw_deg = 0.0f;
    g_wait_cycles_remaining = 0U;
    g_wait_next_stage = CONTROL_STAGE_IDLE;
    g_exec_steps = 0U;
    memset(g_exec_path, 0, sizeof(g_exec_path));
    reset_segment_runtime();

    path_follow_set_pause_indices(NULL, 0U, 0U);
    path_follow_set_path(NULL, 0U);

    car_go_flag = 0U;
    car_stop_flag = 0U;
}

static void handle_prestart_move(void)
{
    path_follow_status_t st = {0};
    float move_yaw_deg;
    float move_yaw_rad;
    float target_x_m;
    float target_y_m;

    if (!g_prestart_move_started)
    {
        path_follow_get_status(&st);

        g_initial_yaw_deg = eulerAngle.yaw;
        g_initial_yaw_ready = 1U;
        move_yaw_deg = face_to_yaw_deg(CONTROL_PRESET_PRESTART_FACE);
        move_yaw_rad = move_yaw_deg * CONTROL_DEG_TO_RAD;

        target_x_m = clampf_local(st.x_m + cosf(move_yaw_rad) * CONTROL_PRESTART_OFFSET_M,
                                  0.0f,
                                  PATH_WORLD_X_MAX_M);
        target_y_m = clampf_local(st.y_m + sinf(move_yaw_rad) * CONTROL_PRESTART_OFFSET_M,
                                  0.0f,
                                  PATH_WORLD_Y_MAX_M);

        car_go_flag = 1U;
        car_stop_flag = 0U;
        path_follow_reset_pose(st.x_m, st.y_m, g_initial_yaw_deg);
        path_follow_hold_current_yaw();
        path_follow_start_pose_correction(target_x_m, target_y_m);
        g_prestart_move_started = 1U;
        return;
    }

    path_follow_get_status(&st);
    if (!st.active)
    {
        car_go_flag = 0U;
        car_stop_flag = 0U;
        g_control_stage = CONTROL_STAGE_SNAP_TO_LANDING;
    }
}

static void handle_snap_to_landing(void)
{
    const preset_route_t *route = get_current_route();
    float x_m = 0.0f;
    float y_m = 0.0f;
    uint16 pause_ms;

    if (!validate_route(route))
    {
        g_control_stage = CONTROL_STAGE_ERROR;
        return;
    }

    begin_path_plan_pause();
    rebuild_full_exec_path_for_display(route);
    reset_segment_runtime();
    car = route->landing_grid;
    remap_position_to_exec_pose(route->landing_grid, &x_m, &y_m);
    path_follow_reset_pose(x_m, y_m, g_initial_yaw_ready ? g_initial_yaw_deg : eulerAngle.yaw);
    path_follow_hold_current_yaw();
    end_path_plan_pause();

    pause_ms = (route->pre_identify_pause_ms > 0U) ?
               route->pre_identify_pause_ms :
               PRESET_DEFAULT_PRE_IDENTIFY_PAUSE_MS;
    begin_wait(pause_ms,
               CONTROL_STAGE_PRE_IDENTIFY_PAUSE,
               CONTROL_STAGE_LOAD_SEGMENT);
}

static uint8 route_point_is_stop(const preset_point_t *p)
{
    if (p == NULL)
        return 0U;
    return (p->action == PRESET_ACTION_FAKE_IDENTIFY ||
            p->action == PRESET_ACTION_STOP_ONLY) ? 1U : 0U;
}

static uint8 load_current_segment(void)
{
    const preset_route_t *route = get_current_route();
    size_t i;
    size_t out_i = 0U;

    if (!validate_route(route) || g_segment_start_idx >= (route->count - 1U))
    {
        return 0U;
    }

    g_segment_end_idx = route->count - 1U;
    for (i = g_segment_start_idx + 1U; i < route->count; i++)
    {
        if (route_point_is_stop(&route->points[i]) ||
            route->points[i].action == PRESET_ACTION_ROUTE_END ||
            i == (route->count - 1U))
        {
            g_segment_end_idx = i;
            break;
        }
    }

    memset(g_segment_path, 0, sizeof(g_segment_path));
    for (i = g_segment_start_idx; i <= g_segment_end_idx && out_i < MAX_CAR_PATH; i++)
    {
        Position exec_point = preset_point_to_position(&route->points[i]);
        path_remap_exec_point(&exec_point);
        g_segment_path[out_i++] = exec_point;
    }

    if (out_i < 2U)
    {
        return 0U;
    }

    g_segment_steps = out_i;
    path_follow_hold_current_yaw();
    path_follow_set_path(g_segment_path, g_segment_steps);
    car_go_flag = 1U;
    car_stop_flag = 0U;
    g_control_stage = CONTROL_STAGE_RUN_SEGMENT;
    return 1U;
}

static void handle_run_segment(void)
{
    const preset_route_t *route = get_current_route();
    const preset_point_t *endpoint;
    path_follow_status_t st = {0};

    if (!validate_route(route))
    {
        g_control_stage = CONTROL_STAGE_ERROR;
        return;
    }

    path_follow_get_status(&st);
    if (st.active)
    {
        return;
    }

    endpoint = &route->points[g_segment_end_idx];
    car.row = endpoint->row;
    car.col = endpoint->col;
    car.id = 0U;
    g_segment_start_idx = g_segment_end_idx;

    if (route_point_is_stop(endpoint))
    {
        g_rotate_started = 0U;
        if (endpoint->face == PRESET_FACE_KEEP)
        {
            uint16 pause_ms = (endpoint->pause_ms > 0U) ?
                              endpoint->pause_ms :
                              PRESET_DEFAULT_POINT_PAUSE_MS;
            begin_wait(pause_ms,
                       CONTROL_STAGE_PAUSE_AT_POINT,
                       CONTROL_STAGE_LOAD_SEGMENT);
        }
        else
        {
            g_control_stage = CONTROL_STAGE_ROTATE_AT_POINT;
        }
        return;
    }

    g_control_stage = CONTROL_STAGE_RETURN_TO_START_ZONE;
}

static void handle_rotate_at_point(void)
{
    const preset_route_t *route = get_current_route();
    const preset_point_t *endpoint;
    path_follow_status_t st = {0};

    if (!validate_route(route) || g_segment_start_idx >= route->count)
    {
        g_control_stage = CONTROL_STAGE_ERROR;
        return;
    }

    endpoint = &route->points[g_segment_start_idx];
    if (!g_rotate_started)
    {
        path_follow_start_rotate_to_yaw(face_to_yaw_deg(endpoint->face));
        car_go_flag = 1U;
        car_stop_flag = 0U;
        g_rotate_started = 1U;
        return;
    }

    path_follow_get_status(&st);
    if (!st.active)
    {
        uint16 pause_ms = (endpoint->pause_ms > 0U) ?
                          endpoint->pause_ms :
                          PRESET_DEFAULT_POINT_PAUSE_MS;
        begin_wait(pause_ms,
                   CONTROL_STAGE_PAUSE_AT_POINT,
                   CONTROL_STAGE_LOAD_SEGMENT);
    }
}

static void handle_return_to_start_zone(void)
{
    path_follow_status_t st = {0};
    float move_yaw_deg;
    float move_yaw_rad;
    float virtual_landing_x_m;
    float virtual_landing_y_m;
    float target_x_m;
    float target_y_m;

    if (!g_return_start_zone_started)
    {
        move_yaw_deg = face_to_yaw_deg(CONTROL_PRESET_PRESTART_FACE);
        move_yaw_rad = move_yaw_deg * CONTROL_DEG_TO_RAD;

        /* Use a virtual pose so the map-outside start zone never becomes a negative grid coordinate. */
        virtual_landing_x_m = PATH_WORLD_X_MAX_M * 0.5f;
        virtual_landing_y_m = PATH_WORLD_Y_MAX_M * 0.5f;
        target_x_m = virtual_landing_x_m - cosf(move_yaw_rad) * CONTROL_PRESTART_OFFSET_M;
        target_y_m = virtual_landing_y_m - sinf(move_yaw_rad) * CONTROL_PRESTART_OFFSET_M;
        target_x_m = clampf_local(target_x_m, 0.0f, PATH_WORLD_X_MAX_M);
        target_y_m = clampf_local(target_y_m, 0.0f, PATH_WORLD_Y_MAX_M);

        begin_path_plan_pause();
        path_follow_reset_pose(virtual_landing_x_m, virtual_landing_y_m, eulerAngle.yaw);
        path_follow_hold_current_yaw();
        path_follow_start_pose_correction(target_x_m, target_y_m);
        end_path_plan_pause();

        car_go_flag = 1U;
        car_stop_flag = 0U;
        g_return_start_zone_started = 1U;
        return;
    }

    path_follow_get_status(&st);
    if (!st.active)
    {
        car_go_flag = 0U;
        car_stop_flag = 0U;
        g_return_start_zone_started = 0U;
        g_control_stage = CONTROL_STAGE_RETURN_YAW_AT_END;
    }
}

static void handle_return_yaw_at_end(void)
{
    const preset_route_t *route = get_current_route();
    path_follow_status_t st = {0};
    uint16 wait_ms;

    if (!validate_route(route))
    {
        g_control_stage = CONTROL_STAGE_ERROR;
        return;
    }

    if (route->reset_yaw_at_end && !g_return_rotate_started)
    {
        path_follow_start_rotate_to_yaw(g_initial_yaw_deg);
        car_go_flag = 1U;
        car_stop_flag = 0U;
        g_return_rotate_started = 1U;
        return;
    }

    if (route->reset_yaw_at_end)
    {
        path_follow_get_status(&st);
        if (st.active)
        {
            return;
        }
    }

    wait_ms = (route->map_end_wait_ms > 0U) ?
              route->map_end_wait_ms :
              PRESET_DEFAULT_MAP_END_WAIT_MS;
    begin_wait(wait_ms,
               CONTROL_STAGE_MAP_END_WAIT,
               CONTROL_STAGE_NEXT_ROUTE);
}

static void handle_next_route(void)
{
    if (g_current_route_index >= PRESET_ROUTE_COUNT)
    {
        car_go_flag = 1U;
        car_stop_flag = 1U;
        g_control_stage = CONTROL_STAGE_FINISHED;
        return;
    }

    g_current_route_index++;
    reset_segment_runtime();
    g_prestart_move_started = 0U;
    g_initial_yaw_ready = 0U;
    g_control_stage = CONTROL_STAGE_PRESTART_MOVE;
}

static void handle_error(void)
{
    path_follow_set_path(NULL, 0U);
    path_follow_set_pause_indices(NULL, 0U, 0U);
    car_go_flag = 1U;
    car_stop_flag = 1U;
}

void control_init(void)
{
    reset_control_runtime_state();
}

void control_restart(void)
{
    reset_control_runtime_state();
}

void control_set_start_enabled(uint8 enabled)
{
    if (enabled)
    {
        g_control_start_enabled = 1U;
        if (g_control_stage == CONTROL_STAGE_IDLE)
        {
            g_current_route_index = g_start_route_index;
            g_prestart_move_started = 0U;
            g_initial_yaw_ready = 0U;
            reset_segment_runtime();
            g_control_stage = CONTROL_STAGE_PRESTART_MOVE;
        }
        return;
    }

    control_restart();
}

uint8 control_get_start_enabled(void)
{
    return g_control_start_enabled;
}

control_stage_t control_get_stage(void)
{
    return g_control_stage;
}

void control_process(void)
{
    if (!g_control_start_enabled)
    {
        car_go_flag = 0U;
        car_stop_flag = 0U;
        g_control_stage = CONTROL_STAGE_IDLE;
        return;
    }

    switch (g_control_stage)
    {
    case CONTROL_STAGE_PRESTART_MOVE:
        handle_prestart_move();
        break;
    case CONTROL_STAGE_SNAP_TO_LANDING:
        handle_snap_to_landing();
        break;
    case CONTROL_STAGE_PRE_IDENTIFY_PAUSE:
    case CONTROL_STAGE_PAUSE_AT_POINT:
    case CONTROL_STAGE_MAP_END_WAIT:
        service_wait_state();
        break;
    case CONTROL_STAGE_LOAD_SEGMENT:
        if (!load_current_segment())
        {
            g_control_stage = CONTROL_STAGE_RETURN_YAW_AT_END;
        }
        break;
    case CONTROL_STAGE_RUN_SEGMENT:
        handle_run_segment();
        break;
    case CONTROL_STAGE_ROTATE_AT_POINT:
        handle_rotate_at_point();
        break;
    case CONTROL_STAGE_RETURN_TO_START_ZONE:
        handle_return_to_start_zone();
        break;
    case CONTROL_STAGE_RETURN_YAW_AT_END:
        handle_return_yaw_at_end();
        break;
    case CONTROL_STAGE_NEXT_ROUTE:
        handle_next_route();
        break;
    case CONTROL_STAGE_ERROR:
        handle_error();
        break;
    case CONTROL_STAGE_FINISHED:
        car_go_flag = 1U;
        car_stop_flag = 1U;
        break;
    case CONTROL_STAGE_IDLE:
    default:
        break;
    }
}

const Position *control_get_exec_path(size_t *steps)
{
    if (steps != NULL)
        *steps = g_exec_steps;
    return g_exec_path;
}

uint8 control_is_path_plan_paused(void)
{
    return g_path_plan_paused;
}

void control_set_start_route_index(uint8 route_index)
{
    if (route_index < CONTROL_PRESET_START_ROUTE_MIN)
        route_index = CONTROL_PRESET_START_ROUTE_MIN;
    if (route_index > CONTROL_PRESET_START_ROUTE_MAX)
        route_index = CONTROL_PRESET_START_ROUTE_MAX;
    g_start_route_index = route_index;
    if (g_control_stage == CONTROL_STAGE_IDLE)
        g_current_route_index = g_start_route_index;
}

uint8 control_get_start_route_index(void)
{
    return g_start_route_index;
}

uint8 control_get_current_route_index(void)
{
    return g_current_route_index;
}

void control_set_prestart_depart_dir(uint8 dir)
{
    (void)dir;
}

uint8 control_get_prestart_depart_dir(void)
{
    return 0U;
}

void control_set_diagonal_path_enabled(uint8 enabled)
{
    (void)enabled;
}

uint8 control_get_diagonal_path_enabled(void)
{
    return 0U;
}
