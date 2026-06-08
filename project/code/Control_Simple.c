#include "Control_Simple.h"
#include "path_follow.h"
#include "path.h"
#include "Attitude.h"
#include "zf_driver_delay.h"
#include <math.h>
#include <string.h>

/* ========================= 常量 ========================= */

#define CONTROL_PRESTART_OFFSET_M           0.30f
#define CONTROL_DEG_TO_RAD                  0.01745329251994329577f
#define CONTROL_FAKE_IDENTIFY_PAUSE_MS      2000U
#define CONTROL_ERROR_RETRY_DELAY_MS        3000U

#define CONTROL_WORLD_X_MAX_M  ((float)MAP_ROWS * GRID_SIZE_M)
#define CONTROL_WORLD_Y_MAX_M  ((float)MAP_COLS * GRID_SIZE_M)

/* ========================= 地图方向枚举（内部使用） ========================= */

typedef enum
{
    CONTROL_MAP_DIR_RIGHT = 0U,
    CONTROL_MAP_DIR_UP    = 1U,
    CONTROL_MAP_DIR_LEFT  = 2U,
    CONTROL_MAP_DIR_DOWN  = 3U
} control_map_dir_t;

typedef enum
{
    EXEC_PREPARE = 0U,
    EXEC_MOVING,
    EXEC_ROTATING,
    EXEC_PAUSING,
} exec_phase_t;

/* ========================= 外部全局引用 ========================= */

extern uint8 car_go_flag;
extern uint8 car_stop_flag;

/* ========================= 内部状态变量 ========================= */

control_stage_t g_control_stage = CONTROL_STAGE_IDLE;

static volatile uint8 g_control_start_enabled     = 0U;
static volatile uint8 g_path_plan_paused          = 0U;
static Position        g_exec_path[MAX_CAR_PATH]  = {{0}};
static size_t          g_exec_steps               = 0U;
static uint8           g_plan_ready               = 0U;
static float           g_map_right_yaw_deg        = 0.0f;
static uint8           g_map_right_yaw_ready      = 0U;
static uint8           g_prestart_move_started    = 0U;
static uint8           g_fake_identify_started    = 0U;
static uint8           g_diagonal_enabled         = 1U;
static uint8           g_control_prestart_depart_dir = 0U;

/* 定点转向相关 */
typedef struct
{
    size_t            index;
    control_map_dir_t face_dir;
} fake_point_t;

static fake_point_t g_fake_points[MAX_CAR_PATH]   = {{0}};
static size_t       g_fake_count                   = 0U;
static size_t       g_fake_cursor                  = 0U;
static exec_phase_t g_exec_phase                   = EXEC_PREPARE;
static size_t       g_exec_seg_start               = 0U;
static uint8        g_exec_rotate_started          = 0U;
static uint32       g_exec_pause_counter           = 0U;

/* ========================= 工具函数 ========================= */

static float clampf_local(float v, float min_v, float max_v)
{
    if (v < min_v) return min_v;
    if (v > max_v) return max_v;
    return v;
}

static float wrap_yaw_deg_local(float yaw_deg)
{
    while (yaw_deg > 180.0f)  yaw_deg -= 360.0f;
    while (yaw_deg < -180.0f) yaw_deg += 360.0f;
    return yaw_deg;
}

static float map_dir_to_yaw_deg(control_map_dir_t dir)
{
    float base_yaw  = g_map_right_yaw_ready ? g_map_right_yaw_deg : eulerAngle.yaw;
    float delta_yaw = 0.0f;

    switch (dir)
    {
    case CONTROL_MAP_DIR_RIGHT: delta_yaw = 0.0f;   break;
    case CONTROL_MAP_DIR_UP:    delta_yaw = 90.0f;  break;
    case CONTROL_MAP_DIR_LEFT:  delta_yaw = 180.0f; break;
    case CONTROL_MAP_DIR_DOWN:
    default:                    delta_yaw = -90.0f; break;
    }

    return wrap_yaw_deg_local(base_yaw + delta_yaw);
}

static control_map_dir_t get_prestart_depart_map_dir(void)
{
    switch (g_control_prestart_depart_dir)
    {
    case 1U: return CONTROL_MAP_DIR_UP;
    case 2U: return CONTROL_MAP_DIR_LEFT;
    case 3U: return CONTROL_MAP_DIR_DOWN;
    case 4U:
    case 0U:
    default: return CONTROL_MAP_DIR_RIGHT;
    }
}

static uint8 is_fake_identify_id(uint8 id)
{
    return (id == FAKE_IDENTIFY_FACE_RIGHT ||
            id == FAKE_IDENTIFY_FACE_UP    ||
            id == FAKE_IDENTIFY_FACE_LEFT  ||
            id == FAKE_IDENTIFY_FACE_DOWN) ? 1U : 0U;
}

static control_map_dir_t fake_id_to_dir(uint8 id)
{
    switch (id)
    {
    case FAKE_IDENTIFY_FACE_RIGHT: return CONTROL_MAP_DIR_RIGHT;
    case FAKE_IDENTIFY_FACE_UP:    return CONTROL_MAP_DIR_UP;
    case FAKE_IDENTIFY_FACE_LEFT:  return CONTROL_MAP_DIR_LEFT;
    case FAKE_IDENTIFY_FACE_DOWN:  return CONTROL_MAP_DIR_DOWN;
    default:                       return CONTROL_MAP_DIR_RIGHT;
    }
}

/*
 * 根据路径几何计算目标航向。
 * base_yaw = FAKE 点 → 下一个路径点 的 heading。
 * FACE_RIGHT = 朝行驶方向右侧看（base + 90°）
 * FACE_UP    = 朝行驶方向正前方看（base）
 * FACE_LEFT  = 朝行驶方向左侧看（base - 90°）
 * FACE_DOWN  = 朝行驶方向后方看（base + 180°）
 */
static float face_dir_to_target_yaw(control_map_dir_t dir, size_t fake_idx)
{
    float base_yaw;

    if (fake_idx + 1U < g_custom_path_steps)
    {
        Position from = g_custom_path[fake_idx];
        Position to   = g_custom_path[fake_idx + 1U];
        path_remap_exec_point(&from);
        path_remap_exec_point(&to);
        base_yaw = path_follow_heading_deg(from, to);
    }
    else
        base_yaw = g_map_right_yaw_ready ? g_map_right_yaw_deg : eulerAngle.yaw;

    switch (dir)
    {
    case CONTROL_MAP_DIR_UP:    return wrap_yaw_deg_local(base_yaw);
    case CONTROL_MAP_DIR_RIGHT: return wrap_yaw_deg_local(base_yaw + 90.0f);
    case CONTROL_MAP_DIR_DOWN:  return wrap_yaw_deg_local(base_yaw + 180.0f);
    case CONTROL_MAP_DIR_LEFT:  return wrap_yaw_deg_local(base_yaw - 90.0f);
    default:                    return wrap_yaw_deg_local(base_yaw);
    }
}

static void begin_path_plan_pause(void) { g_path_plan_paused = 1U; }
static void end_path_plan_pause(void)   { g_path_plan_paused = 0U; }

static void reset_control_runtime_state(void)
{
    g_control_start_enabled  = 0U;
    g_control_stage          = CONTROL_STAGE_IDLE;
    g_path_plan_paused       = 0U;
    g_plan_ready             = 0U;
    g_exec_steps             = 0U;
    memset(g_exec_path, 0, sizeof(g_exec_path));

    g_prestart_move_started  = 0U;
    g_map_right_yaw_deg      = 0.0f;
    g_map_right_yaw_ready    = 0U;
    g_fake_identify_started  = 0U;

    path_follow_set_pause_indices(NULL, 0U, 0U);
    path_follow_set_path(NULL, 0U);

    car_go_flag  = 0U;
    car_stop_flag = 0U;
}

/**
 * @brief 扫描 g_custom_path，收集所有 FAKE_IDENTIFY_FACE_* 点的索引和朝向。
 */
static void scan_fake_identify_points(void)
{
    size_t i;
    g_fake_count  = 0U;
    g_fake_cursor = 0U;
    memset(g_fake_points, 0, sizeof(g_fake_points));

    for (i = 0U; i < g_custom_path_steps; i++)
    {
        if (is_fake_identify_id(g_custom_path[i].id))
        {
            if (g_fake_count < MAX_CAR_PATH)
            {
                g_fake_points[g_fake_count].index    = i;
                g_fake_points[g_fake_count].face_dir =
                    fake_id_to_dir(g_custom_path[i].id);
                g_fake_count++;
            }
        }
    }
}

/**
 * @brief 将 g_custom_path 的一段拷贝到临时数组并下发给 path_follow。
 */
static void load_segment(size_t start_idx, size_t end_idx)
{
    Position seg[MAX_CAR_PATH];
    size_t seg_steps = end_idx - start_idx + 1U;
    size_t i;

    if (seg_steps > MAX_CAR_PATH)
        seg_steps = MAX_CAR_PATH;

    for (i = 0U; i < seg_steps; i++)
    {
        seg[i] = g_custom_path[start_idx + i];
        path_remap_exec_point(&seg[i]);
    }

    path_follow_hold_current_yaw();
    path_follow_set_path(seg, seg_steps);
}

/* ========================= Handler 函数 ========================= */

/**
 * @brief 起步发车阶段。
 *
 * 保持车头朝向，沿菜单选定的方向平移 CONTROL_PRESTART_OFFSET_M。
 * 完成后切到 FAKE_LOCALIZE。
 */
static void handle_prestart_move(void)
{
    path_follow_status_t st = {0};
    control_map_dir_t prestart_dir;
    float hold_yaw_deg, move_yaw_deg, move_yaw_rad;
    float start_x_m, start_y_m, delta_x_m, delta_y_m;
    float target_x_m, target_y_m;

    if (!g_prestart_move_started)
    {
        car_go_flag  = 1U;
        car_stop_flag = 0U;

        path_follow_get_status(&st);
        hold_yaw_deg  = eulerAngle.yaw;
        prestart_dir  = get_prestart_depart_map_dir();
        move_yaw_deg  = map_dir_to_yaw_deg(prestart_dir);
        move_yaw_rad  = move_yaw_deg * CONTROL_DEG_TO_RAD;
        start_x_m     = st.x_m;
        start_y_m     = st.y_m;
        delta_x_m     = cosf(move_yaw_rad) * CONTROL_PRESTART_OFFSET_M;
        delta_y_m     = sinf(move_yaw_rad) * CONTROL_PRESTART_OFFSET_M;

        target_x_m = clampf_local(start_x_m + delta_x_m,
                                  0.0f, CONTROL_WORLD_X_MAX_M);
        target_y_m = clampf_local(start_y_m + delta_y_m,
                                  0.0f, CONTROL_WORLD_Y_MAX_M);

        path_follow_reset_pose(st.x_m, st.y_m, hold_yaw_deg);
        path_follow_hold_current_yaw();
        path_follow_start_pose_correction(target_x_m, target_y_m);
        g_prestart_move_started = 1U;
        return;
    }

    path_follow_get_status(&st);
    if (!st.active)
    {
        car_go_flag  = 0U;
        car_stop_flag = 0U;
        g_control_stage = CONTROL_STAGE_FAKE_LOCALIZE;
    }
}

/**
 * @brief 伪定位阶段。
 *
 * 直接取 IMU 当前航向作为地图方向基准，里程计归零到起点，
 * 无需等待摄像头 CAR 帧。
 */
static void handle_fake_localization(void)
{
    if (!g_map_right_yaw_ready)
    {
        g_map_right_yaw_deg   = eulerAngle.yaw;
        g_map_right_yaw_ready = 1U;
    }

    path_follow_reset_pose(0.0f, 0.0f, g_map_right_yaw_deg);
    path_follow_hold_current_yaw();

    g_fake_identify_started = 0U;
    g_control_stage = CONTROL_STAGE_FAKE_IDENTIFY_PAUSE;
}

/**
 * @brief 伪识别阶段。
 *
 * 延时 CONTROL_FAKE_IDENTIFY_PAUSE_MS，假装在做摄像头识别。
 * 车保持静止，菜单可正常刷新。
 */
static void handle_fake_identify_pause(void)
{
    if (!g_fake_identify_started)
    {
        g_fake_identify_started = 1U;
        return;
    }

    system_delay_ms(CONTROL_FAKE_IDENTIFY_PAUSE_MS);
    g_control_stage = CONTROL_STAGE_LOAD_CUSTOM_PATH;
}

/**
 * @brief 装载自定义路径阶段。
 *
 * 扫描 g_custom_path 中的所有 FAKE_IDENTIFY_FACE_* 点，
 * 初始化分段执行状态：每段从一个 FAKE 点到下一个 FAKE 点（或路径终点）。
 */
static void handle_load_custom_path(void)
{
    if (g_custom_path_steps < 2U || g_custom_path_steps > MAX_CAR_PATH)
    {
        g_control_stage = CONTROL_STAGE_ERROR;
        return;
    }

    begin_path_plan_pause();

    /*
     * 保留 g_exec_path 供 control_get_exec_path() 返回，但实际执行
     * 不再一次下发整条路径，而是由 handle_execute_path() 分段下发。
     */
    memcpy(g_exec_path, g_custom_path, g_custom_path_steps * sizeof(Position));
    g_exec_steps = g_custom_path_steps;
    g_plan_ready = 1U;

    scan_fake_identify_points();

    g_exec_phase          = EXEC_PREPARE;
    g_exec_seg_start      = 0U;
    g_exec_rotate_started = 0U;
    g_exec_pause_counter  = 0U;

    end_path_plan_pause();
    g_control_stage = CONTROL_STAGE_EXECUTE_PATH;
}

/**
 * @brief 路径执行阶段（分段 + 定点转向）。
 *
 * EXEC_PREPARE  → 装载当前段路径，启动行驶
 * EXEC_MOVING   → 等待 path_follow 跑完当前段
 * EXEC_ROTATING → 到达 FAKE_IDENTIFY_FACE_* 点后，原地旋转面对目标
 * EXEC_PAUSING  → 旋转到位后停车约 2 秒，然后回到 PREPARE 装载下一段
 */
static void handle_execute_path(void)
{
    path_follow_status_t st = {0};
    path_follow_get_status(&st);

    switch (g_exec_phase)
    {
    case EXEC_PREPARE:
    {
        size_t seg_end;
        if (g_fake_cursor < g_fake_count)
            seg_end = g_fake_points[g_fake_cursor].index;
        else
            seg_end = g_custom_path_steps - 1U;

        load_segment(g_exec_seg_start, seg_end);
        car_go_flag  = 1U;
        car_stop_flag = 0U;
        g_exec_phase = EXEC_MOVING;
        break;
    }

    case EXEC_MOVING:
        if (st.active)
            return;

        /* 段行驶完成。若刚到达的是 FAKE 点则旋转 + 暂停。 */
        if (g_fake_cursor < g_fake_count)
        {
            g_fake_cursor++;
            g_exec_rotate_started = 0U;
            g_exec_phase = EXEC_ROTATING;
            return;
        }

        /* 没有更多 FAKE 点 → 完成 */
        car_go_flag  = 1U;
        car_stop_flag = 1U;
        g_control_stage = CONTROL_STAGE_FINISHED;
        break;

    case EXEC_ROTATING:
    {
        const fake_point_t *fp = &g_fake_points[g_fake_cursor - 1U];
        float target_yaw = face_dir_to_target_yaw(fp->face_dir, fp->index);

        if (!g_exec_rotate_started)
        {
            path_follow_start_rotate_to_yaw(target_yaw);
            car_go_flag  = 1U;
            car_stop_flag = 0U;
            g_exec_rotate_started = 1U;
            return;
        }

        if (!st.active)
        {
            g_exec_pause_counter = 0U;
            g_exec_phase = EXEC_PAUSING;
        }
        break;
    }

    case EXEC_PAUSING:
    {
        /*
         * 暂停循环计数近似 2 秒（假设主循环 ~250Hz）。
         * 若不准，调整 CONTROL_FAKE_IDENTIFY_PAUSE_LOOPS。
         */
        #define CONTROL_FAKE_IDENTIFY_PAUSE_LOOPS 500U

        car_go_flag  = 1U;
        car_stop_flag = 1U;
        g_exec_pause_counter++;
        if (g_exec_pause_counter >= CONTROL_FAKE_IDENTIFY_PAUSE_LOOPS)
        {
            g_exec_seg_start = g_fake_points[g_fake_cursor - 1U].index;
            g_exec_phase = EXEC_PREPARE;
        }
        break;
    }

    default:
        g_control_stage = CONTROL_STAGE_FINISHED;
        break;
    }
}

/**
 * @brief 错误阶段。
 *
 * 停车，延时后自动跳回 FAKE_LOCALIZE 重试。
 */
static void handle_error(void)
{
    car_go_flag  = 1U;
    car_stop_flag = 1U;

    path_follow_set_path(NULL, 0U);
    path_follow_set_pause_indices(NULL, 0U, 0U);

    system_delay_ms(CONTROL_ERROR_RETRY_DELAY_MS);

    g_prestart_move_started = 0U;
    g_map_right_yaw_ready   = 0U;
    g_fake_identify_started = 0U;
    g_plan_ready            = 0U;

    g_fake_count            = 0U;
    g_fake_cursor           = 0U;
    g_exec_phase            = EXEC_PREPARE;
    g_exec_seg_start        = 0U;
    g_exec_rotate_started   = 0U;
    g_exec_pause_counter    = 0U;

    g_control_stage = CONTROL_STAGE_FAKE_LOCALIZE;
}

/* ========================= 对外接口 ========================= */

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
            g_control_stage = CONTROL_STAGE_PRESTART_MOVE;
    }
    else
    {
        control_restart();
    }
}

uint8 control_get_start_enabled(void)
{
    return g_control_start_enabled;
}

void control_set_prestart_depart_dir(uint8 dir)
{
    if (dir > CONTROL_PRESTART_DEPART_DIR_MAX)
        dir = CONTROL_PRESTART_DEPART_DIR_MAX;
    g_control_prestart_depart_dir = dir;
}

uint8 control_get_prestart_depart_dir(void)
{
    return g_control_prestart_depart_dir;
}

void control_process(void)
{
    if (!g_control_start_enabled)
    {
        car_go_flag  = 0U;
        car_stop_flag = 0U;
        g_control_stage = CONTROL_STAGE_IDLE;
        return;
    }

    switch (g_control_stage)
    {
    case CONTROL_STAGE_PRESTART_MOVE:
        handle_prestart_move();
        break;
    case CONTROL_STAGE_FAKE_LOCALIZE:
        handle_fake_localization();
        break;
    case CONTROL_STAGE_FAKE_IDENTIFY_PAUSE:
        handle_fake_identify_pause();
        break;
    case CONTROL_STAGE_LOAD_CUSTOM_PATH:
        handle_load_custom_path();
        break;
    case CONTROL_STAGE_EXECUTE_PATH:
        handle_execute_path();
        break;
    case CONTROL_STAGE_FINISHED:
        break;
    case CONTROL_STAGE_ERROR:
        handle_error();
        break;
    case CONTROL_STAGE_IDLE:
    default:
        break;
    }
}

control_stage_t control_get_stage(void)
{
    return g_control_stage;
}

const Position *control_get_exec_path(size_t *steps)
{
    if (steps != NULL) *steps = g_exec_steps;
    return g_exec_path;
}

void control_set_diagonal_path_enabled(uint8 enabled)
{
    g_diagonal_enabled = (enabled != 0U) ? 1U : 0U;
    path_set_diagonal_enabled(enabled);
}

uint8 control_get_diagonal_path_enabled(void)
{
    return g_diagonal_enabled;
}

uint8 control_is_path_plan_paused(void)
{
    return g_path_plan_paused;
}
