#include "Control.h"
#include "Game_logic.h"
#include "Mymenu.h"
#include "Attitude.h"
#include <math.h>
#include <string.h>

/* ========================= 参数配置�?========================= */

/**
 * @brief 地图请求发送周期（单位：control_process 调用次数）�?
 *
 * 含义：每调用 CONTROL_REQ_MAP_PERIOD_LOOPS �?control_process�?
 * 发送一�?"MAP" 请求，避免每圈都发导致串口拥塞�?
 */
#define CONTROL_REQ_MAP_PERIOD_LOOPS 40U

/**
 * @brief 非执行阶段车位姿请求发送周期（单位：control_process 调用次数）�?
 *
 * 用于初始定位和等待地图阶段，采样频率较低即可满足需求�?
 */
#define CONTROL_REQ_CAR_PERIOD_WAIT 20U
#define CONTROL_REQ_CAR_PERIOD_LOCALIZE_FAST 8U


/**
 * @brief 初始定位最少采样帧数�?
 *
 * 使用简单平均抑制单帧抖动。达到该采样数后才完成初始定位�?
 */
#define CONTROL_LOCALIZE_MIN_SAMPLES 2U
#define CONTROL_RELOCALIZE_MIN_SAMPLES_PUSHBOX 4U

/**
 * @brief 起步位移距离（米）�?
 *
 * 这里采用与地图网格一致的 `GRID_SIZE_M`，即 0.2m�?
 */
#define CONTROL_PRESTART_OFFSET_M GRID_SIZE_M

/**
 * @brief 起步方向系数�?
 *
 * - 1.0f：沿车头方向前进
 * - -1.0f：沿车头反方向后退
 */
#define CONTROL_PRESTART_DIR_SIGN 1.0f
/* 角度转弧度：yaw_rad = yaw_deg * CONTROL_DEG_TO_RAD */
#define CONTROL_DEG_TO_RAD 0.01745329251994329577f

/**
 * @brief 坐标轴补偿开关�?
 *
 * 现象：规划“向右”实际“向下”，规划“向上”实际“向左”，
 * 对应规划/执行坐标发生了行列转置。开启该开关后�?
 * - 路径�?(row,col) 在下发前转为 (col,row)
 * - 视觉位姿映射同步采用 (x=col, y=row)
 */
#define CONTROL_COORD_TRANSPOSE_COMPENSATE 1U
/* 在转置补偿基础上，额外翻转“上下轴”（对应 map �?row 方向）�?*/
#define CONTROL_COORD_FLIP_VERTICAL 1U

#if CONTROL_COORD_TRANSPOSE_COMPENSATE
#define CONTROL_WORLD_X_MAX_M ((float)(MAP_COLS - 1) * GRID_SIZE_M)
#define CONTROL_WORLD_Y_MAX_M ((float)(MAP_ROWS - 1) * GRID_SIZE_M)
#else
#define CONTROL_WORLD_X_MAX_M ((float)(MAP_ROWS - 1) * GRID_SIZE_M)
#define CONTROL_WORLD_Y_MAX_M ((float)(MAP_COLS - 1) * GRID_SIZE_M)
#endif


/* ========================= 内部数据结构 ========================= */

/**
 * @brief 地图运行时快照�?
 *
 * Game_logic 规划函数会在内部模拟执行并改写全局地图对象�?
 * 为了避免影响实时显示与后续逻辑，规划前先快照，规划后再恢复�?
 */
typedef struct
{
    size_t obstacles_count;                    /**< 快照时障碍物数量�?*/
    size_t boxes_count;                        /**< 快照时箱子数量�?*/
    size_t targets_count;                      /**< 快照时目标点数量�?*/
    size_t bombs_count;                        /**< 快照时炸弹数量�?*/
    Position obstacles_buf[MAX_OBSTACLES];     /**< 快照时障碍物数组�?*/
    Position boxes_buf[MAX_BOXES];             /**< 快照时箱子数组�?*/
    Position targets_buf[MAX_TARGETS];         /**< 快照时目标点数组�?*/
    Position bombs_buf[MAX_BOMBS];             /**< 快照时炸弹数组�?*/
    Position car_pose_grid;                    /**< 快照时小车栅格位置�?*/
} map_runtime_snapshot_t;

#define CONTROL_IDENTIFY_MAX_TARGETS_PER_POINT (MAX_BOXES + MAX_TARGETS)
#define CONTROL_IDENTIFY_YAW_ALIGN_TOL_DEG 5.0f

typedef enum
{
    CONTROL_FLOW_IDENTIFY = 0U,
    CONTROL_FLOW_PUSHBOX = 1U
} control_flow_phase_t;

typedef enum
{
    CONTROL_MAP_DIR_RIGHT = 0U,
    CONTROL_MAP_DIR_UP = 1U,
    CONTROL_MAP_DIR_LEFT = 2U,
    CONTROL_MAP_DIR_DOWN = 3U
} control_map_dir_t;

typedef enum
{
    CONTROL_IDENTIFY_OBJ_BOX = 0U,
    CONTROL_IDENTIFY_OBJ_TARGET = 1U
} control_identify_obj_t;

typedef struct
{
    Position obj_pos_map;
    control_map_dir_t face_dir;
    control_identify_obj_t obj_type;
    uint8 obj_index;
} control_identify_target_t;

typedef enum
{
    CONTROL_IDENTIFY_EXEC_IDLE = 0U,
    CONTROL_IDENTIFY_EXEC_MOVE_SEGMENT,
    CONTROL_IDENTIFY_EXEC_ROTATE_TO_TARGET,
    CONTROL_IDENTIFY_EXEC_DO_RECOGNIZE,
    CONTROL_IDENTIFY_EXEC_ROTATE_BACK
} control_identify_exec_state_t;

/* ========================= 内部状态变�?========================= */

/**
 * @brief 控制状态机当前阶段�?
 */
control_stage_t g_control_stage = CONTROL_STAGE_IDLE;

/**
 * @brief 路径规划保护期标志�?
 *
 * - 1：处于保护期（规�?切换路径中）
 * - 0：正常执�?
 *
 * 该变量在主循环和中断中都会被读取，因此使�?volatile�?
 */
static volatile uint8 g_path_plan_paused = 0U;

/**
 * @brief 执行层路径缓存（最终下发给 path_follow）�?
 */
static Position g_exec_path[MAX_CAR_PATH] = {{0}};

/**
 * @brief 执行层路径有效点数�?
 */
static size_t g_exec_steps = 0U;

/**
 * @brief 当前是否已经得到可执行路径�?
 *
 * - 1：已规划并生成执行路�?
 * - 0：尚未得到有效路�?
 */
static uint8 g_plan_ready = 0U;
/* 规划模式标志位：可通过 control_set_plan_mode() �?Mode1/Mode2/Identify 间切换�?*/
static control_plan_mode_t g_control_plan_mode = CONTROL_PLAN_MODE_2;
static control_flow_phase_t g_control_flow_phase = CONTROL_FLOW_IDENTIFY;

static float g_map_right_yaw_deg = 0.0f;
static uint8 g_map_right_yaw_ready = 0U;
static control_map_dir_t g_body_map_dir = CONTROL_MAP_DIR_RIGHT;

static uint8 g_wait_new_map_frame = 0U;
static uint8 g_wait_map_frame_base = 0U;
static uint8 g_relocalize_force_fresh_pose = 0U;

static size_t g_identify_segment_start_idx = 0U;
static size_t g_identify_endpoint_indices[MAX_CAR_PATH] = {0U};
static uint8 g_identify_endpoint_need_action[MAX_CAR_PATH] = {0U};
static size_t g_identify_endpoint_count = 0U;
static size_t g_identify_endpoint_cursor = 0U;
static Position g_identify_segment_path[MAX_CAR_PATH] = {{0}};
static uint8 g_identify_segment_running = 0U;

static control_identify_target_t g_identify_targets[CONTROL_IDENTIFY_MAX_TARGETS_PER_POINT];
static size_t g_identify_target_count = 0U;
static size_t g_identify_target_cursor = 0U;
static uint8 g_identify_rotate_started = 0U;
static control_identify_exec_state_t g_identify_exec_state = CONTROL_IDENTIFY_EXEC_IDLE;

static uint8 g_identified_box_flags[MAX_BOXES] = {0U};
static uint8 g_identified_target_flags[MAX_TARGETS] = {0U};

/**
 * @brief 初始定位累计样本数�?
 */
static uint8 g_localize_sample_count = 0U;

/**
 * @brief 起步动作是否已经触发�?
 *
 * - 0：尚未下发起步动�?
 * - 1：已下发，等待动作执行结�?
 */
static uint8 g_prestart_move_started = 0U;

/**
 * @brief 初始定位阶段累计的视�?X 坐标和（米）�?
 */
static float g_localize_sum_x_m = 0.0f;

/**
 * @brief 初始定位阶段累计的视�?Y 坐标和（米）�?
 */
static float g_localize_sum_y_m = 0.0f;

/**
 * @brief 初始定位阶段累计的视觉航向角和（度）�?
 */
static float g_localize_sum_yaw_deg = 0.0f;

/**
 * @brief 地图请求分频计数器�?
 */
static uint16 g_map_req_loop_cnt = 0U;

/**
 * @brief 车位姿请求分频计数器�?
 */
static uint16 g_car_req_loop_cnt = 0U;

/* ========================= 内部工具函数 ========================= */

/**
 * @brief 本地限幅函数�?
 *
 * @param v 输入值�?
 * @param min_v 下限�?
 * @param max_v 上限�?
 * @return float 限幅后的值�?
 */
static float clampf_local(float v, float min_v, float max_v)
{
    if (v < min_v)
    {
        return min_v;
    }
    if (v > max_v)
    {
        return max_v;
    }
    return v;
}

static float wrap_yaw_deg_local(float yaw_deg)
{
    while (yaw_deg > 180.0f)
    {
        yaw_deg -= 360.0f;
    }
    while (yaw_deg < -180.0f)
    {
        yaw_deg += 360.0f;
    }
    return yaw_deg;
}

static void reset_localization_accumulator(void)
{
    g_localize_sample_count = 0U;
    g_localize_sum_x_m = 0.0f;
    g_localize_sum_y_m = 0.0f;
    g_localize_sum_yaw_deg = 0.0f;
}

static void mark_wait_new_map_frame(void)
{
    g_wait_new_map_frame = 1U;
    g_wait_map_frame_base = map_frame_count;
}

static void reset_identify_runtime_state(void)
{
    g_identify_segment_start_idx = 0U;
    memset(g_identify_endpoint_indices, 0, sizeof(g_identify_endpoint_indices));
    memset(g_identify_endpoint_need_action, 0, sizeof(g_identify_endpoint_need_action));
    g_identify_endpoint_count = 0U;
    g_identify_endpoint_cursor = 0U;
    memset(g_identify_segment_path, 0, sizeof(g_identify_segment_path));
    g_identify_segment_running = 0U;

    memset(g_identify_targets, 0, sizeof(g_identify_targets));
    g_identify_target_count = 0U;
    g_identify_target_cursor = 0U;
    g_identify_rotate_started = 0U;
    g_identify_exec_state = CONTROL_IDENTIFY_EXEC_IDLE;

    memset(g_identified_box_flags, 0, sizeof(g_identified_box_flags));
    memset(g_identified_target_flags, 0, sizeof(g_identified_target_flags));
}

static void inverse_remap_exec_path_point(Position *p)
{
    uint8 temp = 0U;

    if (p == NULL)
    {
        return;
    }

#if CONTROL_COORD_FLIP_VERTICAL
    p->col = (uint8)((MAP_ROWS - 1U) - p->col);
#endif

#if CONTROL_COORD_TRANSPOSE_COMPENSATE
    temp = p->row;
    p->row = p->col;
    p->col = temp;
#endif
}

static uint8 resolve_map_dir_from_delta(int32 d_row, int32 d_col, control_map_dir_t *dir_out)
{
    if (dir_out == NULL)
    {
        return 0U;
    }

    if (d_row == 0 && d_col == 1)
    {
        *dir_out = CONTROL_MAP_DIR_RIGHT;
        return 1U;
    }
    if (d_row == 0 && d_col == -1)
    {
        *dir_out = CONTROL_MAP_DIR_LEFT;
        return 1U;
    }
    if (d_row == -1 && d_col == 0)
    {
        *dir_out = CONTROL_MAP_DIR_UP;
        return 1U;
    }
    if (d_row == 1 && d_col == 0)
    {
        *dir_out = CONTROL_MAP_DIR_DOWN;
        return 1U;
    }
    return 0U;
}

static float map_dir_to_yaw_deg(control_map_dir_t dir)
{
    float base_yaw = 0.0f;
    float delta_yaw = 0.0f;

    if (g_map_right_yaw_ready)
    {
        base_yaw = g_map_right_yaw_deg;
    }
    else
    {
        base_yaw = eulerAngle.yaw;
    }

    switch (dir)
    {
    case CONTROL_MAP_DIR_RIGHT:
        delta_yaw = 0.0f;
        break;
    case CONTROL_MAP_DIR_UP:
        delta_yaw = 90.0f;
        break;
    case CONTROL_MAP_DIR_LEFT:
        delta_yaw = 180.0f;
        break;
    case CONTROL_MAP_DIR_DOWN:
    default:
        delta_yaw = -90.0f;
        break;
    }

    return wrap_yaw_deg_local(base_yaw + delta_yaw);
}

static void identify_action_stub(const control_identify_target_t *target)
{
    (void)target;
    /* TODO: 鍦ㄨ繖閲屾帴鍏ュ叿浣撶殑璇嗗埆绠楁硶�?*/
}

static void identify_sort_targets_by_direction(void)
{
    size_t i = 0U;
    size_t j = 0U;

    for (i = 0U; i < g_identify_target_count; i++)
    {
        for (j = i + 1U; j < g_identify_target_count; j++)
        {
            if ((uint8)g_identify_targets[j].face_dir < (uint8)g_identify_targets[i].face_dir)
            {
                control_identify_target_t tmp = g_identify_targets[i];
                g_identify_targets[i] = g_identify_targets[j];
                g_identify_targets[j] = tmp;
            }
        }
    }
}

static uint8 collect_identify_targets_on_exec_point(const Position *exec_point)
{
    Position map_point = {0};
    size_t i = 0U;
    int32 d_row = 0;
    int32 d_col = 0;
    int32 manhattan = 0;
    control_map_dir_t face_dir = CONTROL_MAP_DIR_RIGHT;

    if (exec_point == NULL)
    {
        return 0U;
    }

    memset(g_identify_targets, 0, sizeof(g_identify_targets));
    g_identify_target_count = 0U;
    g_identify_target_cursor = 0U;

    map_point = *exec_point;
    inverse_remap_exec_path_point(&map_point);

    for (i = 0U; i < Boxes_count; i++)
    {
        if (g_identified_box_flags[i])
        {
            continue;
        }

        d_row = (int32)boxes[i].row - (int32)map_point.row;
        d_col = (int32)boxes[i].col - (int32)map_point.col;
        manhattan = ((d_row >= 0) ? d_row : -d_row) + ((d_col >= 0) ? d_col : -d_col);
        if (manhattan != 1)
        {
            continue;
        }
        if (!resolve_map_dir_from_delta(d_row, d_col, &face_dir))
        {
            continue;
        }
        if (g_identify_target_count >= CONTROL_IDENTIFY_MAX_TARGETS_PER_POINT)
        {
            break;
        }

        g_identify_targets[g_identify_target_count].obj_pos_map = boxes[i];
        g_identify_targets[g_identify_target_count].face_dir = face_dir;
        g_identify_targets[g_identify_target_count].obj_type = CONTROL_IDENTIFY_OBJ_BOX;
        g_identify_targets[g_identify_target_count].obj_index = (uint8)i;
        g_identify_target_count++;
    }

    for (i = 0U; i < Targets_count; i++)
    {
        if (g_identified_target_flags[i])
        {
            continue;
        }

        d_row = (int32)targets[i].row - (int32)map_point.row;
        d_col = (int32)targets[i].col - (int32)map_point.col;
        manhattan = ((d_row >= 0) ? d_row : -d_row) + ((d_col >= 0) ? d_col : -d_col);
        if (manhattan != 1)
        {
            continue;
        }
        if (!resolve_map_dir_from_delta(d_row, d_col, &face_dir))
        {
            continue;
        }
        if (g_identify_target_count >= CONTROL_IDENTIFY_MAX_TARGETS_PER_POINT)
        {
            break;
        }

        g_identify_targets[g_identify_target_count].obj_pos_map = targets[i];
        g_identify_targets[g_identify_target_count].face_dir = face_dir;
        g_identify_targets[g_identify_target_count].obj_type = CONTROL_IDENTIFY_OBJ_TARGET;
        g_identify_targets[g_identify_target_count].obj_index = (uint8)i;
        g_identify_target_count++;
    }

    identify_sort_targets_by_direction();
    return (g_identify_target_count > 0U) ? 1U : 0U;
}

static uint8 prepare_identify_endpoints(void)
{
    size_t i = 0U;

    g_identify_endpoint_count = 0U;
    g_identify_endpoint_cursor = 0U;
    g_identify_segment_start_idx = 0U;
    g_identify_exec_state = CONTROL_IDENTIFY_EXEC_MOVE_SEGMENT;
    g_identify_rotate_started = 0U;
    g_identify_segment_running = 0U;

    if (g_exec_steps < 2U)
    {
        return 0U;
    }

    for (i = 0U; i < g_exec_steps; i++)
    {
        if (g_exec_path[i].id == IDENTIFICATION)
        {
            if (g_identify_endpoint_count >= MAX_CAR_PATH)
            {
                return 0U;
            }
            g_identify_endpoint_indices[g_identify_endpoint_count] = i;
            g_identify_endpoint_need_action[g_identify_endpoint_count] = 1U;
            g_identify_endpoint_count++;
        }
    }

    if (g_identify_endpoint_count == 0U ||
        g_identify_endpoint_indices[g_identify_endpoint_count - 1U] != (g_exec_steps - 1U))
    {
        if (g_identify_endpoint_count >= MAX_CAR_PATH)
        {
            return 0U;
        }
        g_identify_endpoint_indices[g_identify_endpoint_count] = g_exec_steps - 1U;
        g_identify_endpoint_need_action[g_identify_endpoint_count] = 0U;
        g_identify_endpoint_count++;
    }

    return (g_identify_endpoint_count > 0U) ? 1U : 0U;
}

static uint8 start_identify_segment(size_t end_idx)
{
    size_t i = 0U;
    size_t seg_steps = 0U;

    if (end_idx >= g_exec_steps || end_idx < g_identify_segment_start_idx)
    {
        return 0U;
    }

    seg_steps = end_idx - g_identify_segment_start_idx + 1U;
    g_identify_segment_running = 0U;

    if (seg_steps >= 2U)
    {
        if (seg_steps > MAX_CAR_PATH)
        {
            return 0U;
        }

        for (i = 0U; i < seg_steps; i++)
        {
            g_identify_segment_path[i] = g_exec_path[g_identify_segment_start_idx + i];
        }

        path_follow_set_pause_indices(NULL, 0U, 0U);
        path_follow_hold_current_yaw();
        path_follow_set_path(g_identify_segment_path, seg_steps);
        g_identify_segment_running = 1U;
    }

    car_go_flag = 1U;
    car_stop_flag = 0U;
    g_identify_exec_state = CONTROL_IDENTIFY_EXEC_MOVE_SEGMENT;
    return 1U;
}

static void finish_identify_flow_and_relocalize(void)
{
    car_go_flag = 0U;
    car_stop_flag = 0U;
    path_follow_set_path(NULL, 0U);
    path_follow_set_pause_indices(NULL, 0U, 0U);

    g_control_flow_phase = CONTROL_FLOW_PUSHBOX;
    g_plan_ready = 0U;
    reset_identify_runtime_state();
    reset_localization_accumulator();
    g_map_req_loop_cnt = 0U;
    g_car_req_loop_cnt = 0U;
    g_wait_new_map_frame = 0U;
    g_wait_map_frame_base = 0U;
    g_relocalize_force_fresh_pose = 1U;
    map_data_updated = false;
    car_pose_updated = false;
    g_control_stage = CONTROL_STAGE_STARTUP_LOCALIZE;
}

static uint8 advance_identify_endpoint_or_finish(void)
{
    size_t end_idx = 0U;

    if (g_identify_endpoint_cursor >= g_identify_endpoint_count)
    {
        return 0U;
    }

    end_idx = g_identify_endpoint_indices[g_identify_endpoint_cursor];
    g_identify_segment_start_idx = end_idx;
    g_identify_endpoint_cursor++;

    if (g_identify_endpoint_cursor >= g_identify_endpoint_count)
    {
        finish_identify_flow_and_relocalize();
        return 1U;
    }

    return start_identify_segment(g_identify_endpoint_indices[g_identify_endpoint_cursor]);
}

/**
 * @brief 将规划输出的路径点转换为执行坐标系�?
 *
 * 当开启行列补偿时，把 (row,col) 转为 (col,row)�?
 * 关闭补偿时保持原样�?
 */
static void remap_exec_path_point(Position *p)
{
    uint8 temp = 0U;

    if (p == NULL)
    {
        return;
    }

#if CONTROL_COORD_TRANSPOSE_COMPENSATE
    temp = p->row;
    p->row = p->col;
    p->col = temp;
#endif

#if CONTROL_COORD_FLIP_VERTICAL
    p->col = (uint8)((MAP_ROWS - 1) - p->col);
#endif
}

/**
 * @brief 判断两个栅格点是否在同一单元�?
 */
static uint8 is_same_grid_cell(const Position *a, const Position *b)
{
    if (a == NULL || b == NULL)
    {
        return 0U;
    }
    return (a->row == b->row && a->col == b->col) ? 1U : 0U;
}

/**
 * @brief 判断三点是否“同一直线且同向前进”�?
 *
 * 仅当 A->B �?B->C 共线且方向不反向（点�?>= 0）时返回 1�?
 * 这样可压缩直线段，同时避免把“原路折返”的拐点误删�?
 */
static uint8 is_collinear_forward(const Position *a,
                                  const Position *b,
                                  const Position *c)
{
    int32 v1_r = 0;
    int32 v1_c = 0;
    int32 v2_r = 0;
    int32 v2_c = 0;
    int32 cross = 0;
    int32 dot = 0;

    if (a == NULL || b == NULL || c == NULL)
    {
        return 0U;
    }

    v1_r = (int32)b->row - (int32)a->row;
    v1_c = (int32)b->col - (int32)a->col;
    v2_r = (int32)c->row - (int32)b->row;
    v2_c = (int32)c->col - (int32)b->col;
    cross = v1_r * v2_c - v1_c * v2_r;
    dot = v1_r * v2_r + v1_c * v2_c;

    return (cross == 0 && dot >= 0) ? 1U : 0U;
}

/**
 * @brief 将视觉位姿转换为 path_follow 需要的米制坐标�?
 *
 * 当前工程坐标约定�?
 * - data_handle �?`car_pose.x/y` 为“列/行”浮点栅格�?
 * - path_follow 使用 x 对应行，y 对应列，且单位为�?
 * - 因此转换为：x_m = row * GRID_SIZE_M, y_m = col * GRID_SIZE_M
 *
 * @param[out] x_m 转换后的 x 坐标（米）�?
 * @param[out] y_m 转换后的 y 坐标（米）�?
 * @param[out] yaw_deg 视觉航向角（度）�?
 * @return uint8
 * - 1：转换成�?
 * - 0：输入参数无效或视觉位姿尚未就绪
 */
static uint8 get_camera_pose_meter(float *x_m, float *y_m, float *yaw_deg)
{
    float row_f = 0.0f;
    float col_f = 0.0f;

    if (x_m == NULL || y_m == NULL || yaw_deg == NULL)
    {
        return 0U;
    }
    if (!car_pose_ready)
    {
        return 0U;
    }

/* 视觉位姿到执行坐标系的映射需与路径点 remap 保持一致�?*/
#if CONTROL_COORD_TRANSPOSE_COMPENSATE
    row_f = car_pose.x;
    col_f = car_pose.y;
#else
    row_f = car_pose.y;
    col_f = car_pose.x;
#endif

#if CONTROL_COORD_FLIP_VERTICAL
    col_f = (float)(MAP_ROWS - 1) - col_f;
#endif

    *x_m = row_f * GRID_SIZE_M;
    *y_m = col_f * GRID_SIZE_M;
    *yaw_deg = car_pose.yaw;
    return 1U;
}

/**
 * @brief 按固定节拍发送地图请求命令�?
 */
static void request_map_periodic(void)
{
    g_map_req_loop_cnt++;
    if (g_map_req_loop_cnt >= CONTROL_REQ_MAP_PERIOD_LOOPS)
    {
        uart_send_map_request();
        g_map_req_loop_cnt = 0U;
    }
}

/**
 * @brief 按固定节拍发送车位姿请求命令�?
 *
 * @param period_loops 请求周期（单位：control_process 调用次数）�?
 */
static void request_car_periodic(uint16 period_loops)
{
    g_car_req_loop_cnt++;
    if (g_car_req_loop_cnt >= period_loops)
    {
        uart_send_car_request();
        g_car_req_loop_cnt = 0U;
    }
}

/**
 * @brief 进入路径规划保护期�?
 */
static void begin_path_plan_pause(void)
{
    g_path_plan_paused = 1U;
}

/**
 * @brief 退出路径规划保护期�?
 */
static void end_path_plan_pause(void)
{
    g_path_plan_paused = 0U;
}

/**
 * @brief 采集当前全局地图状态快照�?
 *
 * @param[out] snap 快照输出结构体指针�?
 */
static void snapshot_take(map_runtime_snapshot_t *snap)
{
    if (snap == NULL)
    {
        return;
    }

    snap->obstacles_count = Obstacles_count;
    snap->boxes_count = Boxes_count;
    snap->targets_count = Targets_count;
    snap->bombs_count = Bombs_count;
    snap->car_pose_grid = car;

    memset(snap->obstacles_buf, 0, sizeof(snap->obstacles_buf));
    memset(snap->boxes_buf, 0, sizeof(snap->boxes_buf));
    memset(snap->targets_buf, 0, sizeof(snap->targets_buf));
    memset(snap->bombs_buf, 0, sizeof(snap->bombs_buf));

    if (snap->obstacles_count > 0U)
    {
        memcpy(snap->obstacles_buf,
               obstacles,
               snap->obstacles_count * sizeof(Position));
    }
    if (snap->boxes_count > 0U)
    {
        memcpy(snap->boxes_buf,
               boxes,
               snap->boxes_count * sizeof(Position));
    }
    if (snap->targets_count > 0U)
    {
        memcpy(snap->targets_buf,
               targets,
               snap->targets_count * sizeof(Position));
    }
    if (snap->bombs_count > 0U)
    {
        memcpy(snap->bombs_buf,
               bombs,
               snap->bombs_count * sizeof(Position));
    }
}

/**
 * @brief 恢复全局地图状态到快照时刻�?
 *
 * 注意�?
 * - 只恢复地图对象（障碍/箱子/目标/炸弹/车）
 * - 不恢�?car_path / Car_path_count，保留最新规划结果给执行层使�?
 *
 * @param[in] snap 快照输入结构体指针�?
 */
static void snapshot_restore(const map_runtime_snapshot_t *snap)
{
    if (snap == NULL)
    {
        return;
    }

    Obstacles_count = snap->obstacles_count;
    Boxes_count = snap->boxes_count;
    Targets_count = snap->targets_count;
    Bombs_count = snap->bombs_count;
    car = snap->car_pose_grid;

    memset(obstacles, 0, sizeof(obstacles));
    memset(boxes, 0, sizeof(boxes));
    memset(targets, 0, sizeof(targets));
    memset(bombs, 0, sizeof(bombs));

    if (Obstacles_count > 0U)
    {
        memcpy(obstacles,
               snap->obstacles_buf,
               Obstacles_count * sizeof(Position));
    }
    if (Boxes_count > 0U)
    {
        memcpy(boxes,
               snap->boxes_buf,
               Boxes_count * sizeof(Position));
    }
    if (Targets_count > 0U)
    {
        memcpy(targets,
               snap->targets_buf,
               Targets_count * sizeof(Position));
    }
    if (Bombs_count > 0U)
    {
        memcpy(bombs,
               snap->bombs_buf,
               Bombs_count * sizeof(Position));
    }

    //
    // boxes[0].id = 0;
    // boxes[1].id = 1;
    // boxes[2].id = 2;
    // targets[0].id = 0;
    // targets[1].id = 1;
    // targets[2].id = 2;
    //

}

/**
 * @brief 从规划路径中构建执行层路径�?
 *
 * 处理策略�?
 * - 优先提取拐点路径（降低执行点数，便于跟踪�?
 * - 若拐点提取失败，则回退使用完整规划路径
 *
 * @return uint8
 * - 1：成功生成执行路�?
 * - 0：规划路径无�?
 */
static uint8 build_exec_path_from_planner(void)
{
    size_t i = 0U;
    size_t out_steps = 0U;
    Position mapped = {0};

    g_exec_steps = 0U;
    memset(g_exec_path, 0, sizeof(g_exec_path));

    if (Car_path_count < 2U || Car_path_count > MAX_CAR_PATH)
    {
        return 0U;
    }

    /* 路径下发前统一做：坐标映射 + 重复点剔�?+ 直线段压缩�?*/
    for (i = 0U; i < Car_path_count; i++)
    {
        mapped = car_path[i];
        remap_exec_path_point(&mapped);

        if (out_steps == 0U)
        {
            g_exec_path[out_steps++] = mapped;
            continue;
        }

        if (is_same_grid_cell(&g_exec_path[out_steps - 1U], &mapped))
        {
            /* 同格重复点：若新点带事件 id，覆盖保留�?*/
            if (mapped.id != 0U)
            {
                g_exec_path[out_steps - 1U].id = mapped.id;
            }
            continue;
        }

        if (out_steps >= 2U &&
            g_exec_path[out_steps - 1U].id == 0U &&
            mapped.id == 0U &&
            is_collinear_forward(&g_exec_path[out_steps - 2U],
                                 &g_exec_path[out_steps - 1U],
                                 &mapped))
        {
            /* 直线同向延长：用新终点替换旧终点，实现“一段到底”�?*/
            g_exec_path[out_steps - 1U] = mapped;
            continue;
        }

        if (out_steps >= MAX_CAR_PATH)
        {
            return 0U;
        }
        g_exec_path[out_steps++] = mapped;
    }

    if (out_steps < 2U)
    {
        return 0U;
    }

    g_exec_steps = out_steps;
    return 1U;
}

/**
 * @brief 完成一次路径规划并构建执行路径�?
 *
 * 规划策略�?
 * - 先尝�?Mode2（按 ID 配对�?
 * - 若失败，回退�?Mode1（全局贪心�?
 *
 * @return uint8
 * - 1：规划成功且可执行路径已准备�?
 * - 0：规划失�?
 */
static uint8 control_plan_path(void)
{
    map_runtime_snapshot_t map_snapshot;

    g_plan_ready = 0U;
    memset(&map_snapshot, 0, sizeof(map_snapshot));
    snapshot_take(&map_snapshot);

    /* 根据 planmode 标志位选择规划方案，不做自动模式回退�?*/
    if (g_control_flow_phase == CONTROL_FLOW_IDENTIFY)
    {
        reset_identify_runtime_state();
        Plan_path_Identify();
    }
    else if (g_control_plan_mode == CONTROL_PLAN_MODE_1)
    {
        Plan_path_Mode1();
    }
    else
    {
        Plan_path_Mode2();
    }

    if (Car_path_count < 2U)
    {
        snapshot_restore(&map_snapshot);
        return 0U;
    }

    if (!build_exec_path_from_planner())
    {
        snapshot_restore(&map_snapshot);
        return 0U;
    }

    snapshot_restore(&map_snapshot);
    g_plan_ready = 1U;
    return 1U;
}

/**
 * @brief 处理“起步出发车区”阶段�?
 *
 * 行为说明�?
 * - 首次进入时，读取当前 IMU 航向角作为车头方�?
 * - 按该航向分解出世界坐标位移，下发一�?0.2m 偏移动作
 * - 动作执行完成后，切到初始定位阶段
 */
static void handle_prestart_move(void)
{
    path_follow_status_t st = {0};
    float prestart_yaw_deg = 0.0f;
    float prestart_yaw_rad = 0.0f;
    float start_x_m = 0.0f;
    float start_y_m = 0.0f;
    float delta_x_m = 0.0f;
    float delta_y_m = 0.0f;
    float target_x_m = 0.0f;
    float target_y_m = 0.0f;

    if (!g_prestart_move_started)
    {
        /* 执行起步动作前放开底盘控制输出�?*/
        car_go_flag = 1U;
        car_stop_flag = 0U;

        path_follow_get_status(&st);
        prestart_yaw_deg = eulerAngle.yaw;
        prestart_yaw_rad = prestart_yaw_deg * CONTROL_DEG_TO_RAD;
        start_x_m = st.x_m;
        start_y_m = st.y_m;
        delta_x_m = CONTROL_PRESTART_DIR_SIGN * cosf(prestart_yaw_rad) * CONTROL_PRESTART_OFFSET_M;
        delta_y_m = CONTROL_PRESTART_DIR_SIGN * sinf(prestart_yaw_rad) * CONTROL_PRESTART_OFFSET_M;

        /* 防止起步目标越界（负坐标�?uint8 会变 255，导致远距离猛冲）�?*/
        target_x_m = clampf_local(start_x_m + delta_x_m, 0.0f, CONTROL_WORLD_X_MAX_M);
        target_y_m = clampf_local(start_y_m + delta_y_m, 0.0f, CONTROL_WORLD_Y_MAX_M);

        path_follow_reset_pose(st.x_m, st.y_m, prestart_yaw_deg);
        path_follow_hold_current_yaw();
        path_follow_start_pose_correction(target_x_m, target_y_m);
        g_prestart_move_started = 1U;
        return;
    }

    path_follow_get_status(&st);
    if (!st.active)
    {
        /* 起步动作结束后先回到静止，再进入初始定位阶段�?*/
        car_go_flag = 0U;
        car_stop_flag = 0U;
        g_control_stage = CONTROL_STAGE_STARTUP_LOCALIZE;
    }
}

/**
 * @brief 处理“初始定位”阶段�?
 *
 * 核心逻辑�?
 * - 周期请求视觉车位�?
 * - 采集多帧后取平均
 * - 一次性重置里程计位姿到平均�?
 */
static void handle_startup_localization(void)
{
    uint8 accept_sample = 0U;
    uint8 min_samples = CONTROL_LOCALIZE_MIN_SAMPLES;
    float cam_x_m = 0.0f;
    float cam_y_m = 0.0f;
    float cam_yaw_deg = 0.0f;

    if (g_control_flow_phase == CONTROL_FLOW_PUSHBOX)
    {
        request_car_periodic(CONTROL_REQ_CAR_PERIOD_LOCALIZE_FAST);
        min_samples = CONTROL_RELOCALIZE_MIN_SAMPLES_PUSHBOX;
    }
    else
    {
        request_car_periodic(CONTROL_REQ_CAR_PERIOD_WAIT);
    }

    if (car_pose_updated)
    {
        car_pose_updated = false;
        accept_sample = 1U;
    }
    else if (!g_relocalize_force_fresh_pose &&
             g_localize_sample_count == 0U && car_pose_ready)
    {
        /* 启动初期若已有缓存帧，允许先吃一帧，避免死等 updated 标志�?*/
        accept_sample = 1U;
    }

    if (!accept_sample)
    {
        return;
    }
    if (!get_camera_pose_meter(&cam_x_m, &cam_y_m, &cam_yaw_deg))
    {
        return;
    }

    g_localize_sum_x_m += cam_x_m;
    g_localize_sum_y_m += cam_y_m;
    g_localize_sum_yaw_deg += cam_yaw_deg;
    g_localize_sample_count++;

    if (g_localize_sample_count < min_samples)
    {
        return;
    }

    path_follow_reset_pose(g_localize_sum_x_m / (float)g_localize_sample_count,
                           g_localize_sum_y_m / (float)g_localize_sample_count,
                           g_localize_sum_yaw_deg / (float)g_localize_sample_count);
    path_follow_hold_current_yaw();

    if (!g_map_right_yaw_ready)
    {
        g_map_right_yaw_deg = g_localize_sum_yaw_deg / (float)g_localize_sample_count;
        g_map_right_yaw_ready = 1U;
    }
    g_relocalize_force_fresh_pose = 0U;
    if (g_control_flow_phase == CONTROL_FLOW_IDENTIFY)
    {
        g_body_map_dir = CONTROL_MAP_DIR_RIGHT;
    }
    mark_wait_new_map_frame();
    g_control_stage = CONTROL_STAGE_WAIT_CAMERA_DATA;
}

/**
 * @brief 处理“等待地图”阶段�?
 *
 * 阶段目标�?
 * - 周期请求地图与车位姿
 * - 收到有效地图后切换到规划阶段
 */
static void handle_wait_camera_data(void)
{
    float cam_x_m = 0.0f;
    float cam_y_m = 0.0f;
    float cam_yaw_deg = 0.0f;

    request_map_periodic();
    request_car_periodic(CONTROL_REQ_CAR_PERIOD_WAIT);

    if (car_pose_updated)
    {
        car_pose_updated = false;
        if (get_camera_pose_meter(&cam_x_m, &cam_y_m, &cam_yaw_deg))
        {
            /* 等地图阶段若拿到新位姿，顺带轻量同步一次里程计�?*/
            path_follow_reset_pose(cam_x_m, cam_y_m, cam_yaw_deg);
        }
    }

    if (map_data_ready && map_frame_count > 0U)
    {
        if (!g_wait_new_map_frame || map_frame_count != g_wait_map_frame_base)
        {
            g_wait_new_map_frame = 0U;
            map_data_updated = false;
            g_control_stage = CONTROL_STAGE_PLAN_PATH;
        }
    }
}

/* ========================= 对外接口实现 ========================= */

static uint8 load_identify_path_for_execution(void)
{
    if (!prepare_identify_endpoints())
    {
        return 0U;
    }

    if (g_identify_endpoint_count == 0U)
    {
        return 0U;
    }

    return start_identify_segment(g_identify_endpoint_indices[0]);
}

static void handle_identify_execute_path(void)
{
    path_follow_status_t st = {0};
    size_t endpoint_idx = 0U;
    const control_identify_target_t *curr_target = NULL;
    float target_yaw_deg = 0.0f;
    float yaw_err_deg = 0.0f;

    if (g_identify_endpoint_cursor >= g_identify_endpoint_count)
    {
        finish_identify_flow_and_relocalize();
        return;
    }

    switch (g_identify_exec_state)
    {
    case CONTROL_IDENTIFY_EXEC_MOVE_SEGMENT:
        if (g_identify_segment_running)
        {
            path_follow_get_status(&st);
            if (st.active)
            {
                return;
            }
            g_identify_segment_running = 0U;
        }

        endpoint_idx = g_identify_endpoint_indices[g_identify_endpoint_cursor];
        if (g_identify_endpoint_need_action[g_identify_endpoint_cursor])
        {
            if (collect_identify_targets_on_exec_point(&g_exec_path[endpoint_idx]))
            {
                g_identify_rotate_started = 0U;
                g_identify_exec_state = CONTROL_IDENTIFY_EXEC_ROTATE_TO_TARGET;
                return;
            }
        }

        if (!advance_identify_endpoint_or_finish())
        {
            g_control_stage = CONTROL_STAGE_ERROR;
        }
        break;

    case CONTROL_IDENTIFY_EXEC_ROTATE_TO_TARGET:
        if (g_identify_target_cursor >= g_identify_target_count)
        {
            g_identify_rotate_started = 0U;
            g_identify_exec_state = CONTROL_IDENTIFY_EXEC_ROTATE_BACK;
            return;
        }

        curr_target = &g_identify_targets[g_identify_target_cursor];
        target_yaw_deg = map_dir_to_yaw_deg(curr_target->face_dir);
        yaw_err_deg = wrap_yaw_deg_local(target_yaw_deg - eulerAngle.yaw);
        if (fabsf(yaw_err_deg) <= CONTROL_IDENTIFY_YAW_ALIGN_TOL_DEG)
        {
            g_identify_rotate_started = 0U;
            g_body_map_dir = curr_target->face_dir;
            g_identify_exec_state = CONTROL_IDENTIFY_EXEC_DO_RECOGNIZE;
            return;
        }

        if (!g_identify_rotate_started)
        {
            path_follow_start_rotate_to_yaw(target_yaw_deg);
            car_go_flag = 1U;
            car_stop_flag = 0U;
            g_identify_rotate_started = 1U;
            return;
        }

        path_follow_get_status(&st);
        if (st.active)
        {
            return;
        }

        g_identify_rotate_started = 0U;
        g_body_map_dir = curr_target->face_dir;
        g_identify_exec_state = CONTROL_IDENTIFY_EXEC_DO_RECOGNIZE;
        break;

    case CONTROL_IDENTIFY_EXEC_DO_RECOGNIZE:
        if (g_identify_target_cursor < g_identify_target_count)
        {
            curr_target = &g_identify_targets[g_identify_target_cursor];
            identify_action_stub(curr_target);

            if (curr_target->obj_type == CONTROL_IDENTIFY_OBJ_BOX &&
                curr_target->obj_index < MAX_BOXES)
            {
                g_identified_box_flags[curr_target->obj_index] = 1U;
            }
            else if (curr_target->obj_type == CONTROL_IDENTIFY_OBJ_TARGET &&
                     curr_target->obj_index < MAX_TARGETS)
            {
                g_identified_target_flags[curr_target->obj_index] = 1U;
            }

            g_identify_target_cursor++;
        }

        if (g_identify_target_cursor < g_identify_target_count)
        {
            g_identify_exec_state = CONTROL_IDENTIFY_EXEC_ROTATE_TO_TARGET;
        }
        else
        {
            g_identify_rotate_started = 0U;
            g_identify_exec_state = CONTROL_IDENTIFY_EXEC_ROTATE_BACK;
        }
        break;

    case CONTROL_IDENTIFY_EXEC_ROTATE_BACK:
        /* 不再强制回正到“地图右向”，保持当前车头朝向直接进入下一段。 */
        g_identify_rotate_started = 0U;
        if (!advance_identify_endpoint_or_finish())
        {
            g_control_stage = CONTROL_STAGE_ERROR;
        }
        break;

    case CONTROL_IDENTIFY_EXEC_IDLE:
    default:
        g_control_stage = CONTROL_STAGE_ERROR;
        break;
    }
}

void control_init(void)
{
    g_control_stage = CONTROL_STAGE_PRESTART_MOVE;
    g_control_flow_phase = CONTROL_FLOW_IDENTIFY;
    g_path_plan_paused = 0U;
    g_plan_ready = 0U;
    g_exec_steps = 0U;
    memset(g_exec_path, 0, sizeof(g_exec_path));

    reset_localization_accumulator();
    g_prestart_move_started = 0U;
    g_map_right_yaw_deg = 0.0f;
    g_map_right_yaw_ready = 0U;
    g_body_map_dir = CONTROL_MAP_DIR_RIGHT;
    g_wait_new_map_frame = 0U;
    g_wait_map_frame_base = 0U;
    g_relocalize_force_fresh_pose = 0U;
    reset_identify_runtime_state();

    g_map_req_loop_cnt = 0U;
    g_car_req_loop_cnt = 0U;

    path_follow_set_pause_indices(NULL, 0U, 0U);
    path_follow_set_path(NULL, 0U);

    /* 启动时默认关闭运动输出，路径准备好后再放行�?*/
    car_go_flag = 0U;
    car_stop_flag = 0U;

    map_data_updated = false;
    car_pose_updated = false;
}

void control_restart(void)
{
    g_control_stage = CONTROL_STAGE_PRESTART_MOVE;
    g_control_flow_phase = CONTROL_FLOW_IDENTIFY;
    g_path_plan_paused = 0U;
    g_plan_ready = 0U;
    g_exec_steps = 0U;
    reset_localization_accumulator();
    g_prestart_move_started = 0U;
    g_map_right_yaw_deg = 0.0f;
    g_map_right_yaw_ready = 0U;
    g_body_map_dir = CONTROL_MAP_DIR_RIGHT;
    g_wait_new_map_frame = 0U;
    g_wait_map_frame_base = 0U;
    g_relocalize_force_fresh_pose = 0U;
    reset_identify_runtime_state();
    g_map_req_loop_cnt = 0U;
    g_car_req_loop_cnt = 0U;

    path_follow_set_path(NULL, 0U);
    path_follow_set_pause_indices(NULL, 0U, 0U);

    car_go_flag = 0U;
    car_stop_flag = 0U;
    map_data_updated = false;
    car_pose_updated = false;
}

void control_process(void)
{
    path_follow_status_t st = {0};

    /* 统一串口解析入口：每圈都先消�?FIFO 并更新地�?位姿缓存�?*/
    process_blob_data();
    process_vision_data();

    switch (g_control_stage)
    {
    case CONTROL_STAGE_PRESTART_MOVE:
        handle_prestart_move();
        break;

    case CONTROL_STAGE_STARTUP_LOCALIZE:
        handle_startup_localization();
        break;

    case CONTROL_STAGE_WAIT_CAMERA_DATA:
        handle_wait_camera_data();
        break;

    case CONTROL_STAGE_PLAN_PATH:
        begin_path_plan_pause();
        if (control_plan_path())
        {
            g_control_stage = CONTROL_STAGE_LOAD_PATH;
        }
        else
        {
            g_control_stage = CONTROL_STAGE_ERROR;
        }
        end_path_plan_pause();
        break;

    case CONTROL_STAGE_LOAD_PATH:
        if (!g_plan_ready || g_exec_steps < 2U)
        {
            g_control_stage = CONTROL_STAGE_ERROR;
            break;
        }

        if (g_control_flow_phase == CONTROL_FLOW_IDENTIFY)
        {
            if (!load_identify_path_for_execution())
            {
                g_control_stage = CONTROL_STAGE_ERROR;
                break;
            }
            g_control_stage = CONTROL_STAGE_EXECUTE_PATH;
        }
        else
        {
            path_follow_set_pause_indices(NULL, 0U, 0U);
            path_follow_hold_current_yaw();
            path_follow_set_path(g_exec_path, g_exec_steps);

            car_go_flag = 1U;
            car_stop_flag = 0U;
            g_control_stage = CONTROL_STAGE_EXECUTE_PATH;
        }
        break;

    case CONTROL_STAGE_EXECUTE_PATH:
        if (g_control_flow_phase == CONTROL_FLOW_IDENTIFY)
        {
            handle_identify_execute_path();
            break;
        }

        path_follow_get_status(&st);
        if (!st.active)
        {
            /* 路径执行结束后切换到停车保持态�?*/
            car_go_flag = 1U;
            car_stop_flag = 1U;
            g_control_stage = CONTROL_STAGE_FINISHED;
        }
        break;

    case CONTROL_STAGE_FINISHED:
        /* 完成态保持停车，等待外部调用 control_restart()�?*/
        break;

    case CONTROL_STAGE_ERROR:
        /* 错误态保持安全停车，并持续请求地图，拿到新图后自动重试规划�?*/
        car_go_flag = 1U;
        car_stop_flag = 1U;
        request_map_periodic();
        request_car_periodic(CONTROL_REQ_CAR_PERIOD_WAIT);

        if (map_data_updated)
        {
            map_data_updated = false;
            g_control_stage = CONTROL_STAGE_PLAN_PATH;
        }
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

/**
 * @brief 设置规划模式标志位�?
 *
 * 为了避免非法输入破坏流程，除 CONTROL_PLAN_MODE_1 外都归一�?
 * CONTROL_PLAN_MODE_2�?
 */
void control_set_plan_mode(control_plan_mode_t mode)
{
    if (mode == CONTROL_PLAN_MODE_1)
    {
        g_control_plan_mode = CONTROL_PLAN_MODE_1;
    }
    else
    {
        /* ʶ�����̶̹��������Զ�ִ�У��˴�����������׶� Mode1/Mode2�� */
        g_control_plan_mode = CONTROL_PLAN_MODE_2;
    }
}

/**
 * @brief 获取当前规划模式标志位�?
 */
control_plan_mode_t control_get_plan_mode(void)
{
    return g_control_plan_mode;
}

uint8 control_is_path_plan_paused(void)
{
    return g_path_plan_paused;
}
