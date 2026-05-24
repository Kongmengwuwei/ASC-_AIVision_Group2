#include "Control.h"
#include "data_handle.h"
#include "Game_logic.h"
#include "Map_Path_Data.h"
#include "Mymenu.h"
#include "path.h"
#include "path_follow.h"
#include "Attitude.h"
#include "zf_driver_delay.h"
#include <math.h>
#include <string.h>

/*
 * 手动开关：初始定位之后，是否继续使用视觉 CAR 位姿修正定位。
 *
 * 注意：起步后的第一次定位始终强制使用视觉，因为系统需要先拿到小车真实位置
 * 和地图方向基准；该开关只控制后续流程：
 * - 识别结束进入推箱子阶段前是否二次视觉定位；
 * - 等待地图阶段收到新 CAR 位姿时是否顺带同步 path_follow。
 */
static uint8 g_control_use_followup_vision_localization = 1U;

/*
 * 手动选择起步发车方向。
 * 0：地图右，对应车前方；
 * 1：地图上，对应车左方；
 * 2：地图左，对应车后方；
 * 3：地图下，对应车右方；
 * 4：保留/默认，按地图右处理。
 *
 * 小车只平移到对应方向，车头朝向保持不变。
 */
static uint8 g_control_prestart_depart_dir = 0U;

/*
 * 手动开关：识别阶段是否启用“段前提前转向”。
 * 1：每段识别短路程出发前先转到识别朝向，到点后直接识别。
 * 0：关闭提前转向，恢复为到达识别点后再原地转向识别。
 */
static uint8 g_control_identify_prerotate_enabled = 1U;

/* ========================= 参数配置�?========================= */

/*
 * MAP/CAR requests are sent on demand. These loop-count guards only retry
 * when a request is already pending and no matching response has arrived.
 */
#define CONTROL_REQ_MAP_RETRY_TIMEOUT_LOOPS 200U
#define CONTROL_REQ_CAR_RETRY_TIMEOUT_LOOPS 200U
/* Wait before consuming CAR pose frames to avoid using delayed camera data. */
#define CONTROL_CAMERA_POSE_SETTLE_DELAY_MS 200U
/* 推炸弹到爆炸点后停留 0.5s，等待爆炸效果生效 */
#define CONTROL_BOMB_EXPLOSION_PAUSE_MS 500U


/**
 * @brief 初始定位最少采样帧数�?
 *
 * 使用简单平均抑制单帧抖动。达到该采样数后才完成初始定位�?
 */
#define CONTROL_LOCALIZE_MIN_SAMPLES 3U
#define CONTROL_RELOCALIZE_MIN_SAMPLES_PUSHBOX 3U

/**
 * @brief 起步位移距离（米）�?
 *
 * 这里采用与地图网格一致的 `GRID_SIZE_M`，即 0.3m�?
 */
#define CONTROL_PRESTART_OFFSET_M 0.30f

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
#define CONTROL_COORD_TRANSPOSE_COMPENSATE PATH_COORD_TRANSPOSE_COMPENSATE
/* 在转置补偿基础上，额外翻转“上下轴”（对应 map �?row 方向）�?*/
#define CONTROL_COORD_FLIP_VERTICAL PATH_COORD_FLIP_VERTICAL
#define CONTROL_WORLD_X_MAX_M PATH_WORLD_X_MAX_M
#define CONTROL_WORLD_Y_MAX_M PATH_WORLD_Y_MAX_M


/* ========================= 内部数据结构 ========================= */

#define CONTROL_IDENTIFY_MAX_TARGETS_PER_POINT (MAX_BOXES + MAX_TARGETS)
#define CONTROL_IDENTIFY_YAW_ALIGN_TOL_DEG 5.0f
#define CONTROL_IDENTIFY_RECOG_TIMEOUT_LOOPS 250U
#define CONTROL_IDENTIFY_ID_UNASSIGNED 0xFFU
#define CONTROL_IDENTIFY_ID_MIN 0U
#define CONTROL_IDENTIFY_ID_MAX 9U
#define CONTROL_IDENTIFY_ID_ELIMINATE 10U

typedef enum
{
    CONTROL_FLOW_IDENTIFY = 0U,
    CONTROL_FLOW_PUSHBOX = 1U
} control_flow_phase_t;

typedef enum
{
    CONTROL_PHASE_STEP_LOCALIZE = 0U,
    CONTROL_PHASE_STEP_WAIT_CAMERA_DATA,
    CONTROL_PHASE_STEP_PLAN_PATH,
    CONTROL_PHASE_STEP_LOAD_PATH,
    CONTROL_PHASE_STEP_EXECUTE_PATH,
    CONTROL_PHASE_STEP_FINISHED
} control_phase_step_t;

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
    VisionRecognitionDistance recog_distance;
    uint8 obj_index;
} control_identify_target_t;

typedef struct
{
    uint8 valid;
    uint8 row;
    uint8 col;
    uint8 id;
} control_identify_id_record_t;

typedef enum
{
    CONTROL_IDENTIFY_EXEC_IDLE = 0U,
    CONTROL_IDENTIFY_EXEC_ROTATE_BEFORE_SEGMENT,
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

static volatile uint8 g_control_start_enabled = 0U;

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
/*
 * 推箱阶段使用的规划模式：
 * - 默认 Mode1，表示识别阶段得到 0~9 后按箱子/目标 ID 配对推箱；
 * - 首个 IMG/NUM 识别若收到明确的非 0~9 结果，会自动切到 Mode1；
 * - 首个 IMG/NUM 等待超时只重发请求，不直接判 Mode1，避免主循环过快导致误判；
 * - 仍保留 control_set_plan_mode() 作为外部手动覆盖入口。
 */
static control_plan_mode_t g_control_plan_mode = CONTROL_PLAN_MODE_1;
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
static uint8 g_identify_box_id_assigned[MAX_BOXES] = {0U};
static uint8 g_identify_target_id_assigned[MAX_TARGETS] = {0U};
static uint8 g_identify_recog_waiting = 0U;
static uint16 g_identify_recog_wait_loops = 0U;
static VisionRecognitionType g_identify_recog_wait_type = VISION_RECOGNITION_NONE;
/* 首个 IMG/NUM 识别结果用于判断后续是否按 ID 配对推箱。 */
static uint8 g_identify_first_result_checked = 0U;
/* 首个 IMG/NUM 收到非 0~9 结果时置 1，当前识别阶段会立刻结束并转 Mode1。 */
static uint8 g_identify_abort_to_mode1 = 0U;

static control_identify_id_record_t g_saved_box_id_records[MAX_BOXES] = {{0}};
static control_identify_id_record_t g_saved_target_id_records[MAX_TARGETS] = {{0}};
static uint8 g_saved_identify_ids_ready = 0U;

/**
 * @brief 初始定位累计样本数�?
 */
static uint8 g_localize_sample_count = 0U;
static uint8 g_localize_camera_settled = 0U;

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

static uint8 g_map_request_waiting = 0U;
static uint16 g_map_request_wait_loops = 0U;

static uint8 g_car_request_waiting = 0U;
static uint16 g_car_request_wait_loops = 0U;

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

/**
 * @brief 按“大阶段 + 小步骤”生成公开运行状态。
 *
 * Control 内部仍然用 g_control_flow_phase 表示当前是“识别阶段”还是“推箱子阶段”；
 * 对外显示和调试时则直接使用 control_stage_t。所有阶段内的状态跳转都经过这里，
 * 这样不会出现一部分代码写通用 PLAN/EXECUTE、另一部分代码写阶段状态的混乱情况。
 */
static control_stage_t make_control_phase_stage(control_flow_phase_t phase,
                                                control_phase_step_t step)
{
    if (phase == CONTROL_FLOW_PUSHBOX)
    {
        switch (step)
        {
        case CONTROL_PHASE_STEP_LOCALIZE:
            return CONTROL_STAGE_PUSHBOX_LOCALIZE;
        case CONTROL_PHASE_STEP_WAIT_CAMERA_DATA:
            return CONTROL_STAGE_PUSHBOX_WAIT_CAMERA_DATA;
        case CONTROL_PHASE_STEP_PLAN_PATH:
            return CONTROL_STAGE_PUSHBOX_PLAN_PATH;
        case CONTROL_PHASE_STEP_LOAD_PATH:
            return CONTROL_STAGE_PUSHBOX_LOAD_PATH;
        case CONTROL_PHASE_STEP_EXECUTE_PATH:
            return CONTROL_STAGE_PUSHBOX_EXECUTE_PATH;
        case CONTROL_PHASE_STEP_FINISHED:
        default:
            return CONTROL_STAGE_PUSHBOX_FINISHED;
        }
    }

    switch (step)
    {
    case CONTROL_PHASE_STEP_LOCALIZE:
        return CONTROL_STAGE_IDENTIFY_LOCALIZE;
    case CONTROL_PHASE_STEP_WAIT_CAMERA_DATA:
        return CONTROL_STAGE_IDENTIFY_WAIT_CAMERA_DATA;
    case CONTROL_PHASE_STEP_PLAN_PATH:
        return CONTROL_STAGE_IDENTIFY_PLAN_PATH;
    case CONTROL_PHASE_STEP_LOAD_PATH:
        return CONTROL_STAGE_IDENTIFY_LOAD_PATH;
    case CONTROL_PHASE_STEP_EXECUTE_PATH:
        return CONTROL_STAGE_IDENTIFY_EXECUTE_PATH;
    case CONTROL_PHASE_STEP_FINISHED:
    default:
        return CONTROL_STAGE_IDENTIFY_FINISHED;
    }
}

/**
 * @brief 切换到当前大阶段下的某个小步骤。
 *
 * 例如识别阶段调用 CONTROL_PHASE_STEP_PLAN_PATH 会得到
 * CONTROL_STAGE_IDENTIFY_PLAN_PATH；推箱子阶段调用同一个小步骤会得到
 * CONTROL_STAGE_PUSHBOX_PLAN_PATH。这样菜单和调试变量能一眼看出当前属于哪条流程。
 */
static void set_control_phase_stage(control_phase_step_t step)
{
    g_control_stage = make_control_phase_stage(g_control_flow_phase, step);
}

/**
 * @brief 当前大阶段是否需要进入视觉定位状态。
 *
 * 识别阶段的第一次定位不能被菜单开关关闭，否则规划起点和地图方向基准都可能无效。
 * 推箱子阶段属于后续重定位，由菜单开关控制是否继续使用视觉修正。
 */
static uint8 should_enter_vision_localization_stage(void)
{
    if (g_control_flow_phase == CONTROL_FLOW_IDENTIFY)
    {
        return 1U;
    }
    return g_control_use_followup_vision_localization;
}

static void reset_localization_accumulator(void)
{
    g_localize_sample_count = 0U;
    g_localize_camera_settled = 0U;
    g_localize_sum_x_m = 0.0f;
    g_localize_sum_y_m = 0.0f;
    g_localize_sum_yaw_deg = 0.0f;
}

static void clear_map_request_wait(void)
{
    g_map_request_waiting = 0U;
    g_map_request_wait_loops = 0U;
}

static void clear_car_request_wait(void)
{
    g_car_request_waiting = 0U;
    g_car_request_wait_loops = 0U;
}

static void send_map_request_once(void)
{
    uart_send_map_request();
    g_map_request_waiting = 1U;
    g_map_request_wait_loops = 0U;
}

static void send_car_request_once(void)
{
    uart_send_car_request();
    g_car_request_waiting = 1U;
    g_car_request_wait_loops = 0U;
}

static void service_map_request_wait(void)
{
    if (!g_map_request_waiting)
    {
        send_map_request_once();
        return;
    }

    if (g_map_request_wait_loops < CONTROL_REQ_MAP_RETRY_TIMEOUT_LOOPS)
    {
        g_map_request_wait_loops++;
    }
    if (g_map_request_wait_loops >= CONTROL_REQ_MAP_RETRY_TIMEOUT_LOOPS)
    {
        send_map_request_once();
    }
}

static void service_car_request_wait(void)
{
    if (!g_car_request_waiting)
    {
        send_car_request_once();
        return;
    }

    if (g_car_request_wait_loops < CONTROL_REQ_CAR_RETRY_TIMEOUT_LOOPS)
    {
        g_car_request_wait_loops++;
    }
    if (g_car_request_wait_loops >= CONTROL_REQ_CAR_RETRY_TIMEOUT_LOOPS)
    {
        send_car_request_once();
    }
}

static void mark_wait_new_map_frame(void)
{
    g_wait_new_map_frame = 1U;
    g_wait_map_frame_base = map_frame_count;
    clear_map_request_wait();
}

static void reset_identify_recognition_wait(void)
{
    g_identify_recog_waiting = 0U;
    g_identify_recog_wait_loops = 0U;
    g_identify_recog_wait_type = VISION_RECOGNITION_NONE;
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
    reset_identify_recognition_wait();
    g_identify_first_result_checked = 0U;
    g_identify_abort_to_mode1 = 0U;

    memset(g_identified_box_flags, 0, sizeof(g_identified_box_flags));
    memset(g_identified_target_flags, 0, sizeof(g_identified_target_flags));
    memset(g_identify_box_id_assigned, 0, sizeof(g_identify_box_id_assigned));
    memset(g_identify_target_id_assigned, 0, sizeof(g_identify_target_id_assigned));
}

static void inverse_remap_exec_path_point(Position *p)
{
    path_inverse_remap_exec_point(p);
}

/* 前向声明：供识别分段和主路径下发时复用炸弹停留配置。 */
static void configure_bomb_pause_for_path(const Position *path, size_t steps);

static uint8 is_identification_marker(uint8 marker_id)
{
    return (marker_id == IDENTIFICATION) ? 1U : 0U;
}

static uint8 resolve_map_dir_from_delta(int32 d_row, int32 d_col, control_map_dir_t *dir_out)
{
    if (dir_out == NULL)
    {
        return 0U;
    }

    if (d_row == 0 && d_col > 0)
    {
        *dir_out = CONTROL_MAP_DIR_RIGHT;
        return 1U;
    }
    if (d_row == 0 && d_col < 0)
    {
        *dir_out = CONTROL_MAP_DIR_LEFT;
        return 1U;
    }
    if (d_row < 0 && d_col == 0)
    {
        *dir_out = CONTROL_MAP_DIR_UP;
        return 1U;
    }
    if (d_row > 0 && d_col == 0)
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

static control_map_dir_t get_prestart_depart_map_dir(void)
{
    switch (g_control_prestart_depart_dir)
    {
    case 1U:
        return CONTROL_MAP_DIR_UP;
    case 2U:
        return CONTROL_MAP_DIR_LEFT;
    case 3U:
        return CONTROL_MAP_DIR_DOWN;
    case 4U:
    case 0U:
    default:
        return CONTROL_MAP_DIR_RIGHT;
    }
}

static void clear_saved_identify_ids(void)
{
    memset(g_saved_box_id_records, 0, sizeof(g_saved_box_id_records));
    memset(g_saved_target_id_records, 0, sizeof(g_saved_target_id_records));
    g_saved_identify_ids_ready = 0U;
}

static void reset_control_runtime_state(void)
{
    g_control_start_enabled = 0U;
    g_control_stage = CONTROL_STAGE_IDLE;
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
    clear_saved_identify_ids();
    vision_clear_pending_data();

    clear_map_request_wait();
    clear_car_request_wait();

    path_follow_set_pause_indices(NULL, 0U, 0U);
    path_follow_set_path(NULL, 0U);

    car_go_flag = 0U;
    car_stop_flag = 0U;
    map_data_updated = false;
    car_pose_updated = false;
}

static uint8 identify_result_to_valid_id(const VisionRecognitionResult *result, uint8 *id_out)
{
    if (result == NULL || id_out == NULL)
    {
        return 0U;
    }
    if (!result->success || result->mode_marker || !result->label_is_number)
    {
        return 0U;
    }
    if (result->label_value < (int32)CONTROL_IDENTIFY_ID_MIN ||
        result->label_value > (int32)CONTROL_IDENTIFY_ID_MAX)
    {
        return 0U;
    }

    *id_out = (uint8)result->label_value;
    return 1U;
}

static uint8 update_plan_mode_from_first_identify_result(const VisionRecognitionResult *result,
                                                         uint8 valid_id)
{
    if (g_identify_first_result_checked)
    {
        return 1U;
    }

    if (!valid_id && result != NULL && result->mode_marker)
    {
        g_identify_first_result_checked = 1U;
        /*
         * 新版摄像头用 IMG/NUM,-1,0 表示首帧已识别到关卡模式纯色块。
         * 这不是串口错误，也不是普通识别失败；它表示本关不需要继续做 ID 配对，
         * 因此保持原本要求：立即切到 Mode1，并结束识别阶段。
         */
        g_control_plan_mode = CONTROL_PLAN_MODE_1;
        g_identify_abort_to_mode1 = 1U;
        return 1U;
    }

    if (!valid_id && result != NULL && !result->success)
    {
        /*
         * IMG/NUM,-1,-1 现在只表示本次图案/数字识别失败。
         * 它不再作为 Mode1 的判断依据，首个有效模式结果到来前继续重发等待。
         */
        return 0U;
    }

    g_identify_first_result_checked = 1U;
    if (valid_id)
    {
        /* 首个 IMG/NUM 识别到 0~9：说明需要按 ID 配对，后续推箱使用 Mode2。 */
        g_control_plan_mode = CONTROL_PLAN_MODE_2;
    }
    else
    {
        /* 首个 IMG/NUM 没有正常数字：说明无需配对，立即结束识别并转 Mode1。 */
        g_control_plan_mode = CONTROL_PLAN_MODE_1;
        g_identify_abort_to_mode1 = 1U;
    }
    return 1U;
}

static void save_identify_ids_from_current_map(void)
{
    size_t i = 0U;
    size_t box_cnt = Boxes_count;
    size_t target_cnt = Targets_count;

    if (box_cnt > MAX_BOXES)
    {
        box_cnt = MAX_BOXES;
    }
    if (target_cnt > MAX_TARGETS)
    {
        target_cnt = MAX_TARGETS;
    }

    memset(g_saved_box_id_records, 0, sizeof(g_saved_box_id_records));
    memset(g_saved_target_id_records, 0, sizeof(g_saved_target_id_records));

    for (i = 0U; i < box_cnt; i++)
    {
        g_saved_box_id_records[i].valid = 1U;
        g_saved_box_id_records[i].row = boxes[i].row;
        g_saved_box_id_records[i].col = boxes[i].col;
        g_saved_box_id_records[i].id = boxes[i].id;
    }

    for (i = 0U; i < target_cnt; i++)
    {
        g_saved_target_id_records[i].valid = 1U;
        g_saved_target_id_records[i].row = targets[i].row;
        g_saved_target_id_records[i].col = targets[i].col;
        g_saved_target_id_records[i].id = targets[i].id;
    }

    g_saved_identify_ids_ready = 1U;
}

static uint8 lookup_saved_id_by_cell(const control_identify_id_record_t *records,
                                     size_t record_capacity,
                                     uint8 row,
                                     uint8 col,
                                     uint8 *id_out)
{
    size_t i = 0U;

    if (records == NULL || id_out == NULL)
    {
        return 0U;
    }

    for (i = 0U; i < record_capacity; i++)
    {
        if (!records[i].valid)
        {
            continue;
        }
        if (records[i].row == row && records[i].col == col)
        {
            *id_out = records[i].id;
            return 1U;
        }
    }
    return 0U;
}

static void apply_saved_identify_ids_to_current_map(void)
{
    size_t i = 0U;
    size_t box_cnt = Boxes_count;
    size_t target_cnt = Targets_count;
    uint8 id = 0U;

    if (!g_saved_identify_ids_ready)
    {
        return;
    }

    if (box_cnt > MAX_BOXES)
    {
        box_cnt = MAX_BOXES;
    }
    if (target_cnt > MAX_TARGETS)
    {
        target_cnt = MAX_TARGETS;
    }

    for (i = 0U; i < box_cnt; i++)
    {
        if (lookup_saved_id_by_cell(g_saved_box_id_records, MAX_BOXES, boxes[i].row, boxes[i].col, &id))
        {
            boxes[i].id = id;
        }
    }

    for (i = 0U; i < target_cnt; i++)
    {
        if (lookup_saved_id_by_cell(g_saved_target_id_records, MAX_TARGETS, targets[i].row, targets[i].col, &id))
        {
            targets[i].id = id;
        }
    }
}

static void finalize_identify_ids_for_pushbox(void)
{
    uint8 box_id_count[256] = {0U};
    uint8 target_id_count[256] = {0U};
    size_t unassigned_box_indices[MAX_BOXES] = {0U};
    size_t unassigned_target_indices[MAX_TARGETS] = {0U};
    size_t unassigned_box_count = 0U;
    size_t unassigned_target_count = 0U;
    size_t box_cnt = Boxes_count;
    size_t target_cnt = Targets_count;
    size_t i = 0U;
    size_t box_cursor = 0U;
    size_t target_cursor = 0U;
    uint8 id = 0U;
    uint8 known_ids_paired = 1U;
    uint8 has_any_assigned = 0U;
    uint16 next_auto_id = CONTROL_IDENTIFY_ID_ELIMINATE;

    if (box_cnt > MAX_BOXES)
    {
        box_cnt = MAX_BOXES;
    }
    if (target_cnt > MAX_TARGETS)
    {
        target_cnt = MAX_TARGETS;
    }

    for (i = 0U; i < box_cnt; i++)
    {
        if (g_identify_box_id_assigned[i] && boxes[i].id != CONTROL_IDENTIFY_ID_UNASSIGNED)
        {
            box_id_count[boxes[i].id]++;
            has_any_assigned = 1U;
        }
        else if (unassigned_box_count < MAX_BOXES)
        {
            boxes[i].id = CONTROL_IDENTIFY_ID_UNASSIGNED;
            unassigned_box_indices[unassigned_box_count++] = i;
        }
    }

    for (i = 0U; i < target_cnt; i++)
    {
        if (g_identify_target_id_assigned[i] && targets[i].id != CONTROL_IDENTIFY_ID_UNASSIGNED)
        {
            target_id_count[targets[i].id]++;
            has_any_assigned = 1U;
        }
        else if (unassigned_target_count < MAX_TARGETS)
        {
            targets[i].id = CONTROL_IDENTIFY_ID_UNASSIGNED;
            unassigned_target_indices[unassigned_target_count++] = i;
        }
    }

    for (id = CONTROL_IDENTIFY_ID_MIN; id <= CONTROL_IDENTIFY_ID_MAX; id++)
    {
        if (box_id_count[id] != target_id_count[id])
        {
            known_ids_paired = 0U;
            break;
        }
    }

    if (has_any_assigned &&
        known_ids_paired &&
        unassigned_box_count == unassigned_target_count)
    {
        for (i = 0U; i < unassigned_box_count; i++)
        {
            size_t idx = unassigned_box_indices[i];
            boxes[idx].id = CONTROL_IDENTIFY_ID_ELIMINATE;
            g_identify_box_id_assigned[idx] = 1U;
        }
        for (i = 0U; i < unassigned_target_count; i++)
        {
            size_t idx = unassigned_target_indices[i];
            targets[idx].id = CONTROL_IDENTIFY_ID_ELIMINATE;
            g_identify_target_id_assigned[idx] = 1U;
        }
        save_identify_ids_from_current_map();
        return;
    }

    for (id = CONTROL_IDENTIFY_ID_MIN; id <= CONTROL_IDENTIFY_ID_MAX; id++)
    {
        while (box_id_count[id] > target_id_count[id] &&
               target_cursor < unassigned_target_count)
        {
            size_t target_idx = unassigned_target_indices[target_cursor++];
            targets[target_idx].id = id;
            g_identify_target_id_assigned[target_idx] = 1U;
            target_id_count[id]++;
        }

        while (target_id_count[id] > box_id_count[id] &&
               box_cursor < unassigned_box_count)
        {
            size_t box_idx = unassigned_box_indices[box_cursor++];
            boxes[box_idx].id = id;
            g_identify_box_id_assigned[box_idx] = 1U;
            box_id_count[id]++;
        }
    }

    while (box_cursor < unassigned_box_count && target_cursor < unassigned_target_count)
    {
        size_t box_idx = unassigned_box_indices[box_cursor++];
        size_t target_idx = unassigned_target_indices[target_cursor++];

        while (next_auto_id < 255U &&
               (box_id_count[next_auto_id] > 0U || target_id_count[next_auto_id] > 0U))
        {
            next_auto_id++;
        }
        if (next_auto_id >= 255U)
        {
            next_auto_id = CONTROL_IDENTIFY_ID_ELIMINATE;
        }

        boxes[box_idx].id = (uint8)next_auto_id;
        targets[target_idx].id = (uint8)next_auto_id;
        g_identify_box_id_assigned[box_idx] = 1U;
        g_identify_target_id_assigned[target_idx] = 1U;
        box_id_count[next_auto_id]++;
        target_id_count[next_auto_id]++;
        next_auto_id++;
    }

    while (box_cursor < unassigned_box_count)
    {
        size_t box_idx = unassigned_box_indices[box_cursor++];
        boxes[box_idx].id = CONTROL_IDENTIFY_ID_ELIMINATE;
        g_identify_box_id_assigned[box_idx] = 1U;
    }

    while (target_cursor < unassigned_target_count)
    {
        size_t target_idx = unassigned_target_indices[target_cursor++];
        targets[target_idx].id = CONTROL_IDENTIFY_ID_ELIMINATE;
        g_identify_target_id_assigned[target_idx] = 1U;
    }

    save_identify_ids_from_current_map();
}

static uint8 start_identify_recognition_request(const control_identify_target_t *target,
                                                VisionRecognitionDistance distance)
{
    if (target == NULL)
    {
        return 0U;
    }

    g_identify_recog_wait_loops = 0U;
    vision_clear_pending_data();

    if (target->obj_type == CONTROL_IDENTIFY_OBJ_BOX)
    {
        if (!uart_send_vision_img_request_by_distance(distance))
        {
            return 0U;
        }
        g_identify_recog_wait_type = VISION_RECOGNITION_IMG;
    }
    else
    {
        if (!uart_send_vision_num_request_by_distance(distance))
        {
            return 0U;
        }
        g_identify_recog_wait_type = VISION_RECOGNITION_NUM;
    }

    g_identify_recog_waiting = 1U;
    return 1U;
}

static uint8 identify_action_stub(const control_identify_target_t *target)
{
    VisionRecognitionResult result = {0};
    uint8 recognized_id = 0U;
    uint8 valid_id = 0U;
    VisionRecognitionDistance distance = VISION_RECOGNITION_DISTANCE_ONE_GRID;

    if (target == NULL)
    {
        return 1U;
    }
    if (target->recog_distance == VISION_RECOGNITION_DISTANCE_TWO_GRID ||
        target->recog_distance == VISION_RECOGNITION_DISTANCE_ONE_GRID)
    {
        distance = target->recog_distance;
    }

    /* ʶ�����ڼ䱣��ͣ��������ͼ�񶶶��� */
    car_go_flag = 1U;
    car_stop_flag = 1U;

    if (!g_identify_recog_waiting)
    {
        /*
         * 每次开始新的识别请求前都清一次视觉识别缓存。
         * 原因：NUM/IMG 返回是异步串口行，如果上一轮迟到的失败帧还挂在 updated 标志上，
         * 首个识别点会立刻吃到旧的失败结果，从而被误判为 Mode1。
         */
        if (!start_identify_recognition_request(target, distance))
        {
            return 1U;
        }
        return 0U;
    }

    if (g_identify_recog_wait_type == VISION_RECOGNITION_IMG)
    {
        if (vision_take_img_result(&result))
        {
            valid_id = identify_result_to_valid_id(&result, &recognized_id);
            if (!update_plan_mode_from_first_identify_result(&result, valid_id))
            {
                if (!start_identify_recognition_request(target, distance))
                {
                    return 1U;
                }
                return 0U;
            }

            if (target->obj_type == CONTROL_IDENTIFY_OBJ_BOX &&
                target->obj_index < MAX_BOXES &&
                target->obj_index < Boxes_count &&
                valid_id)
            {
                boxes[target->obj_index].id = recognized_id;
                g_identify_box_id_assigned[target->obj_index] = 1U;
            }
            reset_identify_recognition_wait();
            return 1U;
        }
    }
    else if (g_identify_recog_wait_type == VISION_RECOGNITION_NUM)
    {
        if (vision_take_num_result(&result))
        {
            valid_id = identify_result_to_valid_id(&result, &recognized_id);
            if (!update_plan_mode_from_first_identify_result(&result, valid_id))
            {
                if (!start_identify_recognition_request(target, distance))
                {
                    return 1U;
                }
                return 0U;
            }

            if (target->obj_type == CONTROL_IDENTIFY_OBJ_TARGET &&
                target->obj_index < MAX_TARGETS &&
                target->obj_index < Targets_count &&
                valid_id)
            {
                targets[target->obj_index].id = recognized_id;
                g_identify_target_id_assigned[target->obj_index] = 1U;
            }
            reset_identify_recognition_wait();
            return 1U;
        }
    }

    g_identify_recog_wait_loops++;
    if (g_identify_recog_wait_loops >= CONTROL_IDENTIFY_RECOG_TIMEOUT_LOOPS)
    {
        if (!g_identify_first_result_checked)
        {
            /*
             * 首个 IMG/NUM 决定后续 Mode1/Mode2，不能用“主循环轮询超时”直接判无效。
             * control_process() 没有固定调用周期，250 次循环可能远小于一次 OpenMV 识别耗时；
             * 超时这里只重发请求继续等，真正收到非 0~9 结果时才切 Mode1。
             */
            if (!start_identify_recognition_request(target, distance))
            {
                return 1U;
            }
            return 0U;
        }
        reset_identify_recognition_wait();
        return 1U;
    }

    return 0U;
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

static uint8 identify_cell_has_blocker(uint8 row, uint8 col)
{
    size_t i = 0U;

    for (i = 0U; i < Obstacles_count; i++)
    {
        if (obstacles[i].row == row && obstacles[i].col == col)
            return 1U;
    }
    for (i = 0U; i < Bombs_count; i++)
    {
        if (bombs[i].row == row && bombs[i].col == col)
            return 1U;
    }
    for (i = 0U; i < Boxes_count; i++)
    {
        if (boxes[i].row == row && boxes[i].col == col)
            return 1U;
    }
    for (i = 0U; i < Targets_count; i++)
    {
        if (targets[i].row == row && targets[i].col == col)
            return 1U;
    }
    return 0U;
}

static uint8 identify_target_has_clear_view(const Position *map_point,
                                            const Position *object_pos)
{
    int32 d_row = 0;
    int32 d_col = 0;
    int32 abs_row = 0;
    int32 abs_col = 0;
    int32 manhattan = 0;
    Position mid = {0};

    if (map_point == NULL || object_pos == NULL)
        return 0U;

    d_row = (int32)object_pos->row - (int32)map_point->row;
    d_col = (int32)object_pos->col - (int32)map_point->col;
    abs_row = (d_row >= 0) ? d_row : -d_row;
    abs_col = (d_col >= 0) ? d_col : -d_col;
    manhattan = abs_row + abs_col;

    if ((manhattan != 1 && manhattan != 2) ||
        (d_row != 0 && d_col != 0))
    {
        return 0U;
    }
    if (manhattan == 1)
        return 1U;

    mid.row = (uint8)((int32)map_point->row + ((d_row > 0) ? 1 : ((d_row < 0) ? -1 : 0)));
    mid.col = (uint8)((int32)map_point->col + ((d_col > 0) ? 1 : ((d_col < 0) ? -1 : 0)));

    return identify_cell_has_blocker(mid.row, mid.col) ? 0U : 1U;
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
        if ((manhattan != 1 && manhattan != 2) ||
            (d_row != 0 && d_col != 0))
        {
            continue;
        }
        if (!identify_target_has_clear_view(&map_point, &boxes[i]))
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
        g_identify_targets[g_identify_target_count].recog_distance =
            (manhattan == 2) ? VISION_RECOGNITION_DISTANCE_TWO_GRID :
                               VISION_RECOGNITION_DISTANCE_ONE_GRID;
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
        if ((manhattan != 1 && manhattan != 2) ||
            (d_row != 0 && d_col != 0))
        {
            continue;
        }
        if (!identify_target_has_clear_view(&map_point, &targets[i]))
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
        g_identify_targets[g_identify_target_count].recog_distance =
            (manhattan == 2) ? VISION_RECOGNITION_DISTANCE_TWO_GRID :
                               VISION_RECOGNITION_DISTANCE_ONE_GRID;
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
        if (is_identification_marker(g_exec_path[i].id))
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

static uint8 begin_identify_segment_motion(size_t end_idx)
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

        configure_bomb_pause_for_path(g_identify_segment_path, seg_steps);
        path_follow_hold_current_yaw();
        path_follow_set_path(g_identify_segment_path, seg_steps);
        g_identify_segment_running = 1U;
    }

    car_go_flag = 1U;
    car_stop_flag = 0U;
    g_identify_exec_state = CONTROL_IDENTIFY_EXEC_MOVE_SEGMENT;
    return 1U;
}

static uint8 start_identify_segment(size_t end_idx)
{
    if (end_idx >= g_exec_steps || end_idx < g_identify_segment_start_idx)
    {
        return 0U;
    }

    g_identify_rotate_started = 0U;
    memset(g_identify_targets, 0, sizeof(g_identify_targets));
    g_identify_target_count = 0U;
    g_identify_target_cursor = 0U;

    /*
     * 识别短段开始前先查看该段终点旁边要识别的物体。
     * 若存在待识别目标，先在短段起点转到识别朝向，再保持该朝向行驶到识别位。
     */
    if (g_control_identify_prerotate_enabled &&
        g_identify_endpoint_cursor < g_identify_endpoint_count &&
        g_identify_endpoint_need_action[g_identify_endpoint_cursor] &&
        collect_identify_targets_on_exec_point(&g_exec_path[end_idx]))
    {
        car_go_flag = 1U;
        car_stop_flag = 0U;
        g_identify_exec_state = CONTROL_IDENTIFY_EXEC_ROTATE_BEFORE_SEGMENT;
        return 1U;
    }

    return begin_identify_segment_motion(end_idx);
}

static void finish_identify_flow_and_relocalize(uint8 finalize_ids)
{
    car_go_flag = 0U;
    car_stop_flag = 0U;
    path_follow_set_path(NULL, 0U);
    path_follow_set_pause_indices(NULL, 0U, 0U);

    /* 正常完成识别时保留并补齐 ID；Mode1 快速退出时不需要这些配对信息。 */
    if (finalize_ids)
    {
        finalize_identify_ids_for_pushbox();
    }
    vision_clear_pending_data();

    g_control_flow_phase = CONTROL_FLOW_PUSHBOX;
    g_plan_ready = 0U;
    reset_identify_runtime_state();
    clear_map_request_wait();
    clear_car_request_wait();
    g_wait_new_map_frame = 0U;
    g_wait_map_frame_base = 0U;
    map_data_updated = false;
    car_pose_updated = false;
    reset_localization_accumulator();

    if (should_enter_vision_localization_stage())
    {
        g_relocalize_force_fresh_pose = 1U;
        set_control_phase_stage(CONTROL_PHASE_STEP_LOCALIZE);
    }
    else
    {
        if (!g_map_right_yaw_ready)
        {
            g_map_right_yaw_deg = eulerAngle.yaw;
            g_map_right_yaw_ready = 1U;
        }
        g_relocalize_force_fresh_pose = 0U;
        mark_wait_new_map_frame();
        set_control_phase_stage(CONTROL_PHASE_STEP_WAIT_CAMERA_DATA);
    }
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
        finish_identify_flow_and_relocalize(1U);
        return 1U;
    }

    return start_identify_segment(g_identify_endpoint_indices[g_identify_endpoint_cursor]);
}

/**
 * @brief 为当前执行路径配置“炸弹爆炸点停留”事件。
 *
 * 逻辑：
 * - 扫描路径中 id == BOMB_EXPLOSION 的点；
 * - 在这些点到达后暂停固定时长（0.5s）；
 * - 若当前路径无爆炸点，则清空暂停配置。
 *
 * @param path 路径点数组。
 * @param steps 路径长度。
 */
static void configure_bomb_pause_for_path(const Position *path, size_t steps)
{
    size_t pause_indices[PATH_FOLLOW_MAX_PAUSE_POINTS] = {0U};
    size_t pause_count = 0U;
    size_t i = 0U;

    if (path == NULL || steps == 0U)
    {
        path_follow_set_pause_indices(NULL, 0U, 0U);
        return;
    }

    for (i = 0U; i < steps; i++)
    {
        if (path[i].id != BOMB_EXPLOSION)
        {
            continue;
        }
        if (pause_count >= PATH_FOLLOW_MAX_PAUSE_POINTS)
        {
            break;
        }
        pause_indices[pause_count++] = i;
    }

    if (pause_count > 0U)
    {
        path_follow_set_pause_indices(pause_indices,
                                      pause_count,
                                      CONTROL_BOMB_EXPLOSION_PAUSE_MS);
    }
    else
    {
        path_follow_set_pause_indices(NULL, 0U, 0U);
    }
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

static void settle_camera_before_localization_once(void)
{
    if (g_localize_camera_settled)
    {
        return;
    }

    /* Wait once at the start of a localization phase, then parse the latest CAR frame. */
    system_delay_ms(CONTROL_CAMERA_POSE_SETTLE_DELAY_MS);
    process_blob_data();
    g_localize_camera_settled = 1U;
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
static void snapshot_take(path_map_snapshot_t *snap)
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
static void snapshot_restore(const path_map_snapshot_t *snap)
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
    path_map_snapshot_t map_snapshot;
    path_map_snapshot_t post_plan_snapshot;
    uint8 build_ok = 0U;

    g_plan_ready = 0U;
    memset(&map_snapshot, 0, sizeof(map_snapshot));
    memset(&post_plan_snapshot, 0, sizeof(post_plan_snapshot));
    snapshot_take(&map_snapshot);

    /* 根据 planmode 标志位选择规划方案，不做自动模式回退�?*/
    if (g_control_flow_phase == CONTROL_FLOW_IDENTIFY)
    {
        clear_saved_identify_ids();
        reset_identify_runtime_state();
        vision_clear_pending_data();
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

    /* 规划函数会在全局地图里模拟动作后的结果；map_snapshot 保存规划前地图。
     * path.c 构建执行路径时同时参考“规划后地图”和“规划前地图”，
     * 让捷径同时避开动作前后的对象位置。构建完成后再恢复真实运行时地图。 */
    snapshot_take(&post_plan_snapshot);
    build_ok = path_build_exec_from_planner(car_path,
                                            Car_path_count,
                                            &post_plan_snapshot,
                                            &map_snapshot,
                                            g_exec_path,
                                            MAX_CAR_PATH,
                                            &g_exec_steps);
    snapshot_restore(&map_snapshot);
    if (!build_ok)
    {
        return 0U;
    }

    g_plan_ready = 1U;
    return 1U;
}

/**
 * @brief 处理“起步出发车区”阶段。
 *
 * 行为说明：
 * - 首次进入时，读取当前 IMU 航向角作为车头保持方向；
 * - 根据 control_set_prestart_depart_dir() 设置的编号选择地图右/上/左/下发车方向；
 * - 小车按所选方向平移一个网格距离，车头朝向不变；
 * - 动作执行完成后，切到初始定位阶段。
 */
static void handle_prestart_move(void)
{
    path_follow_status_t st = {0};
    control_map_dir_t prestart_dir = CONTROL_MAP_DIR_RIGHT;
    float hold_yaw_deg = 0.0f;
    float move_yaw_deg = 0.0f;
    float move_yaw_rad = 0.0f;
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
        hold_yaw_deg = eulerAngle.yaw;
        prestart_dir = get_prestart_depart_map_dir();
        move_yaw_deg = map_dir_to_yaw_deg(prestart_dir);
        move_yaw_rad = move_yaw_deg * CONTROL_DEG_TO_RAD;
        start_x_m = st.x_m;
        start_y_m = st.y_m;
        delta_x_m = cosf(move_yaw_rad) * CONTROL_PRESTART_OFFSET_M;
        delta_y_m = sinf(move_yaw_rad) * CONTROL_PRESTART_OFFSET_M;

        /* 防止起步目标越界（负坐标�?uint8 会变 255，导致远距离猛冲）�?*/
        target_x_m = clampf_local(start_x_m + delta_x_m, 0.0f, CONTROL_WORLD_X_MAX_M);
        target_y_m = clampf_local(start_y_m + delta_y_m, 0.0f, CONTROL_WORLD_Y_MAX_M);

        /* 发车只改变平移方向，不改变车头朝向。 */
        path_follow_reset_pose(st.x_m, st.y_m, hold_yaw_deg);
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
        reset_localization_accumulator();
        g_relocalize_force_fresh_pose = 0U;
        if (should_enter_vision_localization_stage())
        {
            set_control_phase_stage(CONTROL_PHASE_STEP_LOCALIZE);
        }
        else
        {
            if (!g_map_right_yaw_ready)
            {
                g_map_right_yaw_deg = eulerAngle.yaw;
                g_map_right_yaw_ready = 1U;
            }
            g_body_map_dir = CONTROL_MAP_DIR_RIGHT;
            mark_wait_new_map_frame();
            set_control_phase_stage(CONTROL_PHASE_STEP_WAIT_CAMERA_DATA);
        }
    }
}

/**
 * @brief 处理“初始定位”阶段�?
 *
 * 核心逻辑�?
 * - 按需请求视觉车位�?
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

    if (!should_enter_vision_localization_stage())
    {
        /* 后续视觉定位关闭时，推箱子阶段重定位直接降级为“等地图->规划”。 */
        if (!g_map_right_yaw_ready)
        {
            g_map_right_yaw_deg = eulerAngle.yaw;
            g_map_right_yaw_ready = 1U;
        }
        if (g_control_flow_phase == CONTROL_FLOW_IDENTIFY)
        {
            g_body_map_dir = CONTROL_MAP_DIR_RIGHT;
        }
        g_relocalize_force_fresh_pose = 0U;
        clear_car_request_wait();
        mark_wait_new_map_frame();
        set_control_phase_stage(CONTROL_PHASE_STEP_WAIT_CAMERA_DATA);
        return;
    }

    if (g_control_flow_phase == CONTROL_FLOW_PUSHBOX)
    {
        min_samples = CONTROL_RELOCALIZE_MIN_SAMPLES_PUSHBOX;
    }

    settle_camera_before_localization_once();

    if (car_pose_updated)
    {
        car_pose_updated = false;
        clear_car_request_wait();
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
        service_car_request_wait();
        return;
    }
    if (!get_camera_pose_meter(&cam_x_m, &cam_y_m, &cam_yaw_deg))
    {
        service_car_request_wait();
        return;
    }

    g_localize_sum_x_m += cam_x_m;
    g_localize_sum_y_m += cam_y_m;
    g_localize_sum_yaw_deg += cam_yaw_deg;
    g_localize_sample_count++;

    if (g_localize_sample_count < min_samples)
    {
        send_car_request_once();
        return;
    }

    clear_car_request_wait();
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
    set_control_phase_stage(CONTROL_PHASE_STEP_WAIT_CAMERA_DATA);
}

/**
 * @brief 处理“等待地图”阶段�?
 *
 * 阶段目标�?
 * - 按需请求地图，并顺带消费可能到达的车位姿
 * - 收到有效地图后切换到规划阶段
 */
static void handle_wait_camera_data(void)
{
    float cam_x_m = 0.0f;
    float cam_y_m = 0.0f;
    float cam_yaw_deg = 0.0f;

    if (g_control_use_followup_vision_localization)
    {
        if (car_pose_updated)
        {
            car_pose_updated = false;
            if (get_camera_pose_meter(&cam_x_m, &cam_y_m, &cam_yaw_deg))
            {
                /* 等地图阶段若拿到新位姿，顺带轻量同步一次里程计。 */
                path_follow_reset_pose(cam_x_m, cam_y_m, cam_yaw_deg);
            }
        }
    }
    else
    {
        /* 后续视觉修正关闭时丢弃旧 CAR 更新标志，避免重新开启后误吃关闭期间的缓存帧。 */
        car_pose_updated = false;
    }

    if (map_data_ready && map_frame_count > 0U)
    {
        if (!g_wait_new_map_frame || map_frame_count != g_wait_map_frame_base)
        {
            g_wait_new_map_frame = 0U;
            if (g_control_flow_phase == CONTROL_FLOW_PUSHBOX)
            {
                /* �ض�λ���õ��µ�ͼʱ����ʶ��׶εõ��� ID ��д����ǰ objects�� */
                apply_saved_identify_ids_to_current_map();
            }
            map_data_updated = false;
            clear_map_request_wait();
            set_control_phase_stage(CONTROL_PHASE_STEP_PLAN_PATH);
            return;
        }
    }

    service_map_request_wait();
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

static void finish_identify_rotation(control_map_dir_t face_dir)
{
    g_identify_rotate_started = 0U;
    g_body_map_dir = face_dir;
    reset_identify_recognition_wait();
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
        finish_identify_flow_and_relocalize(1U);
        return;
    }

    switch (g_identify_exec_state)
    {
    case CONTROL_IDENTIFY_EXEC_ROTATE_BEFORE_SEGMENT:
        if (g_identify_endpoint_cursor >= g_identify_endpoint_count)
        {
            g_control_stage = CONTROL_STAGE_ERROR;
            return;
        }

        if (g_identify_target_cursor >= g_identify_target_count)
        {
            if (!begin_identify_segment_motion(g_identify_endpoint_indices[g_identify_endpoint_cursor]))
            {
                g_control_stage = CONTROL_STAGE_ERROR;
            }
            return;
        }

        curr_target = &g_identify_targets[g_identify_target_cursor];
        target_yaw_deg = map_dir_to_yaw_deg(curr_target->face_dir);
        yaw_err_deg = wrap_yaw_deg_local(target_yaw_deg - eulerAngle.yaw);
        if (fabsf(yaw_err_deg) <= CONTROL_IDENTIFY_YAW_ALIGN_TOL_DEG)
        {
            finish_identify_rotation(curr_target->face_dir);
            if (!begin_identify_segment_motion(g_identify_endpoint_indices[g_identify_endpoint_cursor]))
            {
                g_control_stage = CONTROL_STAGE_ERROR;
            }
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

        finish_identify_rotation(curr_target->face_dir);
        if (!begin_identify_segment_motion(g_identify_endpoint_indices[g_identify_endpoint_cursor]))
        {
            g_control_stage = CONTROL_STAGE_ERROR;
        }
        break;

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
            if (g_identify_target_count > 0U)
            {
                g_identify_exec_state = CONTROL_IDENTIFY_EXEC_DO_RECOGNIZE;
                return;
            }

            if (collect_identify_targets_on_exec_point(&g_exec_path[endpoint_idx]))
            {
                /* 兜底：若段前未拿到目标，则到点后仍按旧逻辑原地转向识别。 */
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
            finish_identify_rotation(curr_target->face_dir);
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

        finish_identify_rotation(curr_target->face_dir);
        g_identify_exec_state = CONTROL_IDENTIFY_EXEC_DO_RECOGNIZE;
        break;

    case CONTROL_IDENTIFY_EXEC_DO_RECOGNIZE:
        if (g_identify_target_cursor < g_identify_target_count)
        {
            curr_target = &g_identify_targets[g_identify_target_cursor];
            if (!identify_action_stub(curr_target))
            {
                return;
            }

            if (g_identify_abort_to_mode1)
            {
                finish_identify_flow_and_relocalize(0U);
                return;
            }

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
        /* 不再强制回正到“地图右向”，保持当前车头朝向直接进入下一段�?*/
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

/**
 * @brief 规划阶段：暂停执行层，调用规划器，并把结果转换成执行路径。
 *
 * 这里集中处理“进入规划保护期/退出规划保护期”，让 control_process() 只表达
 * 状态机的主干，不展开具体细节。
 */
static void handle_plan_path_stage(void)
{
    begin_path_plan_pause();
    if (control_plan_path())
    {
        set_control_phase_stage(CONTROL_PHASE_STEP_LOAD_PATH);
    }
    else
    {
        g_control_stage = CONTROL_STAGE_ERROR;
    }
    end_path_plan_pause();
}

/**
 * @brief 下发阶段：根据当前大流程，把识别路径或推箱路径装载到 path_follow。
 *
 * 识别阶段会把完整识别路径切成若干段，每段起点先转向，再到识别点直接识别；
 * 推箱阶段则直接下发完整执行路径，并配置炸弹爆炸点停留事件。
 */
static void handle_load_path_stage(void)
{
    if (!g_plan_ready || g_exec_steps < 2U)
    {
        g_control_stage = CONTROL_STAGE_ERROR;
        return;
    }

    if (g_control_flow_phase == CONTROL_FLOW_IDENTIFY)
    {
        if (!load_identify_path_for_execution())
        {
            g_control_stage = CONTROL_STAGE_ERROR;
            return;
        }
        set_control_phase_stage(CONTROL_PHASE_STEP_EXECUTE_PATH);
        return;
    }

    configure_bomb_pause_for_path(g_exec_path, g_exec_steps);
    path_follow_hold_current_yaw();
    path_follow_set_path(g_exec_path, g_exec_steps);

    car_go_flag = 1U;
    car_stop_flag = 0U;
    set_control_phase_stage(CONTROL_PHASE_STEP_EXECUTE_PATH);
}

/**
 * @brief 推箱执行阶段：轮询 path_follow 状态，路径结束后进入完成态并保持停车。
 */
static void handle_pushbox_execute_path(void)
{
    path_follow_status_t st = {0};

    path_follow_get_status(&st);
    if (!st.active)
    {
        car_go_flag = 1U;
        car_stop_flag = 1U;
        set_control_phase_stage(CONTROL_PHASE_STEP_FINISHED);
    }
}

/**
 * @brief 执行阶段总入口：识别阶段和推箱阶段的执行模型不同，在这里分流。
 */
static void handle_execute_path_stage(void)
{
    if (g_control_flow_phase == CONTROL_FLOW_IDENTIFY)
    {
        handle_identify_execute_path();
        return;
    }

    handle_pushbox_execute_path();
}

/**
 * @brief 错误阶段：安全停车，并等待新地图后重新尝试规划。
 *
 * 只按需请求新地图；车位姿由定位阶段单独获取。
 */
static void handle_error_stage(void)
{
    car_go_flag = 1U;
    car_stop_flag = 1U;

    if (map_data_updated)
    {
        if (g_control_flow_phase == CONTROL_FLOW_PUSHBOX)
        {
            apply_saved_identify_ids_to_current_map();
        }
        map_data_updated = false;
        clear_map_request_wait();
        set_control_phase_stage(CONTROL_PHASE_STEP_PLAN_PATH);
        return;
    }

    service_map_request_wait();
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

void control_set_prestart_depart_dir(uint8 dir)
{
    if (dir > CONTROL_PRESTART_DEPART_DIR_MAX)
    {
        dir = CONTROL_PRESTART_DEPART_DIR_MAX;
    }
    g_control_prestart_depart_dir = dir;
}

uint8 control_get_prestart_depart_dir(void)
{
    return g_control_prestart_depart_dir;
}

void control_process(void)
{
    /*
     * 控制主循环只做两件事：
     * 1) 先消费串口 FIFO，保证地图、车位姿、识别结果缓存尽量新；
     * 2) 再按当前阶段调用对应 handler，handler 内部只负责推进一个阶段。
     *
     * 这样主状态机保留“流程骨架”，阶段细节放在命名函数里，后续调车时更容易定位问题。
     */
    process_blob_data();
    process_vision_data();

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

    case CONTROL_STAGE_IDENTIFY_LOCALIZE:
    case CONTROL_STAGE_PUSHBOX_LOCALIZE:
        handle_startup_localization();
        break;

    case CONTROL_STAGE_IDENTIFY_WAIT_CAMERA_DATA:
    case CONTROL_STAGE_PUSHBOX_WAIT_CAMERA_DATA:
        handle_wait_camera_data();
        break;

    case CONTROL_STAGE_IDENTIFY_PLAN_PATH:
    case CONTROL_STAGE_PUSHBOX_PLAN_PATH:
        handle_plan_path_stage();
        break;

    case CONTROL_STAGE_IDENTIFY_LOAD_PATH:
    case CONTROL_STAGE_PUSHBOX_LOAD_PATH:
        handle_load_path_stage();
        break;

    case CONTROL_STAGE_IDENTIFY_EXECUTE_PATH:
    case CONTROL_STAGE_PUSHBOX_EXECUTE_PATH:
        handle_execute_path_stage();
        break;

    case CONTROL_STAGE_IDENTIFY_FINISHED:
    case CONTROL_STAGE_PUSHBOX_FINISHED:
        /* 完成态保持停车，等待外部调用 control_restart()。 */
        break;

    case CONTROL_STAGE_ERROR:
        handle_error_stage();
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
    if (steps != NULL)
    {
        *steps = g_exec_steps;
    }
    return g_exec_path;
}

void control_set_diagonal_path_enabled(uint8 enabled)
{
    path_set_diagonal_enabled(enabled);
}

uint8 control_get_diagonal_path_enabled(void)
{
    return path_get_diagonal_enabled();
}

void control_set_followup_vision_localization_enabled(uint8 enabled)
{
    g_control_use_followup_vision_localization = (enabled != 0U) ? 1U : 0U;
}

uint8 control_get_followup_vision_localization_enabled(void)
{
    return g_control_use_followup_vision_localization;
}

/**
 * @brief 手动设置推箱阶段规划模式。
 *
 * 为了避免非法输入破坏流程，除 CONTROL_PLAN_MODE_1 外都按
 * CONTROL_PLAN_MODE_2 处理；识别阶段的首个 IMG/NUM 结果仍可能自动覆盖该值。
 */
void control_set_plan_mode(control_plan_mode_t mode)
{
    if (mode == CONTROL_PLAN_MODE_1)
    {
        g_control_plan_mode = CONTROL_PLAN_MODE_1;
    }
    else
    {
        /* ʶ�����̶̹��������Զ�ִ�У��˴�����������׶�?Mode1/Mode2�� */
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
