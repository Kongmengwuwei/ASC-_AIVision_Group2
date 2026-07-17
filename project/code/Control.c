#include "Control.h"
#include "Algorithm_Test.h"
#include "data_handle.h"
#include "Game_logic.h"
#include "Map_Path_Data.h"
#include "Mymenu.h"
#include "path.h"
#include "path_follow.h"
#include "Attitude.h"
#include "BlueSerial.h"
#include "Motor.h"
#include "zf_driver_delay.h"
#include <math.h>
#include <string.h>

/*
 * 手动选择起步发车方向。
 * 0：起步临时右，对应车前方；
 * 1：起步临时上，对应车左方；
 * 2：起步临时左，对应车后方；
 * 3：起步临时下，对应车右方；
 * 4：保留/默认，按起步临时右处理。
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
static uint8 g_control_continuous_levels_enabled = 0U;
/* Risky ID repair is opt-in. Normal recognition results are not rewritten by default. */
static uint8 g_control_identify_id_fallback_enabled = 0U;
/* 每个识别驻车点、每次推箱结束后的视觉位置校正，默认关闭以保持原流程。 */
static uint8 g_control_checkpoint_vision_localization_enabled = 0U;

/*
 * Extra compensation for the first power-on departure move.
 * Positive values extend the selected Start_Dir move, negative values shorten it.
 * Unit: m. Default +0.05 m keeps the prestart move at 0.35 m total.
 */
float g_control_prestart_depart_compensate_m = -0.025f;

/* ========================= 参数配置�?========================= */

/*
 * MAP/CAR requests are sent on demand. These 10 ms tick guards only retry
 * when a request is already pending and no matching response has arrived.
 */
#define CONTROL_REQ_MAP_RETRY_TIMEOUT_TICKS 500U
#define CONTROL_REQ_CAR_RETRY_TIMEOUT_TICKS 200U
#define CONTROL_LOCALIZE_STOP_STABLE_TICKS 20U
#define CONTROL_LOCALIZE_POST_STOP_DRAIN_TICKS 20U
#define CONTROL_LOCALIZE_WHEEL_STOP_ENCODER_TOL 5
#define CONTROL_LOCALIZE_MAX_SAMPLES 5U
#define CONTROL_LOCALIZE_TWO_SAMPLE_MATCH_M 0.03f
#define CONTROL_PRESET_RECOGNITION_DELAY_MS 500U
/* 推炸弹到爆炸点后停留 0.5s，等待爆炸效果生效 */
#define CONTROL_BOMB_EXPLOSION_PAUSE_MS 500U
/* 回到发车区后，车头回正到发车方向基准的角度容差。 */
#define CONTROL_RETURN_YAW_ALIGN_TOL_DEG 5.0f
#define CONTROL_CONTINUOUS_LEVEL_COUNT 3U
#define CONTROL_CONTINUOUS_STOP_ENCODER_TOL 5
#define CONTROL_CONTINUOUS_STOP_STABLE_LOOPS 3U


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
 * 这是实车调车参数，用于让车身离开发车区；不要求等于地图格长。
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
 * - 视觉位置映射同步采用 (x=col, y=row)
 */
#define CONTROL_COORD_TRANSPOSE_COMPENSATE PATH_COORD_TRANSPOSE_COMPENSATE
/* 在转置补偿基础上，额外翻转“上下轴”（对应 map �?row 方向）�?*/
#define CONTROL_COORD_FLIP_VERTICAL PATH_COORD_FLIP_VERTICAL
#define CONTROL_WORLD_X_MAX_M PATH_WORLD_X_MAX_M
#define CONTROL_WORLD_Y_MAX_M PATH_WORLD_Y_MAX_M
/* ========================= 内部数据结构 ========================= */

#define CONTROL_IDENTIFY_MAX_TARGETS_PER_POINT (MAX_BOXES + MAX_TARGETS)
#define CONTROL_IDENTIFY_YAW_ALIGN_TOL_DEG 5.0f
#define CONTROL_IDENTIFY_RECOG_SETTLE_TICKS 20U
#define CONTROL_IDENTIFY_RECOG_TIMEOUT_TICKS 80U
#define CONTROL_IDENTIFY_RECOG_ACCEPT_SCORE 85
#define CONTROL_IDENTIFY_RECOG_CONFIRM_SCORE 70
/* 连续超时多少次之后才真正重发识别命令（避免在摄像头处理过程中频繁清 FIFO 重发）。 */
#define CONTROL_IDENTIFY_RECOG_MAX_RETRIES 1U
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
    Position stand_pos_map;
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
    CONTROL_IDENTIFY_EXEC_ADVANCE_ENDPOINT
} control_identify_exec_state_t;

typedef struct
{
    uint8 need_identify;
    control_plan_mode_t push_mode;
} control_level_rule_t;

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
 * - 连续发车关卡一不识别，直接使用 Mode1；
 * - 连续发车关卡二/三识别后使用 Mode2；
 * - 单次发车识别后使用 Mode2；
 * - 仍保留 control_set_plan_mode() 作为外部手动覆盖入口。
 */
static control_plan_mode_t g_control_plan_mode = CONTROL_PLAN_MODE_1;
static control_flow_phase_t g_control_flow_phase = CONTROL_FLOW_IDENTIFY;
static uint8 g_continuous_run_active = 0U;
static uint8 g_continuous_level_index = 0U;
static uint8 g_continuous_stop_stable_count = 0U;
static uint8 g_level_start_localization_required = 0U;
static uint8 g_continuous_preset_base_index = 0U;
static uint8 g_return_heading_rotate_started = 0U;
static uint8 g_pushbox_entry_heading_rotate_started = 0U;
static float g_start_yaw_deg = 0.0f;
static uint8 g_start_yaw_ready = 0U;

static uint8 g_wait_new_map_frame = 0U;
static uint8 g_wait_map_frame_base = 0U;
static volatile uint32 g_control_tick_10ms = 0U;

static size_t g_identify_segment_start_idx = 0U;
static size_t g_identify_endpoint_indices[MAX_CAR_PATH] = {0U};
static uint8 g_identify_endpoint_need_action[MAX_CAR_PATH] = {0U};
static size_t g_identify_endpoint_count = 0U;
static size_t g_identify_endpoint_cursor = 0U;
static Position g_identify_segment_path[MAX_CAR_PATH] = {{0}};
static uint8 g_identify_segment_running = 0U;
static uint8 g_identify_segment_map_event_applied = 0U;
static uint8 g_identify_safe_move_prepared = 0U;
static uint8 g_identify_safe_move_running = 0U;
static uint8 g_identify_return_start_valid = 0U;
static Position g_identify_return_start_map = {0U, 0U, 0U};
static Position g_identify_safe_move_path[PATH_SAFE_RELOCATION_MAX_POINTS] = {{0}};
static size_t g_identify_safe_move_steps = 0U;

static control_identify_target_t g_identify_targets[CONTROL_IDENTIFY_MAX_TARGETS_PER_POINT];
static size_t g_identify_target_count = 0U;
static size_t g_identify_target_cursor = 0U;
static uint8 g_identify_rotate_started = 0U;
static uint8 g_identify_checkpoint_localized = 0U;
static uint8 g_identify_targets_collected_at_endpoint = 0U;
static control_identify_exec_state_t g_identify_exec_state = CONTROL_IDENTIFY_EXEC_IDLE;

/* 推箱阶段按 PUSH_END_POINT 分段，段间可插入视觉定位。 */
static Position g_pushbox_segment_path[MAX_CAR_PATH] = {{0}};
static size_t g_pushbox_segment_end_idx = 0U;
static uint8 g_pushbox_segment_running = 0U;
static uint8 g_pushbox_checkpoint_localized = 0U;

/* 识别和推箱检查点共用的一次性视觉定位运行状态。 */
static uint8 g_checkpoint_visual_localization_active = 0U;

static uint8 g_identified_box_flags[MAX_BOXES] = {0U};
static uint8 g_identified_target_flags[MAX_TARGETS] = {0U};
static uint8 g_identify_box_id_assigned[MAX_BOXES] = {0U};
static uint8 g_identify_target_id_assigned[MAX_TARGETS] = {0U};
static int16 g_identify_box_confidence[MAX_BOXES] = {0};
static int16 g_identify_target_confidence[MAX_TARGETS] = {0};
static uint8 g_identify_recog_waiting = 0U;
static uint8 g_identify_recog_retry_count = 0U;
static VisionRecognitionType g_identify_recog_wait_type = VISION_RECOGNITION_NONE;
static uint16 g_identify_recog_expected_sequence = 0U;
static uint32 g_identify_recog_wait_start_tick = 0U;
static uint8 g_identify_recog_settle_waiting = 0U;
static uint32 g_identify_recog_settle_start_tick = 0U;
static uint8 g_identify_recog_confirm_pending = 0U;
static uint8 g_identify_recog_confirm_id = 0U;
static int16 g_identify_recog_confirm_score = -1;

static control_identify_id_record_t g_saved_box_id_records[MAX_BOXES] = {{0}};
static control_identify_id_record_t g_saved_target_id_records[MAX_TARGETS] = {{0}};
static uint8 g_saved_identify_ids_ready = 0U;

/**
 * @brief 初始定位累计样本数�?
 */
static uint8 g_localize_sample_count = 0U;
static uint8 g_localize_camera_settled = 0U;
static uint8 g_localize_stop_sequence_started = 0U;
static uint8 g_localize_fresh_pose_request_started = 0U;
static uint8 g_localize_car_frame_base = 0U;
static uint8 g_localize_map_prefetch_started = 0U;
static uint8 g_localize_map_frame_base = 0U;
static uint32 g_localize_stop_stable_start_tick = 0U;
static uint32 g_localize_post_stop_start_tick = 0U;

/**
 * @brief 起步动作是否已经触发�?
 *
 * - 0：尚未下发起步动�?
 * - 1：已下发，等待动作执行结�?
 */
static uint8 g_prestart_move_started = 0U;
static uint8 g_prestart_nominal_pose_valid = 0U;
static float g_prestart_nominal_target_x_m = 0.0f;
static float g_prestart_nominal_target_y_m = 0.0f;

/**
 * @brief 初始定位阶段累计的视�?X 坐标和（米）�?
 */
static float g_localize_samples_x_m[CONTROL_LOCALIZE_MAX_SAMPLES] = {0.0f};

/**
 * @brief 初始定位阶段累计的视�?Y 坐标和（米）�?
 */
static float g_localize_samples_y_m[CONTROL_LOCALIZE_MAX_SAMPLES] = {0.0f};

static uint8 g_map_request_waiting = 0U;
static uint32 g_map_request_wait_start_tick = 0U;

static uint8 g_car_request_waiting = 0U;
static uint32 g_car_request_wait_start_tick = 0U;

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

static void control_enable_yaw_closed_loop(void)
{
    path_follow_set_stationary_yaw_hold_enabled(1U);
}

static void control_disable_yaw_closed_loop(void)
{
    path_follow_set_stationary_yaw_hold_enabled(0U);
}

static void control_hold_yaw_closed_loop(void)
{
    path_follow_hold_current_yaw();
    control_enable_yaw_closed_loop();
    car_go_flag = 1U;
    car_stop_flag = 0U;
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
        return CONTROL_STAGE_ERROR;
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

static void reset_localization_accumulator(void)
{
    g_localize_sample_count = 0U;
    g_localize_camera_settled = 0U;
    g_localize_stop_sequence_started = 0U;
    g_localize_fresh_pose_request_started = 0U;
    g_localize_car_frame_base = 0U;
    g_localize_map_prefetch_started = 0U;
    g_localize_map_frame_base = 0U;
    g_localize_stop_stable_start_tick = 0U;
    g_localize_post_stop_start_tick = 0U;
    memset(g_localize_samples_x_m, 0, sizeof(g_localize_samples_x_m));
    memset(g_localize_samples_y_m, 0, sizeof(g_localize_samples_y_m));
}

static void clear_map_request_wait(void)
{
    g_map_request_waiting = 0U;
    g_map_request_wait_start_tick = 0U;
}

static void clear_car_request_wait(void)
{
    g_car_request_waiting = 0U;
    g_car_request_wait_start_tick = 0U;
}

static void send_map_request_once(void)
{
    if (Algorithm_Test_PresetInput_IsEnabled())
    {
        (void)Algorithm_Test_PresetInput_ProvideMapFrame();
        g_map_request_waiting = 0U;
        g_map_request_wait_start_tick = 0U;
        return;
    }
    uart_send_map_request();
    g_map_request_waiting = 1U;
    g_map_request_wait_start_tick = g_control_tick_10ms;
}

static void send_car_request_once(void)
{
    if (Algorithm_Test_PresetInput_IsEnabled())
    {
        (void)Algorithm_Test_PresetInput_ProvideCarPoseFrame();
        g_car_request_waiting = 0U;
        g_car_request_wait_start_tick = 0U;
        return;
    }
    uart_send_car_request();
    g_car_request_waiting = 1U;
    g_car_request_wait_start_tick = g_control_tick_10ms;
}

static void service_map_request_wait(void)
{
    if (!g_map_request_waiting)
    {
        send_map_request_once();
        return;
    }

    if ((uint32)(g_control_tick_10ms - g_map_request_wait_start_tick) >=
        CONTROL_REQ_MAP_RETRY_TIMEOUT_TICKS)
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

    if ((uint32)(g_control_tick_10ms - g_car_request_wait_start_tick) >=
        CONTROL_REQ_CAR_RETRY_TIMEOUT_TICKS)
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
    g_identify_recog_retry_count = 0U;
    g_identify_recog_wait_type = VISION_RECOGNITION_NONE;
    g_identify_recog_expected_sequence = 0U;
    g_identify_recog_wait_start_tick = 0U;
    g_identify_recog_settle_waiting = 0U;
    g_identify_recog_settle_start_tick = 0U;
    g_identify_recog_confirm_pending = 0U;
    g_identify_recog_confirm_id = 0U;
    g_identify_recog_confirm_score = -1;
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
    g_identify_segment_map_event_applied = 0U;

    memset(g_identify_targets, 0, sizeof(g_identify_targets));
    g_identify_target_count = 0U;
    g_identify_target_cursor = 0U;
    g_identify_rotate_started = 0U;
    g_identify_checkpoint_localized = 0U;
    g_identify_targets_collected_at_endpoint = 0U;
    g_identify_exec_state = CONTROL_IDENTIFY_EXEC_IDLE;
    reset_identify_recognition_wait();

    memset(g_identified_box_flags, 0, sizeof(g_identified_box_flags));
    memset(g_identified_target_flags, 0, sizeof(g_identified_target_flags));
    memset(g_identify_box_id_assigned, 0, sizeof(g_identify_box_id_assigned));
    memset(g_identify_target_id_assigned, 0, sizeof(g_identify_target_id_assigned));
    memset(g_identify_box_confidence, 0xFF, sizeof(g_identify_box_confidence));
    memset(g_identify_target_confidence, 0xFF, sizeof(g_identify_target_confidence));
}

static void inverse_remap_exec_path_point(Position *p)
{
    path_inverse_remap_exec_point(p);
}

static uint8 identify_get_exec_axis_direction(size_t from_idx,
                                              size_t to_idx,
                                              int32 *d_row,
                                              int32 *d_col)
{
    Position from;
    Position to;
    int32 raw_row = 0;
    int32 raw_col = 0;

    if (d_row == NULL || d_col == NULL ||
        from_idx >= g_exec_steps || to_idx >= g_exec_steps ||
        from_idx == to_idx)
    {
        return 0U;
    }

    from = g_exec_path[from_idx];
    to = g_exec_path[to_idx];
    inverse_remap_exec_path_point(&from);
    inverse_remap_exec_path_point(&to);
    raw_row = (int32)to.row - (int32)from.row;
    raw_col = (int32)to.col - (int32)from.col;
    if ((raw_row == 0 && raw_col == 0) ||
        (raw_row != 0 && raw_col != 0))
    {
        return 0U;
    }

    *d_row = (raw_row > 0) ? 1 : ((raw_row < 0) ? -1 : 0);
    *d_col = (raw_col > 0) ? 1 : ((raw_col < 0) ? -1 : 0);
    return 1U;
}

/**
 * @brief 在识别执行到达爆炸事件后推进真实运行时地图。
 *
 * 路径压缩保证推运区间不会走斜线；因此可由 PUSH_START 后的首段方向
 * 定位原炸弹，由爆炸端点前的末段方向定位最终爆心。只有实际到达事件点
 * 后才更新地图，避免提前把尚未炸毁的墙当成空地。
 */
static uint8 identify_apply_runtime_map_event(size_t endpoint_idx)
{
    size_t push_start_idx = 0U;
    size_t scan = 0U;
    size_t bomb_index = 0U;
    size_t read_index = 0U;
    size_t write_index = 0U;
    size_t old_obstacles_count = 0U;
    Position push_start;
    Position bomb_endpoint;
    int32 first_d_row = 0;
    int32 first_d_col = 0;
    int32 last_d_row = 0;
    int32 last_d_col = 0;
    int32 source_row = 0;
    int32 source_col = 0;
    int32 explode_row = 0;
    int32 explode_col = 0;
    uint8 found_push_start = 0U;
    uint8 found_bomb = 0U;

    if (endpoint_idx >= g_exec_steps)
    {
        return 0U;
    }
    if ((g_exec_path[endpoint_idx].id & BOMB_EXPLOSION) == 0U)
    {
        return 1U;
    }
    if ((g_exec_path[endpoint_idx].id & PUSH_END_POINT) == 0U ||
        endpoint_idx == 0U ||
        Bombs_count > MAX_BOMBS || Obstacles_count > MAX_OBSTACLES)
    {
        return 0U;
    }

    /* 从爆炸点向前找本次推炸弹区间的起点。 */
    scan = endpoint_idx;
    while (scan > 0U)
    {
        scan--;
        if ((g_exec_path[scan].id & PUSH_START_POINT) != 0U)
        {
            push_start_idx = scan;
            found_push_start = 1U;
            break;
        }
        if ((g_exec_path[scan].id & PUSH_END_POINT) != 0U)
        {
            break;
        }
    }
    if (!found_push_start || push_start_idx + 1U > endpoint_idx)
    {
        return 0U;
    }
    if (!identify_get_exec_axis_direction(push_start_idx,
                                          push_start_idx + 1U,
                                          &first_d_row,
                                          &first_d_col) ||
        !identify_get_exec_axis_direction(endpoint_idx - 1U,
                                          endpoint_idx,
                                          &last_d_row,
                                          &last_d_col))
    {
        return 0U;
    }

    push_start = g_exec_path[push_start_idx];
    bomb_endpoint = g_exec_path[endpoint_idx];
    inverse_remap_exec_path_point(&push_start);
    inverse_remap_exec_path_point(&bomb_endpoint);
    source_row = (int32)push_start.row + first_d_row;
    source_col = (int32)push_start.col + first_d_col;
    explode_row = (int32)bomb_endpoint.row + last_d_row;
    explode_col = (int32)bomb_endpoint.col + last_d_col;
    if (source_row < 0 || source_col < 0 ||
        source_row >= (int32)MAP_ROWS || source_col >= (int32)MAP_COLS ||
        explode_row < 0 || explode_col < 0 ||
        explode_row >= (int32)MAP_ROWS || explode_col >= (int32)MAP_COLS)
    {
        return 0U;
    }

    for (bomb_index = 0U; bomb_index < Bombs_count; bomb_index++)
    {
        if (bombs[bomb_index].row == (uint8)source_row &&
            bombs[bomb_index].col == (uint8)source_col)
        {
            found_bomb = 1U;
            break;
        }
    }
    if (!found_bomb)
    {
        return 0U;
    }
    for (read_index = bomb_index + 1U; read_index < Bombs_count; read_index++)
    {
        bombs[read_index - 1U] = bombs[read_index];
    }
    Bombs_count--;
    if (Bombs_count < MAX_BOMBS)
    {
        memset(&bombs[Bombs_count], 0, sizeof(Position));
    }

    /* 与规划层 simulate_bomb_explosion() 保持一致：清除爆心 3x3 墙体。 */
    old_obstacles_count = Obstacles_count;
    for (read_index = 0U; read_index < old_obstacles_count; read_index++)
    {
        int32 d_row = (int32)obstacles[read_index].row - explode_row;
        int32 d_col = (int32)obstacles[read_index].col - explode_col;
        int32 abs_row = (d_row < 0) ? -d_row : d_row;
        int32 abs_col = (d_col < 0) ? -d_col : d_col;

        if (abs_row <= 1 && abs_col <= 1)
        {
            continue;
        }
        obstacles[write_index++] = obstacles[read_index];
    }
    Obstacles_count = write_index;
    while (write_index < old_obstacles_count)
    {
        memset(&obstacles[write_index], 0, sizeof(Position));
        write_index++;
    }
    return 1U;
}

/* 前向声明：供识别分段和主路径下发时复用炸弹停留配置。 */
static void configure_bomb_pause_for_path(const Position *path,
                                          size_t steps,
                                          size_t first_scan_index);
static uint8 identify_cell_has_blocker(uint8 row, uint8 col);

static uint8 is_identification_marker(uint8 marker_id)
{
    return ((marker_id & IDENTIFICATION) != 0U) ? 1U : 0U;
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

static void capture_start_yaw_if_needed(void)
{
    if (!g_start_yaw_ready)
    {
        g_start_yaw_deg = wrap_yaw_deg_local(eulerAngle.yaw);
        g_start_yaw_ready = 1U;
    }
}

static float get_start_yaw_base_deg(void)
{
    if (g_start_yaw_ready)
    {
        return g_start_yaw_deg;
    }

    /*
     * 正常流程会在发车动作首次下发前记录基准。
     * 这里保留兜底，避免异常调试入口提前调用方向转换时得到未初始化角度。
     */
    return eulerAngle.yaw;
}

static float map_dir_to_yaw_deg(control_map_dir_t dir)
{
    float base_yaw_deg = get_start_yaw_base_deg();
    float delta_yaw_deg = 0.0f;

    /*
     * 所有地图方向都以发车时记录的车头方向为基准：
     * - 右：发车 yaw
     * - 上：发车 yaw + 90 deg
     * - 左：发车 yaw + 180 deg
     * - 下：发车 yaw - 90 deg
     *
     * 这里不读取视觉传来的 yaw。相机只负责给出位置，方向闭环完全由 IMU 当前 yaw
     * 向“发车 yaw + 偏移”收敛，避免视觉方向抖动或坐标定义不一致污染整车转向。
     */
    switch (dir)
    {
    case CONTROL_MAP_DIR_UP:
        delta_yaw_deg = 90.0f;
        break;
    case CONTROL_MAP_DIR_LEFT:
        delta_yaw_deg = 180.0f;
        break;
    case CONTROL_MAP_DIR_DOWN:
        delta_yaw_deg = -90.0f;
        break;
    case CONTROL_MAP_DIR_RIGHT:
    default:
        delta_yaw_deg = 0.0f;
        break;
    }

    return wrap_yaw_deg_local(base_yaw_deg + delta_yaw_deg);
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

static const control_level_rule_t g_continuous_level_rules[CONTROL_CONTINUOUS_LEVEL_COUNT] =
{
    {0U, CONTROL_PLAN_MODE_1},
    {1U, CONTROL_PLAN_MODE_2},
    {1U, CONTROL_PLAN_MODE_2}
};

static const control_level_rule_t *continuous_get_level_rule(uint8 level_index)
{
    if (level_index >= CONTROL_CONTINUOUS_LEVEL_COUNT)
    {
        level_index = (uint8)(CONTROL_CONTINUOUS_LEVEL_COUNT - 1U);
    }
    return &g_continuous_level_rules[level_index];
}

static int abs_int16_local(int16 v)
{
    return (v < 0) ? -(int)v : (int)v;
}

static uint8 continuous_wheels_stopped(void)
{
    return (abs_int16_local(up_L_all) <= CONTROL_CONTINUOUS_STOP_ENCODER_TOL &&
            abs_int16_local(up_R_all) <= CONTROL_CONTINUOUS_STOP_ENCODER_TOL &&
            abs_int16_local(down_L_all) <= CONTROL_CONTINUOUS_STOP_ENCODER_TOL &&
            abs_int16_local(down_R_all) <= CONTROL_CONTINUOUS_STOP_ENCODER_TOL) ? 1U : 0U;
}

static uint8 continuous_stop_ready(void)
{
    if (continuous_wheels_stopped())
    {
        if (g_continuous_stop_stable_count < CONTROL_CONTINUOUS_STOP_STABLE_LOOPS)
        {
            g_continuous_stop_stable_count++;
        }
    }
    else
    {
        g_continuous_stop_stable_count = 0U;
    }

    return (g_continuous_stop_stable_count >= CONTROL_CONTINUOUS_STOP_STABLE_LOOPS) ? 1U : 0U;
}

static size_t continuous_preset_index_for_level(uint8 level_index)
{
    size_t preset_index = (size_t)g_continuous_preset_base_index + (size_t)level_index;

    if (Map_preset_count == 0U)
    {
        return 0U;
    }
    return preset_index % Map_preset_count;
}

static uint8 localization_wheels_stopped(void)
{
    return (abs_int16_local(up_L_all) <= CONTROL_LOCALIZE_WHEEL_STOP_ENCODER_TOL &&
            abs_int16_local(up_R_all) <= CONTROL_LOCALIZE_WHEEL_STOP_ENCODER_TOL &&
            abs_int16_local(down_L_all) <= CONTROL_LOCALIZE_WHEEL_STOP_ENCODER_TOL &&
            abs_int16_local(down_R_all) <= CONTROL_LOCALIZE_WHEEL_STOP_ENCODER_TOL) ? 1U : 0U;
}

static float localization_median(const float *samples, uint8 count)
{
    float sorted[CONTROL_LOCALIZE_MAX_SAMPLES] = {0.0f};
    uint8 i;
    uint8 j;
    float temp;

    if (samples == NULL || count == 0U) {
        return 0.0f;
    }
    if (count > CONTROL_LOCALIZE_MAX_SAMPLES) {
        count = CONTROL_LOCALIZE_MAX_SAMPLES;
    }

    memcpy(sorted, samples, (size_t)count * sizeof(sorted[0]));
    for (i = 1U; i < count; i++) {
        temp = sorted[i];
        j = i;
        while (j > 0U && sorted[j - 1U] > temp) {
            sorted[j] = sorted[j - 1U];
            j--;
        }
        sorted[j] = temp;
    }

    return sorted[count / 2U];
}

static uint8 localization_first_two_samples_match(void)
{
    float dx = g_localize_samples_x_m[1] - g_localize_samples_x_m[0];
    float dy = g_localize_samples_y_m[1] - g_localize_samples_y_m[0];
    float threshold_sq = CONTROL_LOCALIZE_TWO_SAMPLE_MATCH_M *
                         CONTROL_LOCALIZE_TWO_SAMPLE_MATCH_M;

    return ((dx * dx + dy * dy) <= threshold_sq) ? 1U : 0U;
}

static void reset_level_runtime_state_for_launch(void)
{
    g_path_plan_paused = 0U;
    g_plan_ready = 0U;
    g_exec_steps = 0U;
    g_return_heading_rotate_started = 0U;
    g_pushbox_entry_heading_rotate_started = 0U;
    g_identify_safe_move_prepared = 0U;
    g_identify_safe_move_running = 0U;
    g_identify_return_start_valid = 0U;
    g_identify_return_start_map = (Position){0U, 0U, 0U};
    g_identify_safe_move_steps = 0U;
    memset(g_identify_safe_move_path, 0, sizeof(g_identify_safe_move_path));
    g_continuous_stop_stable_count = 0U;
    memset(g_exec_path, 0, sizeof(g_exec_path));
    memset(g_pushbox_segment_path, 0, sizeof(g_pushbox_segment_path));
    g_pushbox_segment_end_idx = 0U;
    g_pushbox_segment_running = 0U;
    g_pushbox_checkpoint_localized = 0U;
    g_checkpoint_visual_localization_active = 0U;

    reset_localization_accumulator();
    g_prestart_move_started = 0U;
    g_prestart_nominal_pose_valid = 0U;
    g_prestart_nominal_target_x_m = 0.0f;
    g_prestart_nominal_target_y_m = 0.0f;
    g_wait_new_map_frame = 0U;
    g_wait_map_frame_base = 0U;
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

static void configure_flow_for_level(uint8 level_index)
{
    const control_level_rule_t *rule = continuous_get_level_rule(level_index);

    if (g_continuous_run_active && !rule->need_identify)
    {
        g_control_flow_phase = CONTROL_FLOW_PUSHBOX;
        g_control_plan_mode = rule->push_mode;
        return;
    }

    g_control_flow_phase = CONTROL_FLOW_IDENTIFY;
    g_control_plan_mode = rule->push_mode;
}

static void start_current_level_launch(void)
{
    reset_level_runtime_state_for_launch();
    configure_flow_for_level(g_continuous_level_index);
    g_level_start_localization_required = 1U;

    if (Algorithm_Test_PresetInput_IsEnabled())
    {
        Algorithm_Test_PresetInput_Init(continuous_preset_index_for_level(g_continuous_level_index));
    }

    g_control_stage = CONTROL_STAGE_PRESTART_MOVE;
}

static void start_single_launch(void)
{
    reset_level_runtime_state_for_launch();
    g_continuous_run_active = 0U;
    g_continuous_level_index = 0U;
    g_control_flow_phase = CONTROL_FLOW_IDENTIFY;
    g_control_plan_mode = CONTROL_PLAN_MODE_2;
    g_level_start_localization_required = 1U;
    g_control_stage = CONTROL_STAGE_PRESTART_MOVE;
}

static void start_continuous_launch(void)
{
    g_continuous_run_active = 1U;
    g_continuous_level_index = 0U;
    g_continuous_preset_base_index = Menu_Get_Preset_Map_Index();
    start_current_level_launch();
}

static void reset_control_runtime_state(void)
{
    control_disable_yaw_closed_loop();
    g_control_start_enabled = 0U;
    g_control_stage = CONTROL_STAGE_IDLE;
    g_control_flow_phase = CONTROL_FLOW_IDENTIFY;
    g_control_plan_mode = CONTROL_PLAN_MODE_1;
    g_continuous_run_active = 0U;
    g_continuous_level_index = 0U;
    g_level_start_localization_required = 0U;
    g_continuous_preset_base_index = 0U;
    g_start_yaw_deg = 0.0f;
    g_start_yaw_ready = 0U;
    reset_level_runtime_state_for_launch();
    control_disable_yaw_closed_loop();
    g_control_stage = CONTROL_STAGE_IDLE;
    g_level_start_localization_required = 0U;
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

static const char *identify_recognition_type_name(VisionRecognitionType type)
{
    switch (type)
    {
    case VISION_RECOGNITION_IMG:
        return "IMG";
    case VISION_RECOGNITION_NUM:
        return "NUM";
    default:
        return "NONE";
    }
}

static uint8 start_identify_recognition_request(const control_identify_target_t *target,
                                                VisionRecognitionDistance distance);

static void report_identify_result_bluetooth(const control_identify_target_t *target,
                                             const VisionRecognitionResult *result,
                                             uint8 valid_id,
                                             uint8 recognized_id)
{
    const char *type_name = "NONE";
    const char *label_text = "";
    char obj_char = 'U';

    if (target == NULL || result == NULL)
    {
        return;
    }

    type_name = identify_recognition_type_name(result->type);
    obj_char = (target->obj_type == CONTROL_IDENTIFY_OBJ_BOX) ? 'B' : 'T';
    label_text = result->label;

    if (label_text[0] == '\0')
    {
        if (valid_id)
        {
            BlueSerial_Printf("IDR %s %c%u %u,%u d%u id=%u s=%d ok=%u m=%u\r\n",
                              type_name,
                              obj_char,
                              (unsigned int)target->obj_index,
                              (unsigned int)target->obj_pos_map.row,
                              (unsigned int)target->obj_pos_map.col,
                              (unsigned int)target->recog_distance,
                              (unsigned int)recognized_id,
                              (int)result->score,
                              result->success ? 1U : 0U,
                              result->mode_marker ? 1U : 0U);
            return;
        }

        if (result->mode_marker || !result->success)
        {
            label_text = "-1";
        }
        else
        {
            label_text = "?";
        }
    }

    BlueSerial_Printf("IDR %s %c%u %u,%u d%u lb=%s s=%d ok=%u m=%u v=%u id=%u\r\n",
                      type_name,
                      obj_char,
                      (unsigned int)target->obj_index,
                      (unsigned int)target->obj_pos_map.row,
                      (unsigned int)target->obj_pos_map.col,
                      (unsigned int)target->recog_distance,
                      label_text,
                      (int)result->score,
                      result->success ? 1U : 0U,
                      result->mode_marker ? 1U : 0U,
                      (unsigned int)valid_id,
                      (unsigned int)recognized_id);
}

static int16 normalize_identify_confidence(int16 score)
{
    if (score < 0)
    {
        return -1;
    }
    if (score > 100)
    {
        return 100;
    }
    return score;
}

static void record_identify_confidence(const control_identify_target_t *target,
                                       int16 score)
{
    if (target == NULL)
    {
        return;
    }

    score = normalize_identify_confidence(score);
    if (target->obj_type == CONTROL_IDENTIFY_OBJ_BOX &&
        target->obj_index < MAX_BOXES &&
        target->obj_index < Boxes_count)
    {
        g_identify_box_confidence[target->obj_index] = score;
    }
    else if (target->obj_type == CONTROL_IDENTIFY_OBJ_TARGET &&
             target->obj_index < MAX_TARGETS &&
             target->obj_index < Targets_count)
    {
        g_identify_target_confidence[target->obj_index] = score;
    }
}

static void apply_identify_result_to_map(const control_identify_target_t *target,
                                         uint8 valid_id,
                                         uint8 recognized_id,
                                         int16 confidence)
{
    if (target == NULL || !valid_id)
    {
        return;
    }

    record_identify_confidence(target, confidence);

    if (target->obj_type == CONTROL_IDENTIFY_OBJ_BOX &&
        target->obj_index < MAX_BOXES &&
        target->obj_index < Boxes_count)
    {
        boxes[target->obj_index].id = recognized_id;
        g_identify_box_id_assigned[target->obj_index] = 1U;
    }
    else if (target->obj_type == CONTROL_IDENTIFY_OBJ_TARGET &&
             target->obj_index < MAX_TARGETS &&
             target->obj_index < Targets_count)
    {
        targets[target->obj_index].id = recognized_id;
        g_identify_target_id_assigned[target->obj_index] = 1U;
    }
}

static uint8 finish_or_retry_identify_result(const control_identify_target_t *target,
                                             const VisionRecognitionResult *result,
                                             uint8 valid_id,
                                             uint8 recognized_id,
                                             VisionRecognitionDistance distance)
{
    if (result == NULL)
    {
        reset_identify_recognition_wait();
        return 1U;
    }

    /* 高置信度单帧直接采用，保证停车后的识别流程足够快。 */
    /* A medium-confidence first frame always requires the confirming frame to agree. */
    if (g_identify_recog_confirm_pending)
    {
        int16 confidence = result->score;
        if (g_identify_recog_confirm_score < confidence)
        {
            confidence = g_identify_recog_confirm_score;
        }

        if (valid_id &&
            result->score >= CONTROL_IDENTIFY_RECOG_CONFIRM_SCORE &&
            recognized_id == g_identify_recog_confirm_id)
        {
            apply_identify_result_to_map(target, valid_id, recognized_id, confidence);
        }
        else
        {
            record_identify_confidence(target, confidence);
        }
        reset_identify_recognition_wait();
        return 1U;
    }

    if (valid_id && result->score >= CONTROL_IDENTIFY_RECOG_ACCEPT_SCORE)
    {
        apply_identify_result_to_map(target, valid_id, recognized_id, result->score);
        reset_identify_recognition_wait();
        return 1U;
    }

    /* 中等置信度必须用第二帧同标签确认，避免一帧偶然误判写入地图。 */
    if (valid_id && result->score >= CONTROL_IDENTIFY_RECOG_CONFIRM_SCORE)
    {
        g_identify_recog_confirm_pending = 1U;
        g_identify_recog_confirm_id = recognized_id;
        g_identify_recog_confirm_score = normalize_identify_confidence(result->score);
        if (start_identify_recognition_request(target, distance))
        {
            return 0U;
        }
        record_identify_confidence(target, result->score);
        reset_identify_recognition_wait();
        return 1U;
    }

    /* 低分、-1 或非数字标签：只补拍一帧；确认帧失败则立即跳过。 */
    if (!g_identify_recog_confirm_pending &&
        g_identify_recog_retry_count < CONTROL_IDENTIFY_RECOG_MAX_RETRIES)
    {
        g_identify_recog_retry_count++;
        if (start_identify_recognition_request(target, distance))
        {
            return 0U;
        }
    }

    record_identify_confidence(target, result->score);
    reset_identify_recognition_wait();
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

typedef struct
{
    uint8 index;
    uint8 id;
    int16 confidence;
    uint8 active;
} control_identify_id_candidate_t;

static void count_current_identify_ids(size_t box_cnt,
                                       size_t target_cnt,
                                       uint8 box_id_count[256],
                                       uint8 target_id_count[256])
{
    size_t i = 0U;

    memset(box_id_count, 0, 256U * sizeof(box_id_count[0]));
    memset(target_id_count, 0, 256U * sizeof(target_id_count[0]));
    for (i = 0U; i < box_cnt; i++)
    {
        box_id_count[boxes[i].id]++;
    }
    for (i = 0U; i < target_cnt; i++)
    {
        target_id_count[targets[i].id]++;
    }
}

static uint8 identify_ids_are_strictly_one_to_one(size_t box_cnt,
                                                   size_t target_cnt)
{
    uint8 box_id_count[256];
    uint8 target_id_count[256];
    uint16 id = 0U;

    if (box_cnt != target_cnt)
    {
        return 0U;
    }

    count_current_identify_ids(box_cnt, target_cnt, box_id_count, target_id_count);
    for (id = 0U; id < 256U; id++)
    {
        if (box_id_count[id] != target_id_count[id] || box_id_count[id] > 1U)
        {
            return 0U;
        }
    }
    return 1U;
}

static int select_candidate_with_score(const control_identify_id_candidate_t *candidates,
                                       size_t count,
                                       uint8 select_highest)
{
    size_t i = 0U;
    int selected = -1;

    for (i = 0U; i < count; i++)
    {
        if (!candidates[i].active)
        {
            continue;
        }
        if (selected < 0 ||
            (select_highest && candidates[i].confidence > candidates[selected].confidence) ||
            (!select_highest && candidates[i].confidence < candidates[selected].confidence) ||
            (candidates[i].confidence == candidates[selected].confidence &&
             candidates[i].index < candidates[selected].index))
        {
            selected = (int)i;
        }
    }
    return selected;
}

static uint8 split_duplicate_identify_ids(size_t box_cnt,
                                          size_t target_cnt)
{
    uint8 box_id_count[256];
    uint8 target_id_count[256];
    uint8 box_used[MAX_BOXES];
    uint8 target_used[MAX_TARGETS];
    uint16 id = 0U;
    uint16 next_auto_id = CONTROL_IDENTIFY_ID_ELIMINATE;

    count_current_identify_ids(box_cnt, target_cnt, box_id_count, target_id_count);
    for (id = 0U; id < 256U; id++)
    {
        uint8 duplicate_count = 0U;
        size_t i = 0U;
        int keep_box = -1;
        int keep_target = -1;

        if (box_id_count[id] <= 1U || box_id_count[id] != target_id_count[id])
        {
            continue;
        }

        memset(box_used, 0, sizeof(box_used));
        memset(target_used, 0, sizeof(target_used));
        for (i = 0U; i < box_cnt; i++)
        {
            if (boxes[i].id == (uint8)id &&
                (keep_box < 0 || g_identify_box_confidence[i] > g_identify_box_confidence[keep_box]))
            {
                keep_box = (int)i;
            }
        }
        for (i = 0U; i < target_cnt; i++)
        {
            if (targets[i].id == (uint8)id &&
                (keep_target < 0 || g_identify_target_confidence[i] > g_identify_target_confidence[keep_target]))
            {
                keep_target = (int)i;
            }
        }
        if (keep_box < 0 || keep_target < 0)
        {
            return 0U;
        }
        box_used[keep_box] = 1U;
        target_used[keep_target] = 1U;
        duplicate_count = (uint8)(box_id_count[id] - 1U);

        while (duplicate_count > 0U)
        {
            int box_index = -1;
            int target_index = -1;
            uint16 new_id = 0U;

            for (i = 0U; i < box_cnt; i++)
            {
                if (!box_used[i] && boxes[i].id == (uint8)id &&
                    (box_index < 0 ||
                     g_identify_box_confidence[i] < g_identify_box_confidence[box_index]))
                {
                    box_index = (int)i;
                }
            }
            for (i = 0U; i < target_cnt; i++)
            {
                if (!target_used[i] && targets[i].id == (uint8)id &&
                    (target_index < 0 ||
                     g_identify_target_confidence[i] < g_identify_target_confidence[target_index]))
                {
                    target_index = (int)i;
                }
            }
            if (box_index < 0 || target_index < 0)
            {
                return 0U;
            }

            while (next_auto_id < 255U &&
                   (box_id_count[next_auto_id] > 0U || target_id_count[next_auto_id] > 0U))
            {
                next_auto_id++;
            }
            if (next_auto_id >= 255U)
            {
                return 0U;
            }
            new_id = next_auto_id++;
            box_used[box_index] = 1U;
            target_used[target_index] = 1U;
            boxes[box_index].id = (uint8)new_id;
            targets[target_index].id = (uint8)new_id;
            box_id_count[id]--;
            target_id_count[id]--;
            box_id_count[new_id] = 1U;
            target_id_count[new_id] = 1U;
            BlueSerial_Printf("IDSAFE duplicate id=%u B%u T%u -> id=%u\r\n",
                              (unsigned int)id,
                              (unsigned int)box_index,
                              (unsigned int)target_index,
                              (unsigned int)new_id);
            duplicate_count--;
        }
    }
    return 1U;
}

static uint8 repair_identify_ids_by_confidence_impl(size_t box_cnt,
                                                    size_t target_cnt)
{
    uint8 box_id_count[256];
    uint8 target_id_count[256];
    uint8 box_selected[MAX_BOXES] = {0U};
    uint8 target_selected[MAX_TARGETS] = {0U};
    control_identify_id_candidate_t box_candidates[MAX_BOXES];
    control_identify_id_candidate_t target_candidates[MAX_TARGETS];
    size_t box_candidate_count = 0U;
    size_t target_candidate_count = 0U;
    uint16 id = 0U;
    size_t i = 0U;

    if (identify_ids_are_strictly_one_to_one(box_cnt, target_cnt))
    {
        return 1U;
    }
    if (!g_control_identify_id_fallback_enabled)
    {
        BlueSerial_Printf("IDSAFE off: IDs are not one-to-one (B%u T%u)\r\n",
                          (unsigned int)box_cnt,
                          (unsigned int)target_cnt);
        return 0U;
    }
    if (box_cnt != target_cnt)
    {
        BlueSerial_Printf("IDSAFE abort: object count mismatch B%u T%u\r\n",
                          (unsigned int)box_cnt,
                          (unsigned int)target_cnt);
        return 0U;
    }

    memset(box_candidates, 0, sizeof(box_candidates));
    memset(target_candidates, 0, sizeof(target_candidates));
    count_current_identify_ids(box_cnt, target_cnt, box_id_count, target_id_count);

    /* Odd totals for one ID necessarily produce a surplus on one side and enter this list. */
    for (id = 0U; id < 256U; id++)
    {
        uint8 needed = 0U;

        if (box_id_count[id] > target_id_count[id])
        {
            needed = (uint8)(box_id_count[id] - target_id_count[id]);
            while (needed > 0U)
            {
                int selected = -1;
                for (i = 0U; i < box_cnt; i++)
                {
                    if (!box_selected[i] && boxes[i].id == (uint8)id &&
                        (selected < 0 ||
                         g_identify_box_confidence[i] < g_identify_box_confidence[selected]))
                    {
                        selected = (int)i;
                    }
                }
                if (selected < 0 || box_candidate_count >= MAX_BOXES)
                {
                    return 0U;
                }
                box_selected[selected] = 1U;
                box_candidates[box_candidate_count].index = (uint8)selected;
                box_candidates[box_candidate_count].id = (uint8)id;
                box_candidates[box_candidate_count].confidence = g_identify_box_confidence[selected];
                box_candidates[box_candidate_count].active = 1U;
                box_candidate_count++;
                needed--;
            }
        }
        else if (target_id_count[id] > box_id_count[id])
        {
            needed = (uint8)(target_id_count[id] - box_id_count[id]);
            while (needed > 0U)
            {
                int selected = -1;
                for (i = 0U; i < target_cnt; i++)
                {
                    if (!target_selected[i] && targets[i].id == (uint8)id &&
                        (selected < 0 ||
                         g_identify_target_confidence[i] < g_identify_target_confidence[selected]))
                    {
                        selected = (int)i;
                    }
                }
                if (selected < 0 || target_candidate_count >= MAX_TARGETS)
                {
                    return 0U;
                }
                target_selected[selected] = 1U;
                target_candidates[target_candidate_count].index = (uint8)selected;
                target_candidates[target_candidate_count].id = (uint8)id;
                target_candidates[target_candidate_count].confidence = g_identify_target_confidence[selected];
                target_candidates[target_candidate_count].active = 1U;
                target_candidate_count++;
                needed--;
            }
        }
    }

    if (box_candidate_count != target_candidate_count)
    {
        BlueSerial_Printf("IDSAFE abort: surplus mismatch B%u T%u\r\n",
                          (unsigned int)box_candidate_count,
                          (unsigned int)target_candidate_count);
        return 0U;
    }

    for (i = 0U; i < box_candidate_count; i++)
    {
        int low_box = select_candidate_with_score(box_candidates, box_candidate_count, 0U);
        int low_target = select_candidate_with_score(target_candidates, target_candidate_count, 0U);
        uint8 change_box = 0U;

        if (low_box < 0 || low_target < 0)
        {
            return 0U;
        }
        if (box_candidates[low_box].confidence <= target_candidates[low_target].confidence)
        {
            change_box = 1U;
        }

        if (change_box)
        {
            int trusted_target = select_candidate_with_score(target_candidates,
                                                               target_candidate_count,
                                                               1U);
            uint8 box_index = box_candidates[low_box].index;
            uint8 target_index;
            uint8 old_id = boxes[box_index].id;

            if (trusted_target < 0)
            {
                return 0U;
            }
            target_index = target_candidates[trusted_target].index;
            boxes[box_index].id = targets[target_index].id;
            BlueSerial_Printf("IDSAFE B%u id=%u->%u s=%d via T%u s=%d\r\n",
                              (unsigned int)box_index,
                              (unsigned int)old_id,
                              (unsigned int)boxes[box_index].id,
                              (int)box_candidates[low_box].confidence,
                              (unsigned int)target_index,
                              (int)target_candidates[trusted_target].confidence);
            box_candidates[low_box].active = 0U;
            target_candidates[trusted_target].active = 0U;
        }
        else
        {
            int trusted_box = select_candidate_with_score(box_candidates,
                                                           box_candidate_count,
                                                           1U);
            uint8 target_index = target_candidates[low_target].index;
            uint8 box_index;
            uint8 old_id = targets[target_index].id;

            if (trusted_box < 0)
            {
                return 0U;
            }
            box_index = box_candidates[trusted_box].index;
            targets[target_index].id = boxes[box_index].id;
            BlueSerial_Printf("IDSAFE T%u id=%u->%u s=%d via B%u s=%d\r\n",
                              (unsigned int)target_index,
                              (unsigned int)old_id,
                              (unsigned int)targets[target_index].id,
                              (int)target_candidates[low_target].confidence,
                              (unsigned int)box_index,
                              (int)box_candidates[trusted_box].confidence);
            target_candidates[low_target].active = 0U;
            box_candidates[trusted_box].active = 0U;
        }
    }

    count_current_identify_ids(box_cnt, target_cnt, box_id_count, target_id_count);
    for (id = 0U; id < 256U; id++)
    {
        if (box_id_count[id] != target_id_count[id])
        {
            BlueSerial_Printf("IDSAFE abort: repair check failed at id=%u\r\n",
                              (unsigned int)id);
            return 0U;
        }
    }

    if (!split_duplicate_identify_ids(box_cnt, target_cnt) ||
        !identify_ids_are_strictly_one_to_one(box_cnt, target_cnt))
    {
        BlueSerial_Printf("IDSAFE abort: final one-to-one check failed\r\n");
        return 0U;
    }
    BlueSerial_Printf("IDSAFE applied: one-to-one verified (B%u T%u)\r\n",
                      (unsigned int)box_cnt,
                      (unsigned int)target_cnt);
    return 1U;
}

static uint8 repair_identify_ids_by_confidence(size_t box_cnt,
                                               size_t target_cnt)
{
    uint8 original_box_ids[MAX_BOXES] = {0U};
    uint8 original_target_ids[MAX_TARGETS] = {0U};
    size_t i = 0U;
    uint8 repaired = 0U;

    for (i = 0U; i < box_cnt; i++)
    {
        original_box_ids[i] = boxes[i].id;
    }
    for (i = 0U; i < target_cnt; i++)
    {
        original_target_ids[i] = targets[i].id;
    }

    repaired = repair_identify_ids_by_confidence_impl(box_cnt, target_cnt);
    if (!repaired)
    {
        for (i = 0U; i < box_cnt; i++)
        {
            boxes[i].id = original_box_ids[i];
        }
        for (i = 0U; i < target_cnt; i++)
        {
            targets[i].id = original_target_ids[i];
        }
    }
    return repaired;
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
        if (g_identify_box_id_assigned[i])
        {
            box_id_count[boxes[i].id]++;
            has_any_assigned = 1U;
        }
        else if (unassigned_box_count < MAX_BOXES)
        {
            boxes[i].id = MAP_PRESET_UNKNOWN_ID;
            unassigned_box_indices[unassigned_box_count++] = i;
        }
    }

    for (i = 0U; i < target_cnt; i++)
    {
        if (g_identify_target_id_assigned[i])
        {
            target_id_count[targets[i].id]++;
            has_any_assigned = 1U;
        }
        else if (unassigned_target_count < MAX_TARGETS)
        {
            targets[i].id = MAP_PRESET_UNKNOWN_ID;
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
        (void)repair_identify_ids_by_confidence(box_cnt, target_cnt);
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

    (void)repair_identify_ids_by_confidence(box_cnt, target_cnt);
    save_identify_ids_from_current_map();
}

static uint8 start_identify_recognition_request(const control_identify_target_t *target,
                                                VisionRecognitionDistance distance)
{
    if (target == NULL)
    {
        return 0U;
    }

    if (target->obj_type == CONTROL_IDENTIFY_OBJ_BOX)
    {
        if (!uart_send_vision_request_with_sequence(VISION_RECOGNITION_IMG,
                                                    distance,
                                                    &g_identify_recog_expected_sequence))
        {
            return 0U;
        }
        g_identify_recog_wait_type = VISION_RECOGNITION_IMG;
    }
    else
    {
        if (!uart_send_vision_request_with_sequence(VISION_RECOGNITION_NUM,
                                                    distance,
                                                    &g_identify_recog_expected_sequence))
        {
            return 0U;
        }
        g_identify_recog_wait_type = VISION_RECOGNITION_NUM;
    }

    g_identify_recog_waiting = 1U;
    g_identify_recog_wait_start_tick = g_control_tick_10ms;
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
    control_hold_yaw_closed_loop();

    if (Algorithm_Test_PresetInput_IsEnabled())
    {
        map_preset_plan_mode_t preset_mode = Algorithm_Test_PresetInput_GetPlanMode();
        result.type = (target->obj_type == CONTROL_IDENTIFY_OBJ_BOX) ? VISION_RECOGNITION_IMG : VISION_RECOGNITION_NUM;

        system_delay_ms(CONTROL_PRESET_RECOGNITION_DELAY_MS);

        if (preset_mode == MAP_PRESET_PLAN_MODE1)
        {
            result.success = false;
            result.mode_marker = true;
            result.label_is_number = false;
            result.label_value = -1;
            result.score = 0;
            valid_id = 0U;
        }
        else
        {
            result.success = true;
            result.mode_marker = false;
            if (Algorithm_Test_PresetInput_GetObjectId(target->obj_pos_map,
                                                       (target->obj_type == CONTROL_IDENTIFY_OBJ_TARGET) ? 1U : 0U,
                                                       &recognized_id))
            {
                result.label_is_number = true;
                result.label_value = (int32)recognized_id;
                result.score = 100;
                valid_id = 1U;
            }
            else
            {
                result.label_is_number = false;
                result.label_value = -1;
                result.score = -1;
                valid_id = 0U;
            }
        }

        report_identify_result_bluetooth(target, &result, valid_id, recognized_id);
        apply_identify_result_to_map(target, valid_id, recognized_id, result.score);
        reset_identify_recognition_wait();
        return 1U;
    }

    if (!g_identify_recog_waiting)
    {
        if (!g_identify_recog_settle_waiting)
        {
            g_identify_recog_settle_waiting = 1U;
            g_identify_recog_settle_start_tick = g_control_tick_10ms;
            return 0U;
        }

        if ((uint32)(g_control_tick_10ms - g_identify_recog_settle_start_tick) <
            CONTROL_IDENTIFY_RECOG_SETTLE_TICKS)
        {
            return 0U;
        }

        g_identify_recog_settle_waiting = 0U;
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
            if (result.request_seq != g_identify_recog_expected_sequence)
            {
                return 0U;
            }
            valid_id = identify_result_to_valid_id(&result, &recognized_id);
            report_identify_result_bluetooth(target, &result, valid_id, recognized_id);
            return finish_or_retry_identify_result(target,
                                                   &result,
                                                   valid_id,
                                                   recognized_id,
                                                   distance);
        }
    }
    else if (g_identify_recog_wait_type == VISION_RECOGNITION_NUM)
    {
        if (vision_take_num_result(&result))
        {
            if (result.request_seq != g_identify_recog_expected_sequence)
            {
                return 0U;
            }
            valid_id = identify_result_to_valid_id(&result, &recognized_id);
            report_identify_result_bluetooth(target, &result, valid_id, recognized_id);
            return finish_or_retry_identify_result(target,
                                                   &result,
                                                   valid_id,
                                                   recognized_id,
                                                   distance);
        }
    }

    if ((uint32)(g_control_tick_10ms - g_identify_recog_wait_start_tick) >=
        CONTROL_IDENTIFY_RECOG_TIMEOUT_TICKS)
    {
        if (g_identify_recog_retry_count < CONTROL_IDENTIFY_RECOG_MAX_RETRIES)
        {
            g_identify_recog_retry_count++;
            if (!start_identify_recognition_request(target, distance))
            {
                reset_identify_recognition_wait();
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

static void add_identify_target_if_nearest(Position stand_pos,
                                           Position object_pos,
                                           control_map_dir_t face_dir,
                                           control_identify_obj_t obj_type,
                                           uint8 obj_index,
                                           int32 manhattan)
{
    size_t slot = CONTROL_IDENTIFY_MAX_TARGETS_PER_POINT;
    size_t i = 0U;
    control_identify_target_t candidate = {0};

    for (i = 0U; i < g_identify_target_count; i++)
    {
        if (g_identify_targets[i].face_dir == face_dir)
        {
            slot = i;
            break;
        }
    }

    if (slot < g_identify_target_count)
    {
        uint8 old_dist = (g_identify_targets[slot].recog_distance == VISION_RECOGNITION_DISTANCE_TWO_GRID) ? 2U : 1U;
        if ((uint8)manhattan >= old_dist)
        {
            return;
        }
    }
    else
    {
        if (g_identify_target_count >= CONTROL_IDENTIFY_MAX_TARGETS_PER_POINT)
        {
            return;
        }
        slot = g_identify_target_count++;
    }

    candidate.obj_pos_map = object_pos;
    candidate.stand_pos_map = stand_pos;
    candidate.face_dir = face_dir;
    candidate.obj_type = obj_type;
    candidate.recog_distance = (manhattan == 2) ? VISION_RECOGNITION_DISTANCE_TWO_GRID :
                                                 VISION_RECOGNITION_DISTANCE_ONE_GRID;
    candidate.obj_index = obj_index;
    g_identify_targets[slot] = candidate;
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
        add_identify_target_if_nearest(map_point,
                                       boxes[i],
                                       face_dir,
                                       CONTROL_IDENTIFY_OBJ_BOX,
                                       (uint8)i,
                                       manhattan);
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
        add_identify_target_if_nearest(map_point,
                                       targets[i],
                                       face_dir,
                                       CONTROL_IDENTIFY_OBJ_TARGET,
                                       (uint8)i,
                                       manhattan);
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
        /* 爆炸事件也必须成为分段端点，确保暂停结束后立即推进运行时地图。 */
        if (is_identification_marker(g_exec_path[i].id) ||
            (g_exec_path[i].id & BOMB_EXPLOSION) != 0U)
        {
            if (g_identify_endpoint_count >= MAX_CAR_PATH)
            {
                return 0U;
            }
            g_identify_endpoint_indices[g_identify_endpoint_count] = i;
            g_identify_endpoint_need_action[g_identify_endpoint_count] =
                is_identification_marker(g_exec_path[i].id);
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

        /* 分段首点是上一事件的已处理端点，不应再次触发爆炸停留。 */
        configure_bomb_pause_for_path(g_identify_segment_path,
                                      seg_steps,
                                      (g_identify_segment_start_idx > 0U) ? 1U : 0U);
        control_hold_yaw_closed_loop();
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
    uint8 has_identify_targets = 0U;

    if (end_idx >= g_exec_steps || end_idx < g_identify_segment_start_idx)
    {
        return 0U;
    }

    g_identify_rotate_started = 0U;
    g_identify_checkpoint_localized = 0U;
    g_identify_targets_collected_at_endpoint = 0U;
    g_identify_segment_map_event_applied = 0U;
    memset(g_identify_targets, 0, sizeof(g_identify_targets));
    g_identify_target_count = 0U;
    g_identify_target_cursor = 0U;

    if (g_identify_endpoint_cursor < g_identify_endpoint_count &&
        g_identify_endpoint_need_action[g_identify_endpoint_cursor])
    {
        has_identify_targets = collect_identify_targets_on_exec_point(&g_exec_path[end_idx]);
    }

    /*
     * 识别短段开始前先查看该段终点旁边要识别的物体。
     * 若存在待识别目标，先在短段起点转到识别朝向，再保持该朝向行驶到识别位。
     */
    if (g_control_identify_prerotate_enabled && has_identify_targets)
    {
        car_go_flag = 1U;
        car_stop_flag = 0U;
        g_identify_exec_state = CONTROL_IDENTIFY_EXEC_ROTATE_BEFORE_SEGMENT;
        return 1U;
    }

    return begin_identify_segment_motion(end_idx);
}

static void finish_identify_flow_and_relocalize(void)
{
    Position return_start_map = {0U, 0U, 0U};
    uint8 return_start_valid = 0U;

    if (g_identify_segment_start_idx < g_exec_steps)
    {
        return_start_map = g_exec_path[g_identify_segment_start_idx];
        inverse_remap_exec_path_point(&return_start_map);
        return_start_map.id = 0U;
        return_start_valid = (return_start_map.row < MAP_ROWS &&
                              return_start_map.col < MAP_COLS) ? 1U : 0U;
    }

    /* 识别路径刚结束时仍按行驶中停车处理，避免直接断 PWM 造成滑行。 */
    path_follow_set_path(NULL, 0U);
    path_follow_set_pause_indices(NULL, 0U, 0U);
    control_hold_yaw_closed_loop();

    finalize_identify_ids_for_pushbox();
    vision_clear_pending_data();

    g_control_flow_phase = CONTROL_FLOW_PUSHBOX;
    g_control_plan_mode = CONTROL_PLAN_MODE_2;
    g_plan_ready = 0U;
    reset_identify_runtime_state();
    clear_map_request_wait();
    clear_car_request_wait();
    g_wait_new_map_frame = 0U;
    g_wait_map_frame_base = 0U;
    map_data_updated = false;
    car_pose_updated = false;
    reset_localization_accumulator();
    g_pushbox_entry_heading_rotate_started = 0U;
    g_identify_safe_move_prepared = 0U;
    g_identify_safe_move_running = 0U;
    g_identify_return_start_valid = return_start_valid;
    g_identify_return_start_map = return_start_map;
    g_identify_safe_move_steps = 0U;
    memset(g_identify_safe_move_path, 0, sizeof(g_identify_safe_move_path));
    g_control_stage = CONTROL_STAGE_IDENTIFY_RETURN_HEADING;
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
 * @brief 为当前执行路径配置“炸弹爆炸点停留”事件。
 *
 * 逻辑：
 * - 扫描路径中带有 BOMB_EXPLOSION 事件位的点；
 * - 在这些点到达后暂停固定时长（0.5s）；
 * - 若当前路径无爆炸点，则清空暂停配置。
 *
 * @param path 路径点数组。
 * @param steps 路径长度。
 * @param first_scan_index 开始扫描事件的点下标。
 */
static void configure_bomb_pause_for_path(const Position *path,
                                          size_t steps,
                                          size_t first_scan_index)
{
    size_t pause_indices[PATH_FOLLOW_MAX_PAUSE_POINTS] = {0U};
    size_t pause_count = 0U;
    size_t i = 0U;

    if (path == NULL || steps == 0U)
    {
        path_follow_set_pause_indices(NULL, 0U, 0U);
        return;
    }

    if (first_scan_index > steps)
    {
        first_scan_index = steps;
    }
    for (i = first_scan_index; i < steps; i++)
    {
        if ((path[i].id & BOMB_EXPLOSION) == 0U)
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
 * @brief 将视觉位置转换为 path_follow 需要的米制坐标�?
 *
 * 当前工程坐标约定�?
 * - data_handle �?`car_pose.x/y` 为“列/行”浮点栅格�?
 * - path_follow 使用 x 对应行，y 对应列，且单位为�?
 * - 因此转换为：x_m = row * GRID_SIZE_M, y_m = col * GRID_SIZE_M
 *
 * @param[out] x_m 转换后的 x 坐标（米）�?
 * @param[out] y_m 转换后的 y 坐标（米）�?
 * @return uint8
 * - 1：转换成�?
 * - 0：输入参数无效或视觉位置尚未就绪
 */
static uint8 get_camera_position_meter(float *x_m, float *y_m)
{
    float row_f = 0.0f;
    float col_f = 0.0f;

    if (x_m == NULL || y_m == NULL)
    {
        return 0U;
    }
    if (!car_pose_ready)
    {
        return 0U;
    }

/* 视觉位置到执行坐标系的映射需与路径点 remap 保持一致�?*/
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
    return 1U;
}

static void begin_checkpoint_visual_localization(void)
{
    if (g_checkpoint_visual_localization_active)
    {
        return;
    }

    path_follow_set_path(NULL, 0U);
    reset_localization_accumulator();
    clear_car_request_wait();
    car_pose_updated = false;
    g_checkpoint_visual_localization_active = 1U;
    car_go_flag = 1U;
    car_stop_flag = 1U;
}

/* 检查点定位不请求新地图，只在车辆停稳并排空运动旧帧后采集 CARPOS。 */
static uint8 settle_camera_before_checkpoint_localization(void)
{
    uint32 now_tick = g_control_tick_10ms;

    if (g_localize_camera_settled)
    {
        return 1U;
    }

    if (Algorithm_Test_PresetInput_IsEnabled())
    {
        (void)Algorithm_Test_PresetInput_ProvideCarPoseFrame();
        g_localize_camera_settled = 1U;
        return 1U;
    }

    car_go_flag = 1U;
    car_stop_flag = 1U;
    if (!g_localize_stop_sequence_started)
    {
        uart_stop_car_stream();
        uart_blob_clear_pending_data();
        g_localize_stop_sequence_started = 1U;
        g_localize_post_stop_start_tick = now_tick;
    }

    if (!localization_wheels_stopped())
    {
        g_localize_stop_stable_start_tick = 0U;
        return 0U;
    }
    if (g_localize_stop_stable_start_tick == 0U)
    {
        g_localize_stop_stable_start_tick = now_tick;
        return 0U;
    }
    if ((uint32)(now_tick - g_localize_stop_stable_start_tick) <
        CONTROL_LOCALIZE_STOP_STABLE_TICKS)
    {
        return 0U;
    }
    if ((uint32)(now_tick - g_localize_post_stop_start_tick) <
        CONTROL_LOCALIZE_POST_STOP_DRAIN_TICKS)
    {
        return 0U;
    }

    g_localize_camera_settled = 1U;
    return 1U;
}

/* 返回 1 表示本次检查点定位完成，返回 0 表示仍在停车/等待/采样。 */
static uint8 process_checkpoint_visual_localization(void)
{
    uint8 accept_sample = 0U;
    float cam_x_m = 0.0f;
    float cam_y_m = 0.0f;
    float corrected_x_m = 0.0f;
    float corrected_y_m = 0.0f;

    if (!g_checkpoint_visual_localization_active)
    {
        return 1U;
    }
    if (!settle_camera_before_checkpoint_localization())
    {
        return 0U;
    }

    if (!g_localize_fresh_pose_request_started)
    {
        g_localize_car_frame_base = car_frame_count;
        car_pose_updated = false;
        g_localize_fresh_pose_request_started = 1U;
        send_car_request_once();
        return 0U;
    }

    if (car_pose_updated)
    {
        car_pose_updated = false;
        if (car_frame_count != g_localize_car_frame_base)
        {
            clear_car_request_wait();
            accept_sample = 1U;
        }
    }
    if (!accept_sample)
    {
        service_car_request_wait();
        return 0U;
    }
    if (!get_camera_position_meter(&cam_x_m, &cam_y_m))
    {
        service_car_request_wait();
        return 0U;
    }

    if (g_localize_sample_count >= CONTROL_LOCALIZE_MAX_SAMPLES)
    {
        g_localize_sample_count = (uint8)(CONTROL_LOCALIZE_MAX_SAMPLES - 1U);
    }
    g_localize_samples_x_m[g_localize_sample_count] = cam_x_m;
    g_localize_samples_y_m[g_localize_sample_count] = cam_y_m;
    g_localize_sample_count++;

    if (g_localize_sample_count == 2U && localization_first_two_samples_match())
    {
        corrected_x_m = 0.5f * (g_localize_samples_x_m[0] + g_localize_samples_x_m[1]);
        corrected_y_m = 0.5f * (g_localize_samples_y_m[0] + g_localize_samples_y_m[1]);
    }
    else if (g_localize_sample_count < CONTROL_RELOCALIZE_MIN_SAMPLES_PUSHBOX)
    {
        g_localize_car_frame_base = car_frame_count;
        car_pose_updated = false;
        send_car_request_once();
        return 0U;
    }
    else
    {
        corrected_x_m = localization_median(g_localize_samples_x_m,
                                             g_localize_sample_count);
        corrected_y_m = localization_median(g_localize_samples_y_m,
                                             g_localize_sample_count);
    }

    clear_car_request_wait();
    path_follow_reset_pose(corrected_x_m, corrected_y_m, eulerAngle.yaw);
    BlueSerial_ReportPosition();
    control_hold_yaw_closed_loop();
    g_checkpoint_visual_localization_active = 0U;
    return 1U;
}

static void start_localization_map_prefetch_once(void)
{
    if (g_localize_map_prefetch_started)
    {
        return;
    }

    /* Only accept a map produced after this localization has started. */
    g_localize_map_frame_base = map_frame_count;
    g_localize_map_prefetch_started = 1U;
    map_data_updated = false;
    clear_map_request_wait();
    send_map_request_once();
}

static uint8 localization_prefetched_map_ready(void)
{
    return (g_localize_map_prefetch_started && map_data_ready &&
            map_frame_count != g_localize_map_frame_base) ? 1U : 0U;
}

static void finish_localization_to_map_or_plan(void)
{
    if (localization_prefetched_map_ready())
    {
        g_wait_new_map_frame = 0U;
        if (g_control_flow_phase == CONTROL_FLOW_PUSHBOX)
        {
            apply_saved_identify_ids_to_current_map();
        }
        map_data_updated = false;
        clear_map_request_wait();
        set_control_phase_stage(CONTROL_PHASE_STEP_PLAN_PATH);
        return;
    }

    /*
     * The request may still be in flight while CARPOS localization finishes.
     * Keep its original frame baseline and retry timer when entering stage 31.
     * Rebasing here could miss a just-arriving response, while clearing the
     * wait state would immediately send a duplicate START to the same camera.
     */
    g_wait_new_map_frame = 1U;
    g_wait_map_frame_base = g_localize_map_frame_base;
    set_control_phase_stage(CONTROL_PHASE_STEP_WAIT_CAMERA_DATA);
}

static uint8 settle_camera_before_localization_once(void)
{
    uint32 now_tick;

    if (g_localize_camera_settled)
    {
        return 1U;
    }

    if (Algorithm_Test_PresetInput_IsEnabled())
    {
        start_localization_map_prefetch_once();
        (void)Algorithm_Test_PresetInput_ProvideCarPoseFrame();
        g_localize_camera_settled = 1U;
        return 1U;
    }

    /*
     * A camera position is approximately 350 ms behind its report time while
     * the chassis is moving. Stop CARINIT immediately, then require both a
     * 400 ms post-stop drain and 200 ms of quiet encoder feedback before CARPOS.
     * The two waits overlap while the car is braking.
     */
    car_go_flag = 1U;
    car_stop_flag = 1U;
    now_tick = g_control_tick_10ms;

    if (!g_localize_stop_sequence_started) {
        /* Discard any continuous-report frame that may have started in motion. */
        uart_stop_car_stream();
        uart_blob_clear_pending_data();
        g_localize_stop_sequence_started = 1U;
        g_localize_post_stop_start_tick = now_tick;
        start_localization_map_prefetch_once();
    }

    if (!localization_wheels_stopped()) {
        g_localize_stop_stable_start_tick = 0U;
        return 0U;
    }

    if (0U == g_localize_stop_stable_start_tick) {
        g_localize_stop_stable_start_tick = now_tick;
        return 0U;
    }

    if ((uint32)(now_tick - g_localize_stop_stable_start_tick) <
        CONTROL_LOCALIZE_STOP_STABLE_TICKS) {
        return 0U;
    }

    if ((uint32)(now_tick - g_localize_post_stop_start_tick) <
        CONTROL_LOCALIZE_POST_STOP_DRAIN_TICKS) {
        return 0U;
    }

    g_localize_camera_settled = 1U;
    return 1U;
}

/**
 * @brief 进入路径规划保护期�?
 */
static void begin_path_plan_pause(void)
{
    path_follow_set_path(NULL, 0U);
    control_hold_yaw_closed_loop();
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
 * Planning policy:
 * - identify phase always uses Plan_path_Identify();
 * - pushbox phase uses the current g_control_plan_mode.
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
    float prestart_distance_m = 0.0f;
    float nominal_delta_x_m = 0.0f;
    float nominal_delta_y_m = 0.0f;
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
        capture_start_yaw_if_needed();
        hold_yaw_deg = map_dir_to_yaw_deg(CONTROL_MAP_DIR_RIGHT);
        prestart_dir = get_prestart_depart_map_dir();
        move_yaw_deg = map_dir_to_yaw_deg(prestart_dir);
        move_yaw_rad = move_yaw_deg * CONTROL_DEG_TO_RAD;
        prestart_distance_m = CONTROL_PRESTART_OFFSET_M +
                              g_control_prestart_depart_compensate_m;
        if (prestart_distance_m < 0.0f)
        {
            prestart_distance_m = 0.0f;
        }
        start_x_m = st.x_m;
        start_y_m = st.y_m;

        nominal_delta_x_m = cosf(move_yaw_rad) * CONTROL_PRESTART_OFFSET_M;
        nominal_delta_y_m = sinf(move_yaw_rad) * CONTROL_PRESTART_OFFSET_M;
        g_prestart_nominal_target_x_m = clampf_local(start_x_m + nominal_delta_x_m,
                                                     0.0f,
                                                     CONTROL_WORLD_X_MAX_M);
        g_prestart_nominal_target_y_m = clampf_local(start_y_m + nominal_delta_y_m,
                                                     0.0f,
                                                     CONTROL_WORLD_Y_MAX_M);
        g_prestart_nominal_pose_valid = 1U;

        delta_x_m = cosf(move_yaw_rad) * prestart_distance_m;
        delta_y_m = sinf(move_yaw_rad) * prestart_distance_m;

        /* 防止起步目标越界（负坐标�?uint8 会变 255，导致远距离猛冲）�?*/
        target_x_m = clampf_local(start_x_m + delta_x_m, 0.0f, CONTROL_WORLD_X_MAX_M);
        target_y_m = clampf_local(start_y_m + delta_y_m, 0.0f, CONTROL_WORLD_Y_MAX_M);

        /* 发车只改变平移方向，不改变车头朝向。 */
        path_follow_reset_pose(st.x_m, st.y_m, hold_yaw_deg);
        control_hold_yaw_closed_loop();
        path_follow_start_pose_correction(target_x_m, target_y_m);
        g_prestart_move_started = 1U;
        return;
    }

    path_follow_get_status(&st);
    if (!st.active)
    {
        /*
         * 起步动作刚结束时车仍可能有惯性，不能直接 go=0 断 PWM。
         * 这里沿用其他行驶中停车的标志组合：go=1 且 stop=1，
         * 让 main.c 中的 motor_control(car_stop_array) 继续做闭环零速刹停，
         * 等四轮编码器接近静止后再由底层清 PID 并断 PWM，避免小车滑动。
         */
        if (g_prestart_nominal_pose_valid)
        {
            path_follow_reset_pose(g_prestart_nominal_target_x_m,
                                   g_prestart_nominal_target_y_m,
                                   map_dir_to_yaw_deg(CONTROL_MAP_DIR_RIGHT));
            g_prestart_nominal_pose_valid = 0U;
        }
        control_hold_yaw_closed_loop();
        reset_localization_accumulator();
        set_control_phase_stage(CONTROL_PHASE_STEP_LOCALIZE);
    }
}

/**
 * @brief 识别结束后的安全定位点移动与航向回正过渡阶段。
 *
 * 先移动到最近的可达安全格，再回正发车方向。两项都完成后才进入 stage30，
 * 随后执行停车、CARSTOP、旧帧排空和视觉定位。
 */
static void handle_identify_return_heading_stage(void)
{
    path_follow_status_t st = {0};
    static path_map_snapshot_t runtime_map;
    float start_base_yaw_deg = map_dir_to_yaw_deg(CONTROL_MAP_DIR_RIGHT);

    if (!g_identify_safe_move_prepared)
    {
        g_identify_safe_move_prepared = 1U;
        g_identify_safe_move_steps = 0U;
        memset(g_identify_safe_move_path, 0, sizeof(g_identify_safe_move_path));
        memset(&runtime_map, 0, sizeof(runtime_map));
        snapshot_take(&runtime_map);

        if (g_identify_return_start_valid &&
            path_build_nearest_wall_clear_exec(&g_identify_return_start_map,
                                               &runtime_map,
                                               g_identify_safe_move_path,
                                               PATH_SAFE_RELOCATION_MAX_POINTS,
                                               &g_identify_safe_move_steps) &&
            g_identify_safe_move_steps >= 2U)
        {
            path_follow_set_pause_indices(NULL, 0U, 0U);
            control_hold_yaw_closed_loop();
            path_follow_set_path_pause_enabled(g_identify_safe_move_path,
                                               g_identify_safe_move_steps,
                                               0U);
            car_go_flag = 1U;
            car_stop_flag = 0U;
            g_identify_safe_move_running = 1U;
            return;
        }
    }

    if (g_identify_safe_move_running)
    {
        path_follow_get_status(&st);
        if (st.active)
        {
            car_go_flag = 1U;
            car_stop_flag = 0U;
            return;
        }
        g_identify_safe_move_running = 0U;
        control_hold_yaw_closed_loop();
    }

    if (!g_pushbox_entry_heading_rotate_started)
    {
        car_go_flag = 1U;
        car_stop_flag = 0U;
        path_follow_start_rotate_to_yaw(start_base_yaw_deg);
        control_enable_yaw_closed_loop();
        g_pushbox_entry_heading_rotate_started = 1U;
        return;
    }

    path_follow_get_status(&st);
    if (st.active)
    {
        car_go_flag = 1U;
        car_stop_flag = 0U;
        return;
    }

    /*
     * Heading alignment is complete before stage 30. Only now start the
     * stop/CARSTOP/drain/localization sequence, so CARPOS is sampled with the
     * same launch-relative heading every time.
     */
    g_pushbox_entry_heading_rotate_started = 0U;
    g_identify_safe_move_prepared = 0U;
    g_identify_return_start_valid = 0U;
    control_hold_yaw_closed_loop();
    reset_localization_accumulator();
    set_control_phase_stage(CONTROL_PHASE_STEP_LOCALIZE);
}

/**
 * @brief 处理视觉定位阶段。
 *
 * 进入本阶段前航向已经回正；这里只负责停车、并行启动地图预取、CARPOS 多帧采样，
 * 以及写入视觉位置，不再触发原地旋转。发车定位与推箱前定位共用此流程。
 */
static void handle_localization_stage(void)
{
    uint8 accept_sample = 0U;
    uint8 min_samples = CONTROL_LOCALIZE_MIN_SAMPLES;
    float cam_x_m = 0.0f;
    float cam_y_m = 0.0f;
    float avg_x_m = 0.0f;
    float avg_y_m = 0.0f;
    float start_base_yaw_deg = map_dir_to_yaw_deg(CONTROL_MAP_DIR_RIGHT);

    if (g_control_flow_phase == CONTROL_FLOW_PUSHBOX)
    {
        min_samples = CONTROL_RELOCALIZE_MIN_SAMPLES_PUSHBOX;
    }
    if (min_samples > CONTROL_LOCALIZE_MAX_SAMPLES)
    {
        min_samples = CONTROL_LOCALIZE_MAX_SAMPLES;
    }

    if (!settle_camera_before_localization_once())
    {
        return;
    }

    if (!g_localize_fresh_pose_request_started)
    {
        /* The CARSTOP drain is complete; keep a concurrently received MAP frame. */
        g_localize_car_frame_base = car_frame_count;
        car_pose_updated = false;
        g_localize_fresh_pose_request_started = 1U;
        send_car_request_once();
        return;
    }

    if (car_pose_updated)
    {
        car_pose_updated = false;
        if (car_frame_count != g_localize_car_frame_base)
        {
            clear_car_request_wait();
            accept_sample = 1U;
        }
    }

    if (!accept_sample)
    {
        service_car_request_wait();
        return;
    }
    if (!get_camera_position_meter(&cam_x_m, &cam_y_m))
    {
        service_car_request_wait();
        return;
    }

    g_localize_samples_x_m[g_localize_sample_count] = cam_x_m;
    g_localize_samples_y_m[g_localize_sample_count] = cam_y_m;
    g_localize_sample_count++;

    if (g_localize_sample_count == 2U &&
        localization_first_two_samples_match())
    {
        avg_x_m = 0.5f * (g_localize_samples_x_m[0] + g_localize_samples_x_m[1]);
        avg_y_m = 0.5f * (g_localize_samples_y_m[0] + g_localize_samples_y_m[1]);
    }
    else if (g_localize_sample_count < min_samples)
    {
        g_localize_car_frame_base = car_frame_count;
        car_pose_updated = false;
        send_car_request_once();
        return;
    }
    else
    {
        avg_x_m = localization_median(g_localize_samples_x_m, g_localize_sample_count);
        avg_y_m = localization_median(g_localize_samples_y_m, g_localize_sample_count);
    }

    clear_car_request_wait();

    /*
     * 初始定位的关键修正：
     * - 视觉只提供地图位置平均值；
     * - 航向已经在进入 stage30 前回正，这里只同步位置并保持发车方向基准。
     */
    path_follow_reset_pose(avg_x_m, avg_y_m, start_base_yaw_deg);
    BlueSerial_ReportPosition();
    control_hold_yaw_closed_loop();
    g_level_start_localization_required = 0U;
    finish_localization_to_map_or_plan();
}

/**
 * @brief 处理“等待地图”阶段�?
 *
 * 阶段目标�?
 * - 按需请求地图；本阶段不使用相机 CAR 位姿修正里程计
 * - 收到有效地图后切换到规划阶段
 */
static void handle_wait_camera_data(void)
{
    /* 丢弃等待地图期间到达的 CAR 更新，避免旧帧被后续定位流程使用。 */
    car_pose_updated = false;

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

static void finish_identify_rotation(void)
{
    g_identify_rotate_started = 0U;
    reset_identify_recognition_wait();
}

static void enter_identify_recognition_state(void)
{
    g_identify_exec_state = CONTROL_IDENTIFY_EXEC_DO_RECOGNIZE;
}

static void finish_current_identify_target(const control_identify_target_t *target)
{
    if (target != NULL)
    {
        if (target->obj_type == CONTROL_IDENTIFY_OBJ_BOX &&
            target->obj_index < MAX_BOXES)
        {
            g_identified_box_flags[target->obj_index] = 1U;
        }
        else if (target->obj_type == CONTROL_IDENTIFY_OBJ_TARGET &&
                 target->obj_index < MAX_TARGETS)
        {
            g_identified_target_flags[target->obj_index] = 1U;
        }
    }

    if (g_identify_target_cursor < g_identify_target_count)
    {
        g_identify_target_cursor++;
    }

    if (g_identify_target_cursor < g_identify_target_count)
    {
        g_identify_exec_state = CONTROL_IDENTIFY_EXEC_ROTATE_TO_TARGET;
    }
    else
    {
        g_identify_rotate_started = 0U;
        g_identify_exec_state = CONTROL_IDENTIFY_EXEC_ADVANCE_ENDPOINT;
    }
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
            finish_identify_rotation();
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

        finish_identify_rotation();
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
        if (!g_identify_segment_map_event_applied)
        {
            if (!identify_apply_runtime_map_event(endpoint_idx))
            {
                g_control_stage = CONTROL_STAGE_ERROR;
                return;
            }
            g_identify_segment_map_event_applied = 1U;
        }

        /* 炸弹推动发生在识别路径内，不会进入普通推箱分段；到达爆炸点后单独校正。 */
        if (g_control_checkpoint_vision_localization_enabled &&
            (g_exec_path[endpoint_idx].id & BOMB_EXPLOSION) != 0U &&
            !g_identify_checkpoint_localized)
        {
            begin_checkpoint_visual_localization();
            if (!process_checkpoint_visual_localization())
            {
                return;
            }
            g_identify_checkpoint_localized = 1U;
        }

        if (g_identify_endpoint_need_action[g_identify_endpoint_cursor])
        {
            if (g_identify_target_count == 0U)
            {
                g_identify_targets_collected_at_endpoint =
                    collect_identify_targets_on_exec_point(&g_exec_path[endpoint_idx]);
            }

            if (g_identify_target_count > 0U)
            {
                /* 每个识别驻车点只定位一次，多目标原地转向不会重复请求 CARPOS。 */
                if (g_control_checkpoint_vision_localization_enabled &&
                    !g_identify_checkpoint_localized)
                {
                    begin_checkpoint_visual_localization();
                    if (!process_checkpoint_visual_localization())
                    {
                        return;
                    }
                    g_identify_checkpoint_localized = 1U;
                }

                if (g_control_identify_prerotate_enabled &&
                    !g_identify_targets_collected_at_endpoint)
                {
                    enter_identify_recognition_state();
                }
                else
                {
                    g_identify_rotate_started = 0U;
                    g_identify_exec_state = CONTROL_IDENTIFY_EXEC_ROTATE_TO_TARGET;
                }
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
            g_identify_exec_state = CONTROL_IDENTIFY_EXEC_ADVANCE_ENDPOINT;
            return;
        }

        curr_target = &g_identify_targets[g_identify_target_cursor];
        target_yaw_deg = map_dir_to_yaw_deg(curr_target->face_dir);
        yaw_err_deg = wrap_yaw_deg_local(target_yaw_deg - eulerAngle.yaw);
        if (fabsf(yaw_err_deg) <= CONTROL_IDENTIFY_YAW_ALIGN_TOL_DEG)
        {
            finish_identify_rotation();
            enter_identify_recognition_state();
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

        finish_identify_rotation();
        enter_identify_recognition_state();
        break;

    case CONTROL_IDENTIFY_EXEC_DO_RECOGNIZE:
        if (g_identify_target_cursor < g_identify_target_count)
        {
            curr_target = &g_identify_targets[g_identify_target_cursor];
            if (!identify_action_stub(curr_target))
            {
                return;
            }

            finish_current_identify_target(curr_target);
            return;
        }

        g_identify_exec_state = CONTROL_IDENTIFY_EXEC_ADVANCE_ENDPOINT;
        break;

    case CONTROL_IDENTIFY_EXEC_ADVANCE_ENDPOINT:
        /* 当前识别点处理完成，保持现有车头朝向，直接推进到下一段。 */
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

static uint8 start_pushbox_segment(size_t start_idx)
{
    size_t end_idx;
    size_t scan_idx;
    size_t segment_steps;
    size_t i;

    if (start_idx >= g_exec_steps)
    {
        return 0U;
    }

    end_idx = g_exec_steps - 1U;
    scan_idx = (start_idx == 0U) ? 0U : (start_idx + 1U);
    for (; scan_idx < g_exec_steps; scan_idx++)
    {
        if ((g_exec_path[scan_idx].id & PUSH_END_POINT) != 0U)
        {
            end_idx = scan_idx;
            break;
        }
    }

    segment_steps = end_idx - start_idx + 1U;
    if (segment_steps == 0U || segment_steps > MAX_CAR_PATH)
    {
        return 0U;
    }
    for (i = 0U; i < segment_steps; i++)
    {
        g_pushbox_segment_path[i] = g_exec_path[start_idx + i];
    }

    configure_bomb_pause_for_path(g_pushbox_segment_path,
                                  segment_steps,
                                  (start_idx > 0U) ? 1U : 0U);
    control_hold_yaw_closed_loop();
    path_follow_set_path(g_pushbox_segment_path, segment_steps);
    g_pushbox_segment_end_idx = end_idx;
    g_pushbox_segment_running = 1U;
    g_pushbox_checkpoint_localized = 0U;
    car_go_flag = 1U;
    car_stop_flag = 0U;
    return 1U;
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

    if (g_control_checkpoint_vision_localization_enabled)
    {
        if (!start_pushbox_segment(0U))
        {
            g_control_stage = CONTROL_STAGE_ERROR;
            return;
        }
    }
    else
    {
        configure_bomb_pause_for_path(g_exec_path, g_exec_steps, 0U);
        control_hold_yaw_closed_loop();
        path_follow_set_path(g_exec_path, g_exec_steps);
    }

    car_go_flag = 1U;
    car_stop_flag = 0U;
    set_control_phase_stage(CONTROL_PHASE_STEP_EXECUTE_PATH);
}

static void finish_pushbox_flow_after_return(void)
{
    control_hold_yaw_closed_loop();
    g_return_heading_rotate_started = 0U;
    set_control_phase_stage(CONTROL_PHASE_STEP_FINISHED);
}

/**
 * @brief 推箱执行阶段：轮询 path_follow 状态，路径结束后进入完成态并保持停车。
 */
static void handle_pushbox_execute_path(void)
{
    path_follow_status_t st = {0};
    float return_yaw_deg = 0.0f;
    float yaw_err_deg = 0.0f;

    if (g_control_checkpoint_vision_localization_enabled)
    {
        if (g_pushbox_segment_running)
        {
            path_follow_get_status(&st);
            if (st.active)
            {
                return;
            }
            g_pushbox_segment_running = 0U;
        }

        if (g_pushbox_segment_end_idx >= g_exec_steps)
        {
            g_control_stage = CONTROL_STAGE_ERROR;
            return;
        }

        if ((g_exec_path[g_pushbox_segment_end_idx].id & PUSH_END_POINT) != 0U &&
            !g_pushbox_checkpoint_localized)
        {
            begin_checkpoint_visual_localization();
            if (!process_checkpoint_visual_localization())
            {
                return;
            }
            g_pushbox_checkpoint_localized = 1U;
        }

        if (g_pushbox_segment_end_idx < (g_exec_steps - 1U))
        {
            if (!start_pushbox_segment(g_pushbox_segment_end_idx))
            {
                g_control_stage = CONTROL_STAGE_ERROR;
            }
            return;
        }
    }
    else
    {
        path_follow_get_status(&st);
        if (st.active)
        {
            return;
        }
    }

    return_yaw_deg = map_dir_to_yaw_deg(CONTROL_MAP_DIR_RIGHT);
    yaw_err_deg = wrap_yaw_deg_local(return_yaw_deg - eulerAngle.yaw);
    if (fabsf(yaw_err_deg) > CONTROL_RETURN_YAW_ALIGN_TOL_DEG)
    {
        car_go_flag = 1U;
        car_stop_flag = 0U;
        if (!g_return_heading_rotate_started)
        {
            path_follow_start_rotate_to_yaw(return_yaw_deg);
            g_return_heading_rotate_started = 1U;
        }
        return;
    }

    finish_pushbox_flow_after_return();
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

static void handle_pushbox_finished_stage(void)
{
    control_hold_yaw_closed_loop();

    if (g_continuous_run_active && g_control_continuous_levels_enabled)
    {
        if ((uint8)(g_continuous_level_index + 1U) < CONTROL_CONTINUOUS_LEVEL_COUNT)
        {
            if (!continuous_stop_ready())
            {
                return;
            }
            g_continuous_stop_stable_count = 0U;
            g_continuous_level_index++;
            start_current_level_launch();
            return;
        }
    }

    g_continuous_stop_stable_count = 0U;
    g_continuous_run_active = 0U;
}

/**
 * @brief 错误阶段：安全停车，并等待新地图后重新尝试规划。
 *
 * 只按需请求新地图；车位置由定位阶段单独获取。
 */
static void handle_error_stage(void)
{
    control_hold_yaw_closed_loop();

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

void control_tick_10ms(void)
{
    g_control_tick_10ms++;
}

void control_init(void)
{
    reset_control_runtime_state();
    Algorithm_Test_PresetInput_SetEnabled(Menu_Get_Preset_Input_Enabled(),
                                          Menu_Get_Preset_Map_Index());
}

void control_restart(void)
{
    reset_control_runtime_state();
    if (Algorithm_Test_PresetInput_IsEnabled())
    {
        Algorithm_Test_PresetInput_Init(Menu_Get_Preset_Map_Index());
    }
}

void control_set_start_enabled(uint8 enabled)
{
    if (enabled)
    {
        g_control_start_enabled = 1U;
        if (g_control_stage == CONTROL_STAGE_IDLE)
        {
            if (g_control_continuous_levels_enabled)
            {
                start_continuous_launch();
            }
            else
            {
                start_single_launch();
            }
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
     * 1) 先消费串口 FIFO，保证地图、车位置、识别结果缓存尽量新；
     * 2) 再按当前阶段调用对应 handler，handler 内部只负责推进一个阶段。
     *
     * 这样主状态机保留“流程骨架”，阶段细节放在命名函数里，后续调车时更容易定位问题。
     */
    if (!Algorithm_Test_PresetInput_IsEnabled())
    {
        process_blob_data();
        process_vision_data();
    }

    if (!g_control_start_enabled)
    {
        control_disable_yaw_closed_loop();
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

    case CONTROL_STAGE_IDENTIFY_RETURN_HEADING:
        handle_identify_return_heading_stage();
        break;

    case CONTROL_STAGE_IDENTIFY_LOCALIZE:
    case CONTROL_STAGE_PUSHBOX_LOCALIZE:
        handle_localization_stage();
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

    case CONTROL_STAGE_PUSHBOX_FINISHED:
        handle_pushbox_finished_stage();
        /* Single-run stops here; continuous-run may launch the next level. */
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

void control_set_checkpoint_vision_localization_enabled(uint8 enabled)
{
    g_control_checkpoint_vision_localization_enabled = (enabled != 0U) ? 1U : 0U;
}

uint8 control_get_checkpoint_vision_localization_enabled(void)
{
    return g_control_checkpoint_vision_localization_enabled;
}

void control_set_identify_prerotate_enabled(uint8 enabled)
{
    g_control_identify_prerotate_enabled = (enabled != 0U) ? 1U : 0U;
}

uint8 control_get_identify_prerotate_enabled(void)
{
    return g_control_identify_prerotate_enabled;
}

void control_set_identify_id_fallback_enabled(uint8 enabled)
{
    g_control_identify_id_fallback_enabled = (enabled != 0U) ? 1U : 0U;
}

uint8 control_get_identify_id_fallback_enabled(void)
{
    return g_control_identify_id_fallback_enabled;
}

void control_set_continuous_levels_enabled(uint8 enabled)
{
    g_control_continuous_levels_enabled = (enabled != 0U) ? 1U : 0U;
    if (!g_control_continuous_levels_enabled)
    {
        g_continuous_run_active = 0U;
    }
}

uint8 control_get_continuous_levels_enabled(void)
{
    return g_control_continuous_levels_enabled;
}

/**
 * @brief 手动设置推箱阶段规划模式。
 *
 * 为了避免非法输入破坏流程，除 CONTROL_PLAN_MODE_1 外都按
 * CONTROL_PLAN_MODE_2 处理；识别阶段不再自动覆盖该值。
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
    if (g_path_plan_paused && path_follow_get_stationary_yaw_hold_enabled())
    {
        return 0U;
    }
    return g_path_plan_paused;
}
