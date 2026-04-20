#include "Control_Old.h"

#define RECON_REQUEST_CAPTURE_CMD "SNAP\n" // 发送给识别端的抓拍命令。

#define CAR_INIT_POSE_X_M 0.05f
#define CAR_INIT_POSE_Y_M 1.10f
#define CAR_INIT_ALIGN_TOLERANCE_M 0.02f
#define BOMB_PUSH_SETTLE_PAUSE_MS 1000U
#define RECON_PATH_USE_PAUSE_EVENTS 0U

// size_t steps;                                              // 规划器输出的完整路径点数量（path 中有效元素个数）。
// int res;                                                   // 规划或流程函数返回值，0 通常表示成功。
// PlannerPointV3_BFS path[GREEDY_AREA];                      // 规划器输出的网格路径（规划坐标系）。
// size_t box_target_mapping[PLANNER_V3_TWO_PHASE_MAX_BOXES]; // 箱子索引 -> 目标索引的匹配关系。
// PlannerChainInfo chain_info;
// PlannerAllBoxPaths first_paths, final_paths; // 首车识别阶段路径与最终推箱阶段路径结果缓存。
// size_t used_bombs[1];
// PlannerBoxOverlap overlaps[PLANNER_V3_BFS_MAX_BOXES];

Position corner_path[MAX_CAR_PATH]; // 实际执行坐标系下的折线路径（角点序列）。
size_t corner_steps = 0;            // corner_path 当前有效角点个数。

typedef enum
{
    RECON_STAGE_IDLE = 0,       // 空闲：尚未开始识别流程。
    RECON_STAGE_LOAD_SEGMENT,   // 装载本次识别任务对应的路径段。
    RECON_STAGE_FOLLOW_SEGMENT, // 跟随路径段移动到识别观察点。
    RECON_STAGE_TURN_TO_ANCHOR, // 原地转向到预设朝向，保证识别视角一致。
    RECON_STAGE_WAIT_RESULT,    // 等待 UART4 返回识别结果标签。
    RECON_STAGE_COMPLETE,       // 识别流程结束且结果可用于重规划。
    RECON_STAGE_ERROR           // 识别流程失败。
} recon_stage_t;

typedef struct
{
    uint8 valid;               // 任务项是否有效。
    uint8 item_type;           // 识别对象类型（箱子/目标）。
    uint8 source_index;        // 对应源数组索引（box 或 target 的下标）。
    uint16 corner_index;       // 达到识别位姿时对应的角点索引。
    float target_yaw_deg_exec; // 到达后需要对齐的目标朝向（执行坐标系角度）。
} ReconVisitTask;

typedef enum
{
    CAR_INIT_STAGE_IDLE = 0,           // 空闲。
    CAR_INIT_STAGE_PRESTART_MOVE,      // 执行预设短位移，为外部定位准备观测条件。
    CAR_INIT_STAGE_WAIT_EXTERNAL_POSE, // 等待外部位姿数据。
    CAR_INIT_STAGE_CORRECTING,         // 使用外部位姿持续修正里程计姿态。
    CAR_INIT_STAGE_READY               // 上电定位完成，可进入常规任务。
} car_init_stage_t;

/* 识别阶段状态机运行时数据。 */
recon_stage_t g_recon_stage = RECON_STAGE_IDLE;             // 当前识别状态机阶段。
uint8 g_recon_active = 0U;                                  // 识别流程是否正在执行。
uint8 g_recon_complete = 0U;                                // 识别流程是否已成功完成。
uint8 g_recon_failed = 0U;                                  // 识别流程是否失败。
size_t g_recon_current_visit_idx = 0U;                      // 当前执行到第几个识别访问点。
Position g_recon_segment_path[100] = {0};                   // 当前一次识别任务要走的路径段。
Position g_recon_boxes_snapshot[MAX_BOXES] = {{0}};         // 开始识别前拍下的箱子快照。
Position g_recon_targets_snapshot[MAX_TARGETS] = {{0}};     // 开始识别前拍下的目标点快照。
Position g_recon_bombs_snapshot[MAX_BOMBS] = {{0}};         // 开始识别前拍下的炸弹快照。
Position g_recon_obstacles_snapshot[MAX_OBSTACLES] = {{0}}; // 开始识别前拍下的障碍快照。
size_t g_recon_box_count = 0U;                              // 快照中箱子数量。
size_t g_recon_target_count = 0U;                           // 快照中目标数量。
size_t g_recon_bomb_count = 0U;                             // 快照中炸弹数量。
size_t g_recon_obstacle_count = 0U;                         // 快照中障碍数量。
Position g_recon_car_start_planner = {0, 0};                // 识别前小车规划坐标起点。
Position g_recon_car_after_exec_planner = {0, 0};           // 识别路径执行后小车规划坐标。
uint8 g_recon_car_after_exec_valid = 0U;                    // 上述终点坐标是否有效。
size_t g_recon_manual_box_target_mapping[MAX_BOXES] = {0};  // 基于识别标签解析出的箱子-目标映射。
const size_t g_recon_manual_target_mapping_seed[MAX_BOXES] = {0U, 1U, 2U, 3U, 4U};

ReconVisitTask draw_path_first_car_recon_tasks[MAX_TARGETS + MAX_BOXES] = {0}; // 首车识别任务列表。
size_t draw_path_first_car_recon_task_count = 0U;                              // 任务列表有效项数量。
Position draw_path_first_car_corner_path_exec[MAX_CAR_PATH] = {0};             // 映射到执行坐标后的首车角点路径。
size_t draw_path_first_car_corner_steps_exec = 0U;                             // 上述角点路径有效长度。
size_t g_recon_preplanned_box_target_mapping[MAX_BOXES] = {0};                 // 预规划时使用的种子映射快照。
uint8 g_recon_push_route_ready = 0U;                                           // 推箱阶段预规划结果是否已可复用。

uint8 data_control_flag = 0; // 主流程控制标志：0=等待/规划阶段，1=路径已下发并执行中。

/* 上电定位（初始姿态校准）相关状态。 */
volatile uint8 car_init_required = 0;     // 是否需要执行上电定位流程。
volatile uint8 car_init_active = 0;       // 上电定位流程是否正在进行。
volatile uint8 car_init_done = 0;         // 上电定位流程是否完成。
volatile uint8 prestart_move_started = 0; // 上电定位前置位移动作是否已触发。

int g_carinit_last_target_idx = -1;                      // 上一次触发动态校正时绑定的 target 索引（防重复触发）。
uint8 g_carinit_wait_external = 0;                       // 已发送 CARINIT，当前等待外部位姿中。
uint8 g_carinit_forward_check = 0;                       // 前向一致性检查阶段标志。
uint8 g_carinit_axis = 0;                                // 当前路段主运动轴：1=X，2=Y。
size_t g_carinit_bind_target_idx = 0;                    // 当前外部位姿流绑定到哪一个路径 target。
float g_carinit_anchor_parallel_m = 0.0f;                // 前向检查时记录的并行轴锚点。
uint8 g_carinit_stream_open = 0;                         // 是否已向外部打开 CARINIT 数据流。
car_init_stage_t g_car_init_stage = CAR_INIT_STAGE_IDLE; // 上电定位子状态机当前阶段。
CarPose g_car_init_target_m = {0, 0, 0, 0, 0, 0};        // 最近一次外部提供的目标位姿（米）。
uint8 g_car_init_target_valid = 0;                       // g_car_init_target_m 是否有效。
uint8 path_plan_pause = 0U;                              // 路径重规划保护期开关（暂停姿态更新与 UART 消费）。

/* 进入“规划保护期”：暂停姿态更新与串口缓冲消费，避免重规划期间状态被异步更新污染。 */
static void begin_path_plan_pause(void)
{
    if (path_plan_pause)
    {
        return;
    }
    path_plan_pause = 1U;
    data_handle_discard_pending_rx();
    data_handle_discard_pending_recon_rx();
    pit_disable(PIT_CH2);
    pit_flag_clear(PIT_CH2);
}

/* 退出“规划保护期”：恢复 PIT2 与数据流，并给姿态解算留一个短暂稳定窗口。 */
static void end_path_plan_pause(void)
{
    if (!path_plan_pause)
    {
        return;
    }
    data_handle_discard_pending_rx();
    data_handle_discard_pending_recon_rx();
    pit_flag_clear(PIT_CH2);
    pit_enable(PIT_CH2);
    path_plan_pause = 0U;
    /* Let PIT2 refresh attitude a couple of times before path following resumes. */
    system_delay_ms(4);
    data_handle_discard_pending_rx();
    data_handle_discard_pending_recon_rx();
}

/* 本地绝对值函数：避免在中断/嵌入式环境中引入额外依赖。 */
static float absf_local(float v)
{
    return (v >= 0.0f) ? v : (-v);
}

/* 清空最终推箱路径相关缓存，供重规划失败回退或下一轮规划复用。 */
static void recon_clear_final_route_buffers(void)
{
    Car_path_count = 0U;
    corner_steps = 0U;
    g_recon_push_route_ready = 0U;
    memset(path, 0, sizeof(path));
    memset(corner_path, 0, sizeof(corner_path));
    memset(&final_paths, 0, sizeof(final_paths));
}

/* 将首车路径构造成“可执行坐标系下的识别任务序列”。 */
static uint8 build_draw_path_first_car_recon_tasks_exec(void)
{
    size_t visit_idx; // 遍历每个识别访问点的索引。

    draw_path_first_car_recon_task_count = 0U;
    draw_path_first_car_corner_steps_exec = 0U;
    memset(draw_path_first_car_recon_tasks, 0, sizeof(draw_path_first_car_recon_tasks));
    memset(draw_path_first_car_corner_path_exec, 0, sizeof(draw_path_first_car_corner_path_exec));

    if (draw_path_first_car_corner_steps > DRAW_PATH_FIRST_CAR_FULL_PATH_CAPACITY ||
        draw_path_first_car_visit_count > DRAW_PATH_FIRST_CAR_MAX_ITEMS)
    {
        return 0U;
    }

    if (draw_path_first_car_corner_steps > 0U)
    {
        memcpy(draw_path_first_car_corner_path_exec,
               draw_path_first_car_corner_path,
               draw_path_first_car_corner_steps * sizeof(draw_path_first_car_corner_path_exec[0]));
        map_planner_to_exec_points(draw_path_first_car_corner_path_exec,
                                   draw_path_first_car_corner_path_exec,
                                   draw_path_first_car_corner_steps);
    }
    draw_path_first_car_corner_steps_exec = draw_path_first_car_corner_steps;

    for (visit_idx = 0U; visit_idx < draw_path_first_car_visit_count; ++visit_idx)
    {
        const DrawPathFirstCarVisitInfo *visit_info = &draw_path_first_car_visit_info[visit_idx]; // 当前访问点规划信息。
        Point reach_exec;                                                                         // 访问点在执行坐标系中的到达点。
        Point anchor_exec;                                                                        // 访问点在执行坐标系中的朝向锚点。
        uint16 corner_index;                                                                      // 该访问点对应的角点索引。

        if (!visit_info->valid)
        {
            return 0U;
        }

        corner_index = draw_path_first_car_visit_corner_indices[visit_idx];
        if (corner_index == DRAW_PATH_FIRST_CAR_CORNER_INDEX_INVALID ||
            corner_index >= draw_path_first_car_corner_steps_exec)
        {
            return 0U;
        }

        reach_exec = map_planner_to_exec_point(visit_info->reach_point);
        anchor_exec = map_planner_to_exec_point(visit_info->anchor_point);

        draw_path_first_car_recon_tasks[visit_idx].valid = 1U;
        draw_path_first_car_recon_tasks[visit_idx].item_type = visit_info->item_type;
        draw_path_first_car_recon_tasks[visit_idx].source_index = visit_info->source_index;
        draw_path_first_car_recon_tasks[visit_idx].corner_index = corner_index;
        draw_path_first_car_recon_tasks[visit_idx].target_yaw_deg_exec =
            map_exec_heading_deg(reach_exec, anchor_exec);
        draw_path_first_car_recon_task_count++;
    }

    return 1U;
}

/* 预览记录首车识别路径执行后的车辆终点（规划坐标），供后续推箱路线衔接。 */
static void recon_capture_preview_final_planner_car(void)
{
    if (draw_path_first_car_full_steps > 0U)
    {
        g_recon_car_after_exec_planner =
            draw_path_first_car_full_path[draw_path_first_car_full_steps - 1U];
    }
    else
    {
        g_recon_car_after_exec_planner = g_recon_car_start_planner;
    }
    g_recon_car_after_exec_valid = 1U;
}

/* 根据固定种子映射生成一个箱子-目标初始分配（用于预规划加速）。 */
static uint8 recon_build_seed_box_target_mapping(size_t *out_mapping)
{
    size_t used_targets[MAX_TARGETS] = {0}; // 目标占用标记，防止重复分配。
    size_t box_idx;                         // 箱子遍历下标。

    if (out_mapping == NULL || g_recon_box_count > g_recon_target_count)
    {
        return 0U;
    }

    for (box_idx = 0U; box_idx < PLANNER_V3_TWO_PHASE_MAX_BOXES; ++box_idx)
    {
        out_mapping[box_idx] = SIZE_MAX;
    }

    for (box_idx = 0U; box_idx < g_recon_box_count; ++box_idx)
    {
        size_t target_idx = g_recon_manual_target_mapping_seed[box_idx]; // 种子映射给出的目标下标。

        if (target_idx >= g_recon_target_count || used_targets[target_idx] != 0U)
        {
            return 0U;
        }

        used_targets[target_idx] = 1U;
        out_mapping[box_idx] = target_idx;
    }

    return 1U;
}

/* 比较两份箱子-目标映射是否完全一致。 */
static uint8 recon_box_target_mapping_equal(const size_t *lhs,
                                            const size_t *rhs,
                                            size_t count)
{
    size_t idx; // 映射逐项比较下标。

    if (lhs == NULL || rhs == NULL)
    {
        return 0U;
    }

    for (idx = 0U; idx < count; ++idx)
    {
        if (lhs[idx] != rhs[idx])
        {
            return 0U;
        }
    }

    return 1U;
}

/* 使用给定映射重新规划“识别后推箱路线”，并可选择保存映射副本。 */
static uint8 recon_plan_push_route_with_mapping(PlannerPointV3_BFS planner_car,
                                                const size_t *mapping,
                                                size_t *saved_mapping)
{
    size_t idx; // 映射复制下标。

    if (mapping == NULL || g_recon_box_count == 0U || g_recon_target_count == 0U)
    {
        return 0U;
    }

    recon_clear_final_route_buffers();

    res = plan_boxes_greedy_v3_manual_assignment_with_return_control(
        MAP_ROWS, MAP_COLS, planner_car,
        g_recon_boxes_snapshot, g_recon_box_count,
        g_recon_targets_snapshot, g_recon_target_count,
        g_recon_bombs_snapshot, g_recon_bomb_count,
        g_recon_obstacles_snapshot, g_recon_obstacle_count,
        mapping,
        NULL,
        path, GREEDY_AREA, &Car_path_count,
        PLANNER_V3_RETURN_TO_START_ENABLE,
        box_target_mapping, &final_paths,
        corner_path, GREEDY_AREA, &corner_steps);
    if (res != 0 || corner_steps == 0U)
    {
        recon_clear_final_route_buffers();
        return 0U;
    }

    map_planner_to_exec_points(corner_path, corner_path, corner_steps);
    g_recon_push_route_ready = 1U;

    if (saved_mapping != NULL)
    {
        for (idx = 0U; idx < PLANNER_V3_TWO_PHASE_MAX_BOXES; ++idx)
        {
            saved_mapping[idx] = (idx < g_recon_box_count) ? mapping[idx] : SIZE_MAX;
        }
    }

    return 1U;
}

/* 复位识别流程的所有运行时状态与缓存。 */
static void recon_reset_state(void)
{
    size_t i; // 通用循环下标。

    g_recon_stage = RECON_STAGE_IDLE;
    g_recon_active = 0U;
    g_recon_complete = 0U;
    g_recon_failed = 0U;
    g_recon_current_visit_idx = 0U;
    memset(g_recon_segment_path, 0, sizeof(g_recon_segment_path));
    memset(g_recon_boxes_snapshot, 0, sizeof(g_recon_boxes_snapshot));
    memset(g_recon_targets_snapshot, 0, sizeof(g_recon_targets_snapshot));
    memset(g_recon_bombs_snapshot, 0, sizeof(g_recon_bombs_snapshot));
    memset(g_recon_obstacles_snapshot, 0, sizeof(g_recon_obstacles_snapshot));
    g_recon_box_count = 0U;
    g_recon_target_count = 0U;
    g_recon_bomb_count = 0U;
    g_recon_obstacle_count = 0U;
    g_recon_car_start_planner.row = 0;
    g_recon_car_start_planner.col = 0;
    g_recon_car_after_exec_planner.row = 0;
    g_recon_car_after_exec_planner.col = 0;
    g_recon_car_after_exec_valid = 0U;
    draw_path_first_car_recon_task_count = 0U;
    draw_path_first_car_corner_steps_exec = 0U;
    memset(draw_path_first_car_recon_tasks, 0, sizeof(draw_path_first_car_recon_tasks));
    memset(draw_path_first_car_corner_path_exec, 0, sizeof(draw_path_first_car_corner_path_exec));
    memset(&first_paths, 0, sizeof(first_paths));
    for (i = 0U; i < PLANNER_V3_TWO_PHASE_MAX_BOXES; ++i)
    {
        g_recon_manual_box_target_mapping[i] = SIZE_MAX;
        g_recon_preplanned_box_target_mapping[i] = SIZE_MAX;
        box_target_mapping[i] = SIZE_MAX;
    }
    clear_draw_path_first_car_paths();
    recon_clear_final_route_buffers();
    recon_status_reset(0U, 0U);
}

/* 从最新地图全局数据抓取一份识别用快照，避免流程中被动态数据改写。 */
static void recon_snapshot_current_map(void)
{
    g_recon_box_count = actual_boxes_count;
    g_recon_target_count = actual_targets_count;
    g_recon_bomb_count = actual_bombs_count;
    g_recon_obstacle_count = actual_obstacles_count;
    g_recon_car_start_planner = car;

    memset(g_recon_boxes_snapshot, 0, sizeof(g_recon_boxes_snapshot));
    memset(g_recon_targets_snapshot, 0, sizeof(g_recon_targets_snapshot));
    memset(g_recon_bombs_snapshot, 0, sizeof(g_recon_bombs_snapshot));
    memset(g_recon_obstacles_snapshot, 0, sizeof(g_recon_obstacles_snapshot));

    if (g_recon_box_count > 0U)
    {
        memcpy(g_recon_boxes_snapshot, boxes, g_recon_box_count * sizeof(g_recon_boxes_snapshot[0]));
    }
    if (g_recon_target_count > 0U)
    {
        memcpy(g_recon_targets_snapshot, targets, g_recon_target_count * sizeof(g_recon_targets_snapshot[0]));
    }
    if (g_recon_bomb_count > 0U)
    {
        memcpy(g_recon_bombs_snapshot, bombs, g_recon_bomb_count * sizeof(g_recon_bombs_snapshot[0]));
    }
    if (g_recon_obstacle_count > 0U)
    {
        memcpy(g_recon_obstacles_snapshot, obstacles, g_recon_obstacle_count * sizeof(g_recon_obstacles_snapshot[0]));
    }
}

/* 构建“第 visit_idx 个识别访问点”对应的路径段。 */
static uint8 recon_build_visit_segment(size_t visit_idx, size_t *out_steps)
{
    size_t start_corner_idx; // 本段起始角点索引（上一访问点角点）。
    size_t end_corner_idx;   // 本段结束角点索引（当前访问点角点）。
    size_t copy_count;       // 本段需要拷贝的角点数量。

    if (out_steps == NULL ||
        visit_idx >= draw_path_first_car_recon_task_count ||
        !draw_path_first_car_recon_tasks[visit_idx].valid)
    {
        return 0U;
    }

    end_corner_idx = draw_path_first_car_recon_tasks[visit_idx].corner_index;
    start_corner_idx = (visit_idx == 0U) ? 0U : draw_path_first_car_recon_tasks[visit_idx - 1U].corner_index;
    if (end_corner_idx < start_corner_idx ||
        end_corner_idx >= draw_path_first_car_corner_steps_exec)
    {
        return 0U;
    }

    copy_count = end_corner_idx - start_corner_idx + 1U;
    if (copy_count == 0U || copy_count > DRAW_PATH_FIRST_CAR_FULL_PATH_CAPACITY)
    {
        return 0U;
    }

    memcpy(g_recon_segment_path,
           &draw_path_first_car_corner_path_exec[start_corner_idx],
           copy_count * sizeof(g_recon_segment_path[0]));
    *out_steps = copy_count;
    return 1U;
}

/* 将一次识别结果按任务类型写回状态表（箱子或目标标签）。 */
static uint8 recon_store_result(const ReconVisitTask *task, int label_value)
{
    if (task == NULL || !task->valid)
    {
        return 0U;
    }

    if (task->item_type == DRAW_PATH_FIRST_CAR_ITEM_BOX)
    {
        if (task->source_index >= g_recon_box_count)
        {
            return 0U;
        }
        return recon_status_store_label(UART_RECON_ITEM_BOX, task->source_index, label_value);
    }

    if (task->item_type == DRAW_PATH_FIRST_CAR_ITEM_TARGET)
    {
        if (task->source_index >= g_recon_target_count)
        {
            return 0U;
        }
        return recon_status_store_label(UART_RECON_ITEM_TARGET, task->source_index, label_value);
    }

    return 0U;
}

/* 检查目标标签是否处于“占位值全 0”场景（识别端未给出有效类别）。 */
static uint8 recon_targets_use_placeholder_label_zero(void)
{
    size_t target_idx;             // 目标遍历下标。
    size_t seen_target_count = 0U; // 收到有效目标标签的数量。

    if (g_recon_target_count == 0U)
    {
        return 0U;
    }

    for (target_idx = 0U; target_idx < g_recon_target_count; ++target_idx)
    {
        if (!recon_status_target_valid[target_idx])
        {
            continue;
        }

        seen_target_count++;
        if (recon_status_target_labels[target_idx] != 0)
        {
            return 0U;
        }
    }

    return (seen_target_count > 0U) ? 1U : 0U;
}

/* 占位标签场景下，使用预设顺序做箱子-目标手工映射回退。 */
static uint8 recon_apply_manual_box_target_mapping(void)
{
    size_t used_targets[MAX_TARGETS] = {0}; // 目标占用标记。
    size_t box_idx;                         // 箱子遍历下标。

    if (g_recon_box_count > g_recon_target_count)
    {
        return 0U;
    }

    for (box_idx = 0U; box_idx < g_recon_box_count; ++box_idx)
    {
        size_t target_idx; // 当前箱子映射到的目标下标。

        target_idx = g_recon_manual_target_mapping_seed[box_idx];
        if (target_idx >= g_recon_target_count || used_targets[target_idx] != 0U)
        {
            return 0U;
        }

        used_targets[target_idx] = 1U;
        box_target_mapping[box_idx] = target_idx;
        g_recon_manual_box_target_mapping[box_idx] = target_idx;
    }

    return 1U;
}

/* 判断单个箱子与单个目标在标签约束下是否可配对。 */
static uint8 recon_box_target_pair_compatible(size_t box_idx, size_t target_idx)
{
    if (box_idx >= g_recon_box_count || target_idx >= g_recon_target_count)
    {
        return 0U;
    }

    if (recon_status_box_valid[box_idx] && recon_status_target_valid[target_idx])
    {
        return (recon_status_box_labels[box_idx] == recon_status_target_labels[target_idx]) ? 1U : 0U;
    }

    return 1U;
}

/* 统计某个箱子在当前“未占用目标集合”下可选候选数。 */
static uint8 recon_count_box_candidates(size_t box_idx, const uint8 *used_targets)
{
    size_t target_idx; // 目标遍历下标。
    uint8 count = 0U;  // 可行候选数量。

    for (target_idx = 0U; target_idx < g_recon_target_count; ++target_idx)
    {
        if (used_targets[target_idx] != 0U)
        {
            continue;
        }
        if (!recon_box_target_pair_compatible(box_idx, target_idx))
        {
            continue;
        }
        count++;
    }

    return count;
}

/* 按候选数从小到大重排箱子顺序，优先处理约束最强项以提升回溯效率。 */
static void recon_sort_box_order_by_candidates(size_t *box_order, size_t count)
{
    size_t i;                              // 外层选择排序下标。
    size_t j;                              // 内层搜索最优候选下标。
    uint8 used_targets[MAX_TARGETS] = {0}; // 预留占位数组（当前实现未标记占用）。

    for (i = 0U; i < count; ++i)
    {
        size_t best = i;                                                                // 当前轮最优位置。
        uint8 best_candidates = recon_count_box_candidates(box_order[i], used_targets); // 当前最小候选数。

        for (j = i + 1U; j < count; ++j)
        {
            uint8 candidate_count = recon_count_box_candidates(box_order[j], used_targets); // 新候选的可选数量。

            if (candidate_count < best_candidates)
            {
                best = j;
                best_candidates = candidate_count;
            }
        }

        if (best != i)
        {
            size_t tmp = box_order[i]; // 交换临时变量。
            box_order[i] = box_order[best];
            box_order[best] = tmp;
        }
    }
}

/* 深度优先回溯搜索唯一的箱子-目标可行映射。 */
static void recon_backtrack_box_target_mapping(const size_t *box_order,
                                               size_t depth,
                                               size_t box_count,
                                               uint8 *used_targets,
                                               size_t *working_mapping,
                                               size_t *resolved_mapping,
                                               uint8 *solution_count)
{
    size_t target_idx; // 尝试分配的目标下标。
    size_t box_idx;    // 当前深度对应的箱子下标。

    if (*solution_count > 1U)
    {
        return;
    }

    if (depth >= box_count)
    {
        memcpy(resolved_mapping,
               working_mapping,
               g_recon_box_count * sizeof(resolved_mapping[0]));
        (*solution_count)++;
        return;
    }

    box_idx = box_order[depth];
    for (target_idx = 0U; target_idx < g_recon_target_count; ++target_idx)
    {
        if (used_targets[target_idx] != 0U)
        {
            continue;
        }
        if (!recon_box_target_pair_compatible(box_idx, target_idx))
        {
            continue;
        }

        used_targets[target_idx] = 1U;
        working_mapping[box_idx] = target_idx;
        recon_backtrack_box_target_mapping(box_order,
                                           depth + 1U,
                                           box_count,
                                           used_targets,
                                           working_mapping,
                                           resolved_mapping,
                                           solution_count);
        working_mapping[box_idx] = SIZE_MAX;
        used_targets[target_idx] = 0U;

        if (*solution_count > 1U)
        {
            return;
        }
    }
}

/* 基于识别标签解析并生成最终箱子-目标映射。 */
static uint8 recon_build_box_target_mapping(void)
{
    size_t box_order[PLANNER_V3_TWO_PHASE_MAX_BOXES] = {SIZE_MAX, SIZE_MAX, SIZE_MAX, SIZE_MAX, SIZE_MAX};        // 箱子搜索顺序。
    size_t working_mapping[PLANNER_V3_TWO_PHASE_MAX_BOXES] = {SIZE_MAX, SIZE_MAX, SIZE_MAX, SIZE_MAX, SIZE_MAX};  // 回溯过程中的临时映射。
    size_t resolved_mapping[PLANNER_V3_TWO_PHASE_MAX_BOXES] = {SIZE_MAX, SIZE_MAX, SIZE_MAX, SIZE_MAX, SIZE_MAX}; // 唯一解映射结果。
    uint8 used_targets[MAX_TARGETS] = {0};                                                                        // 目标占用标记。
    uint8 solution_count = 0U;                                                                                    // 找到的解数量（要求恰好 1 个）。
    size_t box_idx;                                                                                               // 箱子遍历下标。

    if (g_recon_box_count > g_recon_target_count)
    {
        return 0U;
    }

    for (box_idx = 0U; box_idx < PLANNER_V3_TWO_PHASE_MAX_BOXES; ++box_idx)
    {
        box_target_mapping[box_idx] = SIZE_MAX;
        g_recon_manual_box_target_mapping[box_idx] = SIZE_MAX;
        if (box_idx < g_recon_box_count)
        {
            box_order[box_idx] = box_idx;
            working_mapping[box_idx] = SIZE_MAX;
            resolved_mapping[box_idx] = SIZE_MAX;
        }
    }

    if (recon_targets_use_placeholder_label_zero())
    {
        return recon_apply_manual_box_target_mapping();
    }

    recon_sort_box_order_by_candidates(box_order, g_recon_box_count);

    for (box_idx = 0U; box_idx < g_recon_box_count; ++box_idx)
    {
        if (recon_count_box_candidates(box_order[box_idx], used_targets) == 0U)
        {
            return 0U;
        }
    }

    memset(used_targets, 0, sizeof(used_targets));
    recon_backtrack_box_target_mapping(box_order,
                                       0U,
                                       g_recon_box_count,
                                       used_targets,
                                       working_mapping,
                                       resolved_mapping,
                                       &solution_count);
    if (solution_count != 1U)
    {
        return 0U;
    }

    for (box_idx = 0U; box_idx < g_recon_box_count; ++box_idx)
    {
        if (resolved_mapping[box_idx] >= g_recon_target_count)
        {
            return 0U;
        }
        box_target_mapping[box_idx] = resolved_mapping[box_idx];
        g_recon_manual_box_target_mapping[box_idx] = resolved_mapping[box_idx];
    }

    return 1U;
}

/* 在识别阶段真正结束后，读取当前执行位姿并转换到规划坐标保存。 */
static void recon_capture_final_planner_car(void)
{
    path_follow_status_t st = {0};               // 当前路径跟随状态（执行坐标）。
    CarPosition planner_position = {0.0f, 0.0f}; // 换算后的规划坐标位置。

    path_follow_get_status(&st);
    planner_position = map_exec_to_planner_position(st.x_m / GRID_SIZE_M,
                                                    st.y_m / GRID_SIZE_M);
    g_recon_car_after_exec_planner.row = (int8_t)lroundf(planner_position.row);
    g_recon_car_after_exec_planner.col = (int8_t)lroundf(planner_position.col);
    g_recon_car_after_exec_valid = 1U;
}

/* 对外提供“识别后车辆规划坐标终点”读取接口。 */
uint8 recon_get_car_after_exec_planner(PlannerPointV3_BFS *out_pos)
{
    if (out_pos == NULL || !g_recon_car_after_exec_valid)
    {
        return 0U;
    }

    *out_pos = g_recon_car_after_exec_planner;
    return 1U;
}

/* 准备识别阶段路线与状态机：抓快照、建首车路径、生成识别任务。 */
static uint8 recon_prepare_route(void)
{
    size_t seed_mapping[PLANNER_V3_TWO_PHASE_MAX_BOXES] = {SIZE_MAX, SIZE_MAX, SIZE_MAX, SIZE_MAX, SIZE_MAX}; // 预规划的种子映射。

    recon_reset_state();
    recon_snapshot_current_map();
    recon_status_reset(g_recon_box_count, g_recon_target_count);

    if (g_recon_box_count > PLANNER_V3_TWO_PHASE_MAX_BOXES)
    {
        g_recon_failed = 1U;
        g_recon_stage = RECON_STAGE_ERROR;
        return 0U;
    }

    res = build_draw_path_first_car_paths_v3(
        MAP_ROWS, MAP_COLS, g_recon_car_start_planner,
        g_recon_boxes_snapshot, g_recon_box_count,
        g_recon_targets_snapshot, g_recon_target_count,
        g_recon_bombs_snapshot, g_recon_bomb_count,
        g_recon_obstacles_snapshot, g_recon_obstacle_count,
        DRAW_PATH_FIRST_CAR_REACH_DIST_BOTH,
        NULL, 0U, NULL);
    if (res != 0)
    {
        g_recon_failed = 1U;
        g_recon_stage = RECON_STAGE_ERROR;
        return 0U;
    }

    first_paths = draw_path_first_car_paths;

    if (!build_draw_path_first_car_recon_tasks_exec())
    {
        g_recon_failed = 1U;
        g_recon_stage = RECON_STAGE_ERROR;
        return 0U;
    }

    recon_capture_preview_final_planner_car();

    if (recon_build_seed_box_target_mapping(seed_mapping))
    {
        (void)recon_plan_push_route_with_mapping(g_recon_car_after_exec_planner,
                                                 seed_mapping,
                                                 g_recon_preplanned_box_target_mapping);
    }

    g_recon_active = 1U;
    g_recon_stage = RECON_STAGE_LOAD_SEGMENT;
    return 1U;
}

/* 识别流程成功收尾：保存终点并更新状态机标志。 */
static void recon_finalize_success(void)
{
    recon_capture_final_planner_car();
    g_recon_active = 0U;
    g_recon_complete = 1U;
    g_recon_stage = RECON_STAGE_COMPLETE;
}

/* 识别流程失败收尾：关闭识别串口接收并置失败状态。 */
static void recon_fail(void)
{
    data_handle_close_recon_rx();
    g_recon_active = 0U;
    g_recon_failed = 1U;
    g_recon_stage = RECON_STAGE_ERROR;
}

/* 识别阶段状态机主驱动：装载路径段 -> 跟随 -> 转向 -> 等识别结果 -> 下一项。 */
static void handle_recon_phase(void)
{
    path_follow_status_t st = {0}; // 路径跟随实时状态快照。

    /* 识别状态机未激活时直接退出，避免无意义开销。 */
    if (!g_recon_active)
    {
        return;
    }

    /* 没有识别任务时可直接判定成功。 */
    if (draw_path_first_car_recon_task_count == 0U)
    {
        recon_finalize_success();
        return;
    }

    path_follow_get_status(&st);

    switch (g_recon_stage)
    {
    /* 1) 装载本次识别路径段并下发给 path_follow。 */
    case RECON_STAGE_LOAD_SEGMENT:
    {
        size_t segment_steps = 0U; // 当前识别段有效点数。

        /* 全部访问点完成后，基于识别标签解析最终箱子-目标映射。 */
        if (g_recon_current_visit_idx >= draw_path_first_car_recon_task_count)
        {
            if (recon_build_box_target_mapping())
            {
                recon_finalize_success();
            }
            else
            {
                recon_fail();
            }
            break;
        }

        if (!recon_build_visit_segment(g_recon_current_visit_idx, &segment_steps))
        {
            recon_fail();
            break;
        }

        /* 加载路径段前锁定当前朝向，减少识别位姿抖动。 */
        path_follow_hold_current_yaw();
        path_follow_set_path_pause_enabled(g_recon_segment_path,
                                           segment_steps,
                                           RECON_PATH_USE_PAUSE_EVENTS);
        g_recon_stage = RECON_STAGE_FOLLOW_SEGMENT;
        break;
    }

    /* 2) 等路径段执行完成后，转向识别锚点朝向。 */
    case RECON_STAGE_FOLLOW_SEGMENT:
        if (!st.active)
        {
            path_follow_start_rotate_to_yaw(
                draw_path_first_car_recon_tasks[g_recon_current_visit_idx].target_yaw_deg_exec);
            g_recon_stage = RECON_STAGE_TURN_TO_ANCHOR;
        }
        break;

    /* 3) 转向完成后，打开识别接收窗口并发起一次抓拍命令。 */
    case RECON_STAGE_TURN_TO_ANCHOR:
        if (!st.active)
        {
            const ReconVisitTask *task = &draw_path_first_car_recon_tasks[g_recon_current_visit_idx]; // 当前识别任务。
            data_handle_open_recon_rx(task->item_type);
            uart_write_string(UART_4, RECON_REQUEST_CAPTURE_CMD);
            g_recon_stage = RECON_STAGE_WAIT_RESULT;
        }
        break;

    /* 4) 等待识别标签返回，成功后存储并进入下一访问点。 */
    case RECON_STAGE_WAIT_RESULT:
    {
        UartReconResult recon_result = {0}; // UART4 解包后的单条识别结果。

        if (!data_handle_take_recon_result(&recon_result))
        {
            break;
        }
        if (!recon_store_result(&draw_path_first_car_recon_tasks[g_recon_current_visit_idx],
                                recon_result.label_value))
        {
            recon_fail();
            break;
        }

        path_follow_set_target_yaw(draw_path_first_car_recon_tasks[g_recon_current_visit_idx].target_yaw_deg_exec);
        g_recon_current_visit_idx++;
        g_recon_stage = RECON_STAGE_LOAD_SEGMENT;
        break;
    }

    /* 终态与异常态不做动作，等待外层流程处理。 */
    case RECON_STAGE_COMPLETE:
    case RECON_STAGE_ERROR:
    case RECON_STAGE_IDLE:
    default:
        break;
    }
}

/* 清理动态行进校正（CARINIT）流程的中间状态。 */
static void reset_carinit_dynamic_state(uint8 reset_last_idx)
{
    /* reset_last_idx=1 时，连同“已触发过的 target 索引”一起清理。 */
    g_carinit_wait_external = 0;
    g_carinit_forward_check = 0;
    g_carinit_axis = 0;
    g_carinit_bind_target_idx = 0;
    g_carinit_anchor_parallel_m = 0.0f;
    g_carinit_stream_open = 0;
    if (reset_last_idx)
    {
        g_carinit_last_target_idx = -1;
    }
}

/**
 * @brief 根据规划器给出的“推炸弹角点索引”生成暂停点列表，并下发给 path_follow。
 *
 * @param route_corner_count 当前 corner_path 中有效角点数量。
 *
 * @note planner_v3_last_bomb_push_corner_indices 已按炸弹执行顺序排列。
 *       此处会过滤越界索引，并去重，最终设置统一的停顿时长 BOMB_PUSH_SETTLE_PAUSE_MS。
 */
static void configure_bomb_pause_points(size_t route_corner_count)
{
    size_t pause_indices[PATH_FOLLOW_MAX_PAUSE_POINTS]; // 传给 path_follow 的暂停点索引列表。
    size_t pause_count = 0U;                            // 已收集的暂停点数量。
    size_t limit = planner_v3_last_bomb_push_count;     // 炸弹推送事件数量上限。
    size_t i;                                           // 外层遍历下标。

    if (limit > PLANNER_V3_TWO_PHASE_MAX_BOMBS)
    {
        limit = PLANNER_V3_TWO_PHASE_MAX_BOMBS;
    }

    for (i = 0U; i < limit && pause_count < PATH_FOLLOW_MAX_PAUSE_POINTS; ++i)
    {
        size_t pause_idx = planner_v3_last_bomb_push_corner_indices[i]; // 候选暂停角点索引。
        size_t j;                                                       // 去重比较下标。
        uint8 duplicated = 0U;                                          // 是否重复标记。

        if (pause_idx == SIZE_MAX || pause_idx >= route_corner_count)
        {
            continue;
        }

        for (j = 0U; j < pause_count; ++j)
        {
            if (pause_indices[j] == pause_idx)
            {
                duplicated = 1U;
                break;
            }
        }
        if (duplicated)
        {
            continue;
        }

        pause_indices[pause_count++] = pause_idx;
    }

    if (pause_count > 0U)
    {
        path_follow_set_pause_indices(pause_indices, pause_count, BOMB_PUSH_SETTLE_PAUSE_MS);
    }
    else
    {
        path_follow_set_pause_indices(NULL, 0U, 0U);
    }
}

/* 解析当前路段主运动轴，并返回沿主轴剩余距离。 */
static uint8 resolve_motion_axis(const path_follow_status_t *st, float *remain_parallel_m)
{
    if (st->segment_axis == 1U || st->segment_axis == 2U)
    {
        if (remain_parallel_m)
        {
            *remain_parallel_m = (st->segment_axis == 1U)
                                     ? absf_local(st->target_x_m - st->x_m)
                                     : absf_local(st->target_y_m - st->y_m);
        }
        return st->segment_axis;
    }

    float rem_x = absf_local(st->target_x_m - st->x_m); // X 方向剩余距离。
    float rem_y = absf_local(st->target_y_m - st->y_m); // Y 方向剩余距离。

    if (remain_parallel_m)
    {
        *remain_parallel_m = (rem_x >= rem_y) ? rem_x : rem_y;
    }
    return (rem_x >= rem_y) ? 1U : 2U;
}

/* 复位上电定位流程状态，可选择同时清空目标位姿。 */
static void reset_startup_localization_state(car_init_stage_t next_stage, uint8 clear_target)
{
    prestart_move_started = 0;
    g_car_init_stage = next_stage;
    if (clear_target)
    {
        g_car_init_target_m.x = 0.0f;
        g_car_init_target_m.y = 0.0f;
        g_car_init_target_valid = 0U;
    }
}

/* 判断当前估计位姿是否已对齐到外部目标位姿。 */
static uint8 startup_localization_aligned(const path_follow_status_t *st)
{
    if (st == NULL || !g_car_init_target_valid)
    {
        return 0U;
    }

    return (absf_local(st->x_m - g_car_init_target_m.x) < CAR_INIT_ALIGN_TOLERANCE_M &&
            absf_local(st->y_m - g_car_init_target_m.y) < CAR_INIT_ALIGN_TOLERANCE_M)
               ? 1U
               : 0U;
}

/* 完成上电定位收尾：落盘最终位姿并标记流程完成。 */
static void finish_startup_localization(const path_follow_status_t *st)
{
    float yaw_deg = 0.0f; // 完成校准后保留当前朝向角。

    if (st != NULL)
    {
        yaw_deg = st->yaw_deg;
    }

    if (g_car_init_target_valid)
    {
        path_follow_reset_pose(g_car_init_target_m.x, g_car_init_target_m.y, yaw_deg);
    }

    car_init_active = 0;
    car_init_done = 1;
    reset_startup_localization_state(CAR_INIT_STAGE_READY, 0U);
}

/* 上电定位状态机驱动：预动作、等外部位姿、修正、完成。 */
static void handle_startup_localization(void)
{
    path_follow_status_t st = {0}; // 路径跟随状态快照。
    path_follow_get_status(&st);

    /* 未要求上电定位时，状态机回到空闲态。 */
    if (!car_init_required)
    {
        if (g_car_init_stage != CAR_INIT_STAGE_IDLE)
        {
            reset_startup_localization_state(CAR_INIT_STAGE_IDLE, 1U);
        }
        return;
    }

    /* 从 IDLE 进入流程时，先设定一个保守初值位姿。 */
    if (car_init_active && g_car_init_stage == CAR_INIT_STAGE_IDLE)
    {
        path_follow_set_path(NULL, 0U);
        path_follow_reset_pose(CAR_INIT_POSE_X_M, CAR_INIT_POSE_Y_M, st.yaw_deg);
        reset_startup_localization_state(CAR_INIT_STAGE_PRESTART_MOVE, 1U);
    }

    if (!car_init_active)
    {
        return;
    }

    /* 阶段 A：先执行预设小位移，再请求外部定位 START。 */
    if (g_car_init_stage == CAR_INIT_STAGE_PRESTART_MOVE)
    {
        if (!prestart_move_started)
        {
            path_follow_start_offset_move(prestart_move_forward_m - prestart_move_backward_m,
                                          prestart_move_left_m - prestart_move_right_m);
            prestart_move_started = 1;
            return;
        }

        path_follow_get_status(&st);
        if (!st.active && prestart_move_started)
        {
            reset_startup_localization_state(CAR_INIT_STAGE_WAIT_EXTERNAL_POSE, 1U);
            uart_data_processing_enabled = true;
            data_reception_complete = false;
            init_map_received_count = 0;
            current_round_complete = false;
            fifo_clear(&uart_data_fifo);
            car_position_valid = false;
            printf("START\n");
        }
        return;
    }

    /* 阶段 B：等待地图与外部位姿齐备后，决定是否直接完成或进入修正。 */
    if (g_car_init_stage == CAR_INIT_STAGE_WAIT_EXTERNAL_POSE)
    {
        if (!data_reception_complete || !car_position_valid)
        {
            return;
        }

        g_car_init_target_m = car_position_m;
        g_car_init_target_valid = 1U;
        car_position_valid = false;

        if (startup_localization_aligned(&st))
        {
            finish_startup_localization(&st);
        }
        else
        {
            path_follow_start_pose_correction(g_car_init_target_m.x, g_car_init_target_m.y);
            g_car_init_stage = CAR_INIT_STAGE_CORRECTING;
        }
        return;
    }

    /* 阶段 C：持续接收外部位姿并迭代修正，直到收敛到阈值内。 */
    if (g_car_init_stage == CAR_INIT_STAGE_CORRECTING)
    {
        path_follow_get_status(&st);

        if (car_position_valid)
        {
            g_car_init_target_m = car_position_m;
            g_car_init_target_valid = 1U;
            car_position_valid = false;
            if (!startup_localization_aligned(&st))
            {
                path_follow_start_pose_correction(g_car_init_target_m.x, g_car_init_target_m.y);
                path_follow_get_status(&st);
            }
        }

        if (startup_localization_aligned(&st))
        {
            finish_startup_localization(&st);
        }
        else if (!st.active && g_car_init_target_valid)
        {
            path_follow_start_pose_correction(g_car_init_target_m.x, g_car_init_target_m.y);
        }
    }
}

/* 行进过程中的动态位姿校正（CARINIT/CARSTOP 协议交互）。 */
static void handle_dynamic_carinit(void)
{
    path_follow_status_t st = {0}; // 行进状态快照。
    path_follow_get_status(&st);

    /* 动态校正总开关关闭：确保外部流关闭并清理状态。 */
    if (!dynamic_map_enable)
    {
        if (g_carinit_stream_open)
        {
            printf("CARSTOP\n");
        }
        car_position_valid = false;
        reset_carinit_dynamic_state(0U);
        return;
    }

    /* 轨迹未执行时，结束当前 CARINIT 会话。 */
    if (!st.active)
    {
        if (g_carinit_stream_open)
        {
            printf("CARSTOP\n");
            g_carinit_stream_open = 0;
        }
        reset_carinit_dynamic_state(1U);
        return;
    }
    /* 路径暂停（例如炸弹停顿）时不做动态校正。 */
    if (st.paused)
    {
        if (g_carinit_stream_open)
        {
            printf("CARSTOP\n");
        }
        car_position_valid = false;
        reset_carinit_dynamic_state(0U);
        return;
    }
    if (car_init_active || data_control_flag == 0 || corner_steps < 2U)
    {
        return;
    }

    /* 切换到新目标点时，关闭旧的外部流绑定，避免错配位姿。 */
    if (g_carinit_stream_open && st.target_idx != g_carinit_bind_target_idx)
    {
        printf("CARSTOP\n");
        car_position_valid = false;
        g_carinit_stream_open = 0;
        reset_carinit_dynamic_state(0U);
        return;
    }

    {
        float remain_parallel_m = 0.0f;                            // 沿主运动轴到段末的剩余距离。
        uint8 axis = resolve_motion_axis(&st, &remain_parallel_m); // 当前路段主运动轴。

        /* 在接近当前段末端时触发一次 CARINIT，请求外部位姿参与校正。 */
        if ((st.target_idx + 1U) < corner_steps &&
            !g_carinit_wait_external &&
            !g_carinit_forward_check &&
            (int)st.target_idx != g_carinit_last_target_idx &&
            remain_parallel_m <= (GRID_SIZE_M + 0.03f) &&
            remain_parallel_m > 0.03f)
        {
            car_position_valid = false;
            printf("CARINIT\n");
            g_carinit_wait_external = 1;
            g_carinit_axis = axis;
            g_carinit_bind_target_idx = st.target_idx;
            g_carinit_last_target_idx = (int)st.target_idx;
            g_carinit_stream_open = 1;
        }
    }

    if (!car_position_valid)
    {
        return;
    }

    if (!(g_carinit_wait_external || g_carinit_forward_check) || (g_carinit_axis != 1U && g_carinit_axis != 2U))
    {
        car_position_valid = false;
        return;
    }

    /* 读取里程计位姿（odo）与外部位姿（ext），并按主运动轴拆分误差。 */
    {
        float odo_x_m = st.x_m;                                            // 里程计 X（米）。
        float odo_y_m = st.y_m;                                            // 里程计 Y（米）。
        float ext_x_m = car_position_m.x;                                  // 外部定位 X（米）。
        float ext_y_m = car_position_m.y;                                  // 外部定位 Y（米）。
        float odo_parallel_m = (g_carinit_axis == 1U) ? odo_x_m : odo_y_m; // 里程计主轴分量。
        float ext_parallel_m = (g_carinit_axis == 1U) ? ext_x_m : ext_y_m; // 外部定位主轴分量。
        float odo_perp_m = (g_carinit_axis == 1U) ? odo_y_m : odo_x_m;     // 里程计垂轴分量。
        float ext_perp_m = (g_carinit_axis == 1U) ? ext_y_m : ext_x_m;     // 外部定位垂轴分量。
        float parallel_err_m = odo_parallel_m - ext_parallel_m;            // 主轴误差。
        float perp_err_m = odo_perp_m - ext_perp_m;                        // 垂轴误差。

        float new_x_m = odo_x_m;    // 计算后的新 X（默认沿用里程计）。
        float new_y_m = odo_y_m;    // 计算后的新 Y（默认沿用里程计）。
        uint8 need_pose_update = 0; // 是否执行 path_follow_reset_pose。

        /* 垂直主运动轴方向偏差过大时，直接对齐到外部位姿。 */
        if (absf_local(perp_err_m) > 0.02f)
        {
            if (g_carinit_axis == 1U)
            {
                new_y_m = ext_perp_m;
            }
            else
            {
                new_x_m = ext_perp_m;
            }
            need_pose_update = 1;
        }

        /* 正在做前向一致性检查：外部位姿回到锚点附近时再确认更新。 */
        if (g_carinit_forward_check)
        {
            if (absf_local(ext_parallel_m - g_carinit_anchor_parallel_m) < 0.02f)
            {
                if (g_carinit_axis == 1U)
                {
                    new_x_m = ext_parallel_m;
                }
                else
                {
                    new_y_m = ext_parallel_m;
                }
                need_pose_update = 1;
                g_carinit_forward_check = 0;
                g_carinit_wait_external = 0;
            }
        }
        /* 里程计领先太多：先记录锚点，等待下一帧确认再更新。 */
        else if (parallel_err_m > 0.05f)
        {
            g_carinit_anchor_parallel_m = odo_parallel_m;
            g_carinit_forward_check = 1;
            g_carinit_wait_external = 1;
        }
        /* 里程计落后：允许“只向前拉齐”，并限制每次最大修正量。 */
        else if (parallel_err_m < -0.04f)
        {
            /* When odometry lags behind, only pull the estimate forward. */
            float pull_m = (-parallel_err_m) * 0.5f;
            if (pull_m > 0.05f)
            {
                pull_m = 0.05f;
            }
            if (pull_m < 0.01f)
            {
                pull_m = 0.01f;
            }

            if (g_carinit_axis == 1U)
            {
                new_x_m = odo_parallel_m + pull_m;
                if (new_x_m > ext_parallel_m)
                {
                    new_x_m = ext_parallel_m;
                }
            }
            else
            {
                new_y_m = odo_parallel_m + pull_m;
                if (new_y_m > ext_parallel_m)
                {
                    new_y_m = ext_parallel_m;
                }
            }
            need_pose_update = 1;
            g_carinit_wait_external = 0;
        }
        else
        {
            g_carinit_wait_external = 0;
        }

        if (need_pose_update)
        {
            /* 仅在必要时重置位姿，避免高频抖动。 */
            path_follow_reset_pose(new_x_m, new_y_m, st.yaw_deg);
        }
    }

    car_position_valid = false;
}
