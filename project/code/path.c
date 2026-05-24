#include "path.h"
#include <math.h>
#include <string.h>

/* 斜线捷径的安全净空，单位为“格”。障碍/箱子/炸弹/目标点中心距离斜线小于该值时禁止走斜线。 */
#define PATH_DIAGONAL_CLEARANCE_CELL 1.0001f
/* 动态规划压缩执行路径时使用的“无穷大”代价值。 */
#define PATH_COST_INF 1.0e30f
/* 动态规划前驱索引的无效值，MAX_CAR_PATH 远小于 0xFFFF。 */
#define PATH_INDEX_NONE 0xFFFFU

/* 以下缓冲只在 path_build_exec_from_planner() 中使用。
 * 做成静态全局，是为了避免在嵌入式栈上临时申请较大的 DP 表。 */
static Position s_raw_path[MAX_CAR_PATH] = {{0}};
static float s_path_cost[MAX_CAR_PATH] = {0.0f};
static uint16 s_prev_index[MAX_CAR_PATH] = {0U};
static uint16 s_rebuild_indices[MAX_CAR_PATH] = {0U};
static uint16 s_mandatory_prefix[MAX_CAR_PATH + 1U] = {0U};
static uint16 s_push_edge_prefix[MAX_CAR_PATH + 1U] = {0U};
/* 运行期开关：1 允许动态规划选择斜线捷径，0 时只保留水平/竖直执行段。 */
static uint8 g_path_diagonal_enabled = 1U;

void path_remap_exec_point(Position *p)
{
    uint8 temp = 0U;

    if (p == NULL)
    {
        return;
    }

#if PATH_COORD_TRANSPOSE_COMPENSATE
    temp = p->row;
    p->row = p->col;
    p->col = temp;
#endif

#if PATH_COORD_FLIP_VERTICAL
    p->col = (uint8)((MAP_ROWS - 1U) - p->col);
#endif
}

void path_inverse_remap_exec_point(Position *p)
{
    uint8 temp = 0U;

    if (p == NULL)
    {
        return;
    }

#if PATH_COORD_FLIP_VERTICAL
    p->col = (uint8)((MAP_ROWS - 1U) - p->col);
#endif

#if PATH_COORD_TRANSPOSE_COMPENSATE
    temp = p->row;
    p->row = p->col;
    p->col = temp;
#endif
}

void path_set_diagonal_enabled(uint8 enabled)
{
    g_path_diagonal_enabled = (enabled != 0U) ? 1U : 0U;
}

uint8 path_get_diagonal_enabled(void)
{
    return g_path_diagonal_enabled;
}

static uint8 path_is_same_grid_cell(const Position *a, const Position *b)
{
    if (a == NULL || b == NULL)
    {
        return 0U;
    }
    return (a->row == b->row && a->col == b->col) ? 1U : 0U;
}

/**
 * @brief 给路径点事件 id 分配保留优先级。
 *
 * 规划层有时会在同一个格子上连续写入多个点，例如普通转折点和爆炸点重合。
 * 压缩前会合并连续重复格子，此时必须保留更关键的事件 id：
 * 爆炸点、识别点、推箱子起止点都会影响上层状态机或 path_follow 的暂停事件，
 * 优先级必须高于普通转折点。
 */
static uint8 path_marker_priority(uint8 marker_id)
{
    if (marker_id == BOMB_EXPLOSION)
    {
        return 5U;
    }
    if (marker_id == IDENTIFICATION)
    {
        return 4U;
    }
    if (marker_id == PUSH_END_POINT)
    {
        return 3U;
    }
    if (marker_id == PUSH_StART_POINT)
    {
        return 2U;
    }
    if (marker_id == TURNING_POINT)
    {
        return 1U;
    }
    return 0U;
}

static uint8 path_merge_marker(uint8 old_marker, uint8 new_marker)
{
    return (path_marker_priority(new_marker) >=
            path_marker_priority(old_marker)) ? new_marker : old_marker;
}

static uint8 path_is_required_exec_marker(uint8 marker_id)
{
    return (marker_id == IDENTIFICATION ||
            marker_id == BOMB_EXPLOSION ||
            marker_id == PUSH_StART_POINT ||
            marker_id == PUSH_END_POINT) ? 1U : 0U;
}

static uint8 path_is_push_span_end_marker(uint8 marker_id)
{
    return (marker_id == PUSH_END_POINT ||
            marker_id == BOMB_EXPLOSION) ? 1U : 0U;
}

static uint8 path_is_map_cell_valid(const Position *p)
{
    if (p == NULL)
    {
        return 0U;
    }
    return (p->row < MAP_ROWS && p->col < MAP_COLS) ? 1U : 0U;
}

static uint8 path_is_slanted_segment(const Position *a, const Position *b)
{
    if (a == NULL || b == NULL)
    {
        return 0U;
    }
    return (a->row != b->row && a->col != b->col) ? 1U : 0U;
}

static float path_grid_segment_distance(const Position *a, const Position *b)
{
    float d_row = 0.0f;
    float d_col = 0.0f;

    if (a == NULL || b == NULL)
    {
        return 0.0f;
    }

    d_row = (float)((int32)b->row - (int32)a->row);
    d_col = (float)((int32)b->col - (int32)a->col);
    return sqrtf(d_row * d_row + d_col * d_col);
}

/**
 * @brief 计算一个格子中心到执行线段的最短距离平方，单位为“格^2”。
 */
static float path_cell_to_segment_distance_sq(const Position *cell,
                                              const Position *seg_start,
                                              const Position *seg_end)
{
    float ar = 0.0f;
    float ac = 0.0f;
    float br = 0.0f;
    float bc = 0.0f;
    float pr = 0.0f;
    float pc = 0.0f;
    float dr = 0.0f;
    float dc = 0.0f;
    float len_sq = 0.0f;
    float t = 0.0f;
    float near_r = 0.0f;
    float near_c = 0.0f;
    float err_r = 0.0f;
    float err_c = 0.0f;

    if (cell == NULL || seg_start == NULL || seg_end == NULL)
    {
        return PATH_COST_INF;
    }

    ar = (float)seg_start->row;
    ac = (float)seg_start->col;
    br = (float)seg_end->row;
    bc = (float)seg_end->col;
    pr = (float)cell->row;
    pc = (float)cell->col;
    dr = br - ar;
    dc = bc - ac;
    len_sq = dr * dr + dc * dc;
    if (len_sq <= 1e-6f)
    {
        err_r = pr - ar;
        err_c = pc - ac;
        return err_r * err_r + err_c * err_c;
    }

    t = ((pr - ar) * dr + (pc - ac) * dc) / len_sq;
    if (t < 0.0f)
    {
        t = 0.0f;
    }
    else if (t > 1.0f)
    {
        t = 1.0f;
    }

    near_r = ar + t * dr;
    near_c = ac + t * dc;
    err_r = pr - near_r;
    err_c = pc - near_c;
    return err_r * err_r + err_c * err_c;
}

static uint8 path_blocker_list_near_segment(const Position *list,
                                            size_t count,
                                            size_t capacity,
                                            const Position *seg_start,
                                            const Position *seg_end)
{
    size_t i = 0U;
    size_t limit = count;
    float clearance_sq = PATH_DIAGONAL_CLEARANCE_CELL *
                         PATH_DIAGONAL_CLEARANCE_CELL;

    if (list == NULL || seg_start == NULL || seg_end == NULL)
    {
        return 0U;
    }
    if (limit > capacity)
    {
        limit = capacity;
    }

    for (i = 0U; i < limit; i++)
    {
        if (!path_is_map_cell_valid(&list[i]))
        {
            continue;
        }
        if (path_cell_to_segment_distance_sq(&list[i], seg_start, seg_end) <= clearance_sq)
        {
            return 1U;
        }
    }

    return 0U;
}

static uint8 path_blocker_list_contains_cell(const Position *list,
                                             size_t count,
                                             size_t capacity,
                                             uint8 row,
                                             uint8 col)
{
    size_t i = 0U;
    size_t limit = count;

    if (list == NULL)
    {
        return 0U;
    }
    if (limit > capacity)
    {
        limit = capacity;
    }

    for (i = 0U; i < limit; i++)
    {
        if (path_is_map_cell_valid(&list[i]) &&
            list[i].row == row && list[i].col == col)
        {
            return 1U;
        }
    }

    return 0U;
}

/**
 * @brief 判断某个地图格是否被不可穿越对象占用。
 *
 * include_targets 为 1 时，目标点也会作为占用物处理。这样做更保守，
 * 可以避免横竖捷径直接穿过箱子目标点、炸弹目标点等语义位置。
 */
static uint8 path_cell_has_blocker(const path_map_snapshot_t *current_map,
                                   const path_map_snapshot_t *extra_map,
                                   uint8 row,
                                   uint8 col,
                                   uint8 include_targets)
{
    if (row >= MAP_ROWS || col >= MAP_COLS || current_map == NULL)
    {
        return 1U;
    }

    if (path_blocker_list_contains_cell(current_map->obstacles_buf,
                                        current_map->obstacles_count,
                                        MAX_OBSTACLES,
                                        row,
                                        col) ||
        path_blocker_list_contains_cell(current_map->boxes_buf,
                                        current_map->boxes_count,
                                        MAX_BOXES,
                                        row,
                                        col) ||
        path_blocker_list_contains_cell(current_map->bombs_buf,
                                        current_map->bombs_count,
                                        MAX_BOMBS,
                                        row,
                                        col))
    {
        return 1U;
    }

    if (include_targets &&
        path_blocker_list_contains_cell(current_map->targets_buf,
                                        current_map->targets_count,
                                        MAX_TARGETS,
                                        row,
                                        col))
    {
        return 1U;
    }

    if (extra_map != NULL)
    {
        if (path_blocker_list_contains_cell(extra_map->obstacles_buf,
                                            extra_map->obstacles_count,
                                            MAX_OBSTACLES,
                                            row,
                                            col) ||
            path_blocker_list_contains_cell(extra_map->boxes_buf,
                                            extra_map->boxes_count,
                                            MAX_BOXES,
                                            row,
                                            col) ||
            path_blocker_list_contains_cell(extra_map->bombs_buf,
                                            extra_map->bombs_count,
                                            MAX_BOMBS,
                                            row,
                                            col))
        {
            return 1U;
        }
        if (include_targets &&
            path_blocker_list_contains_cell(extra_map->targets_buf,
                                            extra_map->targets_count,
                                            MAX_TARGETS,
                                            row,
                                            col))
        {
            return 1U;
        }
    }

    return 0U;
}

static uint8 path_axis_segment_has_clearance(const path_map_snapshot_t *current_map,
                                             const path_map_snapshot_t *extra_map,
                                             const Position *seg_start,
                                             const Position *seg_end)
{
    int32 row = 0;
    int32 col = 0;
    int32 end = 0;
    int32 step = 0;

    if (!path_is_map_cell_valid(seg_start) ||
        !path_is_map_cell_valid(seg_end))
    {
        return 0U;
    }

    if (seg_start->row == seg_end->row)
    {
        if (seg_start->col == seg_end->col)
        {
            return 1U;
        }

        row = (int32)seg_start->row;
        step = (seg_end->col > seg_start->col) ? 1 : -1;
        col = (int32)seg_start->col + step;
        end = (int32)seg_end->col;
        while (col != end)
        {
            if (row < 0 || col < 0 || row >= (int32)MAP_ROWS || col >= (int32)MAP_COLS)
            {
                return 0U;
            }
            if (path_cell_has_blocker(current_map, extra_map, (uint8)row, (uint8)col, 1U))
            {
                return 0U;
            }
            col += step;
        }
        return 1U;
    }

    if (seg_start->col == seg_end->col)
    {
        if (seg_start->row == seg_end->row)
        {
            return 1U;
        }

        col = (int32)seg_start->col;
        step = (seg_end->row > seg_start->row) ? 1 : -1;
        row = (int32)seg_start->row + step;
        end = (int32)seg_end->row;
        while (row != end)
        {
            if (row < 0 || col < 0 || row >= (int32)MAP_ROWS || col >= (int32)MAP_COLS)
            {
                return 0U;
            }
            if (path_cell_has_blocker(current_map, extra_map, (uint8)row, (uint8)col, 1U))
            {
                return 0U;
            }
            row += step;
        }
        return 1U;
    }

    return 0U;
}

static uint8 path_raw_subpath_is_axis_run(const Position *path,
                                          size_t start_idx,
                                          size_t end_idx)
{
    const Position *start = NULL;
    const Position *end = NULL;
    size_t k = 0U;
    int32 expected_row_step = 0;
    int32 expected_col_step = 0;
    int32 prev_row = 0;
    int32 prev_col = 0;

    if (path == NULL || start_idx >= end_idx)
    {
        return 0U;
    }

    start = &path[start_idx];
    end = &path[end_idx];
    if (!path_is_map_cell_valid(start) ||
        !path_is_map_cell_valid(end))
    {
        return 0U;
    }

    if (start->row == end->row && start->col != end->col)
    {
        expected_col_step = (end->col > start->col) ? 1 : -1;
    }
    else if (start->col == end->col && start->row != end->row)
    {
        expected_row_step = (end->row > start->row) ? 1 : -1;
    }
    else
    {
        return 0U;
    }

    prev_row = (int32)start->row;
    prev_col = (int32)start->col;
    for (k = start_idx + 1U; k <= end_idx; k++)
    {
        int32 curr_row = (int32)path[k].row;
        int32 curr_col = (int32)path[k].col;
        int32 delta_row = curr_row - prev_row;
        int32 delta_col = curr_col - prev_col;

        if (!path_is_map_cell_valid(&path[k]))
        {
            return 0U;
        }

        if (expected_col_step != 0)
        {
            if (curr_row != (int32)start->row || delta_row != 0)
            {
                return 0U;
            }
            if ((expected_col_step > 0 && delta_col <= 0) ||
                (expected_col_step < 0 && delta_col >= 0))
            {
                return 0U;
            }
        }
        else
        {
            if (curr_col != (int32)start->col || delta_col != 0)
            {
                return 0U;
            }
            if ((expected_row_step > 0 && delta_row <= 0) ||
                (expected_row_step < 0 && delta_row >= 0))
            {
                return 0U;
            }
        }

        prev_row = curr_row;
        prev_col = curr_col;
    }

    return 1U;
}

static uint8 path_slanted_segment_has_clearance(const path_map_snapshot_t *current_map,
                                                const path_map_snapshot_t *extra_map,
                                                const Position *seg_start,
                                                const Position *seg_end)
{
    if (current_map == NULL ||
        !path_is_map_cell_valid(seg_start) ||
        !path_is_map_cell_valid(seg_end))
    {
        return 0U;
    }

    if (path_blocker_list_near_segment(current_map->obstacles_buf,
                                       current_map->obstacles_count,
                                       MAX_OBSTACLES,
                                       seg_start,
                                       seg_end) ||
        path_blocker_list_near_segment(current_map->boxes_buf,
                                       current_map->boxes_count,
                                       MAX_BOXES,
                                       seg_start,
                                       seg_end) ||
        path_blocker_list_near_segment(current_map->bombs_buf,
                                       current_map->bombs_count,
                                       MAX_BOMBS,
                                       seg_start,
                                       seg_end) ||
        path_blocker_list_near_segment(current_map->targets_buf,
                                       current_map->targets_count,
                                       MAX_TARGETS,
                                       seg_start,
                                       seg_end))
    {
        return 0U;
    }

    if (extra_map != NULL)
    {
        /* 额外快照通常保存规划前地图；当前快照通常是规划模拟后的地图。
         * 两套地图都检查，可以覆盖推箱/炸墙前后的占用，避免斜线捷径撞到动态对象。 */
        if (path_blocker_list_near_segment(extra_map->obstacles_buf,
                                           extra_map->obstacles_count,
                                           MAX_OBSTACLES,
                                           seg_start,
                                           seg_end) ||
            path_blocker_list_near_segment(extra_map->boxes_buf,
                                           extra_map->boxes_count,
                                           MAX_BOXES,
                                           seg_start,
                                           seg_end) ||
            path_blocker_list_near_segment(extra_map->bombs_buf,
                                           extra_map->bombs_count,
                                           MAX_BOMBS,
                                           seg_start,
                                           seg_end) ||
            path_blocker_list_near_segment(extra_map->targets_buf,
                                           extra_map->targets_count,
                                           MAX_TARGETS,
                                           seg_start,
                                           seg_end))
        {
            return 0U;
        }
    }

    return 1U;
}

/**
 * @brief 为路径事件和推箱子区间建立前缀和表。
 */
static void path_prepare_prefix(const Position *path, size_t count)
{
    size_t i = 0U;
    uint16 mandatory_count = 0U;
    uint16 push_edge_count = 0U;
    uint8 push_active = 0U;

    memset(s_mandatory_prefix, 0, sizeof(s_mandatory_prefix));
    memset(s_push_edge_prefix, 0, sizeof(s_push_edge_prefix));

    if (path == NULL || count == 0U)
    {
        return;
    }

    for (i = 0U; i < count; i++)
    {
        if (path_is_required_exec_marker(path[i].id))
        {
            mandatory_count++;
        }
        s_mandatory_prefix[i + 1U] = mandatory_count;
    }

    for (i = 0U; i < count; i++)
    {
        uint8 edge_is_push = 0U;

        if (i + 1U < count)
        {
            if (path_is_push_span_end_marker(path[i].id))
            {
                push_active = 0U;
            }
            if (path[i].id == PUSH_StART_POINT)
            {
                push_active = 1U;
            }

            edge_is_push = push_active;
            if (path_is_push_span_end_marker(path[i + 1U].id))
            {
                push_active = 0U;
            }
        }

        if (edge_is_push)
        {
            push_edge_count++;
        }
        s_push_edge_prefix[i + 1U] = push_edge_count;
    }
}

static uint8 path_segment_skips_required_marker(size_t start_idx, size_t end_idx)
{
    if (end_idx <= start_idx + 1U)
    {
        return 0U;
    }
    return ((s_mandatory_prefix[end_idx] -
             s_mandatory_prefix[start_idx + 1U]) > 0U) ? 1U : 0U;
}

static uint8 path_segment_crosses_push_span(size_t start_idx, size_t end_idx)
{
    if (end_idx <= start_idx)
    {
        return 0U;
    }
    return ((s_push_edge_prefix[end_idx] -
             s_push_edge_prefix[start_idx]) > 0U) ? 1U : 0U;
}

/**
 * @brief 判断动态规划中的一个 i->j 候选执行段是否允许下发给 path_follow。
 */
static uint8 path_exec_segment_allowed(const Position *path,
                                       size_t count,
                                       const path_map_snapshot_t *current_map,
                                       const path_map_snapshot_t *extra_map,
                                       size_t start_idx,
                                       size_t end_idx)
{
    const Position *seg_start = NULL;
    const Position *seg_end = NULL;
    uint8 is_slanted = 0U;
    uint8 crosses_push_span = 0U;
    uint8 follows_raw_axis_run = 0U;

    if (path == NULL || start_idx >= count || end_idx >= count || start_idx >= end_idx)
    {
        return 0U;
    }
    if (path_segment_skips_required_marker(start_idx, end_idx))
    {
        return 0U;
    }

    seg_start = &path[start_idx];
    seg_end = &path[end_idx];
    if (!path_is_map_cell_valid(seg_start) ||
        !path_is_map_cell_valid(seg_end))
    {
        return 0U;
    }

    is_slanted = path_is_slanted_segment(seg_start, seg_end);
    if (is_slanted && !g_path_diagonal_enabled)
    {
        return 0U;
    }

    if (end_idx == start_idx + 1U)
    {
        return 1U;
    }

    crosses_push_span = path_segment_crosses_push_span(start_idx, end_idx);
    follows_raw_axis_run = path_raw_subpath_is_axis_run(path, start_idx, end_idx);

    if (crosses_push_span)
    {
        if (is_slanted)
        {
            return 0U;
        }
        return follows_raw_axis_run;
    }

    if (follows_raw_axis_run)
    {
        return 1U;
    }
    if (is_slanted)
    {
        return path_slanted_segment_has_clearance(current_map, extra_map, seg_start, seg_end);
    }

    return path_axis_segment_has_clearance(current_map, extra_map, seg_start, seg_end);
}

uint8 path_build_exec_from_planner(const Position *planner_path,
                                   size_t planner_steps,
                                   const path_map_snapshot_t *current_map,
                                   const path_map_snapshot_t *extra_map,
                                   Position *exec_path,
                                   size_t exec_capacity,
                                   size_t *exec_steps)
{
    size_t i = 0U;
    size_t j = 0U;
    size_t raw_steps = 0U;
    size_t rebuild_count = 0U;
    size_t out_steps = 0U;
    uint16 idx = 0U;

    if (exec_steps != NULL)
    {
        *exec_steps = 0U;
    }
    if (exec_path == NULL || exec_steps == NULL || planner_path == NULL ||
        current_map == NULL || exec_capacity < 2U ||
        planner_steps < 2U || planner_steps > MAX_CAR_PATH)
    {
        return 0U;
    }

    memset(exec_path, 0, sizeof(Position) * exec_capacity);
    memset(s_raw_path, 0, sizeof(s_raw_path));
    memset(s_path_cost, 0, sizeof(s_path_cost));
    memset(s_prev_index, 0, sizeof(s_prev_index));
    memset(s_rebuild_indices, 0, sizeof(s_rebuild_indices));

    /* 第一步：复制原始规划点，并合并连续重复格子的事件 id。 */
    for (i = 0U; i < planner_steps; i++)
    {
        Position raw = planner_path[i];

        if (!path_is_map_cell_valid(&raw))
        {
            return 0U;
        }

        if (raw_steps == 0U)
        {
            s_raw_path[raw_steps++] = raw;
            continue;
        }

        if (path_is_same_grid_cell(&s_raw_path[raw_steps - 1U], &raw))
        {
            s_raw_path[raw_steps - 1U].id =
                path_merge_marker(s_raw_path[raw_steps - 1U].id, raw.id);
            continue;
        }
        if (raw_steps >= MAX_CAR_PATH)
        {
            return 0U;
        }
        s_raw_path[raw_steps++] = raw;
    }

    if (raw_steps < 2U)
    {
        return 0U;
    }

    /* 第二步：建立事件点/推箱区间前缀和，供候选线段 O(1) 查询。 */
    path_prepare_prefix(s_raw_path, raw_steps);

    /* 第三步：动态规划求最短可执行路径。 */
    for (i = 0U; i < raw_steps; i++)
    {
        s_path_cost[i] = PATH_COST_INF;
        s_prev_index[i] = PATH_INDEX_NONE;
    }
    s_path_cost[0] = 0.0f;

    for (j = 1U; j < raw_steps; j++)
    {
        for (i = 0U; i < j; i++)
        {
            float candidate_cost = 0.0f;

            if (s_path_cost[i] >= PATH_COST_INF * 0.5f)
            {
                continue;
            }
            if (!path_exec_segment_allowed(s_raw_path,
                                           raw_steps,
                                           current_map,
                                           extra_map,
                                           i,
                                           j))
            {
                continue;
            }

            candidate_cost = s_path_cost[i] +
                             path_grid_segment_distance(&s_raw_path[i], &s_raw_path[j]);
            if (candidate_cost < s_path_cost[j])
            {
                s_path_cost[j] = candidate_cost;
                s_prev_index[j] = (uint16)i;
            }
        }
    }

    if (s_prev_index[raw_steps - 1U] == PATH_INDEX_NONE)
    {
        return 0U;
    }

    /* 第四步：从终点反向重建被选中的原始路径点。 */
    idx = (uint16)(raw_steps - 1U);
    while (rebuild_count < MAX_CAR_PATH)
    {
        s_rebuild_indices[rebuild_count++] = idx;
        if (idx == 0U)
        {
            break;
        }
        idx = s_prev_index[idx];
        if (idx == PATH_INDEX_NONE)
        {
            return 0U;
        }
    }

    if (rebuild_count < 2U ||
        s_rebuild_indices[rebuild_count - 1U] != 0U)
    {
        return 0U;
    }

    /* 第五步：最终执行点 remap 到 path_follow 使用的执行坐标系。 */
    for (i = 0U; i < rebuild_count; i++)
    {
        Position mapped = s_raw_path[s_rebuild_indices[rebuild_count - 1U - i]];
        if (out_steps >= exec_capacity)
        {
            return 0U;
        }
        path_remap_exec_point(&mapped);
        exec_path[out_steps++] = mapped;
    }

    *exec_steps = out_steps;
    return 1U;
}
