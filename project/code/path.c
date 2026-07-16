#include "path.h"
#include <math.h>
#include <string.h>

/* 斜线捷径的安全净空，单位为“格”。障碍/箱子/炸弹/目标点中心距离斜线小于该值时禁止走斜线。 */
#define PATH_DIAGONAL_CLEARANCE_CELL 1.0001f
/* 动态规划压缩执行路径时使用的“无穷大”代价值。 */
#define PATH_COST_INF 1.0e30f
/* 动态规划前驱索引的无效值，MAX_CAR_PATH 远小于 0xFFFF。 */
#define PATH_INDEX_NONE 0xFFFFU
/* 初始阶段 + 每个箱子/炸弹至多产生一次推运结束阶段。 */
#define PATH_STAGE_SNAPSHOT_CAPACITY (MAX_BOXES + MAX_BOMBS + 1U)
#define PATH_DYNAMIC_OBJECT_NONE 0U
#define PATH_DYNAMIC_OBJECT_BOX  1U
#define PATH_DYNAMIC_OBJECT_BOMB 2U

/* 以下缓冲只在 path_build_exec_from_planner() 中使用。
 * 做成静态全局，是为了避免在嵌入式栈上临时申请较大的 DP 表。 */
static Position s_raw_path[MAX_CAR_PATH] = {{0}};
static float s_path_cost[MAX_CAR_PATH] = {0.0f};
static uint16 s_prev_index[MAX_CAR_PATH] = {0U};
static uint16 s_rebuild_indices[MAX_CAR_PATH] = {0U};
static uint16 s_mandatory_prefix[MAX_CAR_PATH + 1U] = {0U};
static uint16 s_push_edge_prefix[MAX_CAR_PATH + 1U] = {0U};
static uint16 s_last_required_before[MAX_CAR_PATH] = {0U};
static Position s_safe_relocation_raw_path[PATH_SAFE_RELOCATION_MAX_POINTS] = {{0}};
static path_map_snapshot_t s_stage_maps[PATH_STAGE_SNAPSHOT_CAPACITY];
static uint8 s_raw_stage_index[MAX_CAR_PATH] = {0U};
static uint8 s_stage_model_valid = 0U;
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
 * @brief 合并同一格上的路径事件位，保留该点发生的全部事件。
 */
static uint8 path_merge_marker(uint8 old_marker, uint8 new_marker)
{
    return (uint8)((old_marker | new_marker) & PATH_ALL_EVENTS);
}

static uint8 path_is_required_exec_marker(uint8 marker_id)
{
    return ((marker_id & PATH_REQUIRED_EVENTS) != 0U) ? 1U : 0U;
}

static uint8 path_is_push_span_end_marker(uint8 marker_id)
{
    return ((marker_id & (PUSH_END_POINT | BOMB_EXPLOSION)) != 0U) ? 1U : 0U;
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

static int32 path_abs_i32(int32 value)
{
    return (value < 0) ? -value : value;
}

static int32 path_find_position_index(const Position *list,
                                      size_t count,
                                      size_t capacity,
                                      uint8 row,
                                      uint8 col)
{
    size_t i = 0U;

    if (list == NULL || count > capacity)
    {
        return -1;
    }
    for (i = 0U; i < count; i++)
    {
        if (list[i].row == row && list[i].col == col)
        {
            return (int32)i;
        }
    }
    return -1;
}

static void path_remove_position_index(Position *list,
                                       size_t *count,
                                       size_t capacity,
                                       size_t index)
{
    size_t i = 0U;

    if (list == NULL || count == NULL || *count > capacity || index >= *count)
    {
        return;
    }
    for (i = index + 1U; i < *count; i++)
    {
        list[i - 1U] = list[i];
    }
    (*count)--;
    if (*count < capacity)
    {
        memset(&list[*count], 0, sizeof(Position));
    }
}

static uint8 path_snapshot_counts_valid(const path_map_snapshot_t *map)
{
    if (map == NULL)
    {
        return 0U;
    }
    return (map->obstacles_count <= MAX_OBSTACLES &&
            map->boxes_count <= MAX_BOXES &&
            map->targets_count <= MAX_TARGETS &&
            map->bombs_count <= MAX_BOMBS) ? 1U : 0U;
}

static uint8 path_position_sets_equal(const Position *a,
                                      size_t a_count,
                                      const Position *b,
                                      size_t b_count,
                                      size_t capacity)
{
    size_t i = 0U;

    if (a == NULL || b == NULL || a_count != b_count ||
        a_count > capacity || b_count > capacity)
    {
        return 0U;
    }
    for (i = 0U; i < a_count; i++)
    {
        if (path_find_position_index(b, b_count, capacity,
                                     a[i].row, a[i].col) < 0)
        {
            return 0U;
        }
    }
    for (i = 0U; i < b_count; i++)
    {
        if (path_find_position_index(a, a_count, capacity,
                                     b[i].row, b[i].col) < 0)
        {
            return 0U;
        }
    }
    return 1U;
}

static void path_apply_bomb_explosion(path_map_snapshot_t *map,
                                      uint8 bomb_row,
                                      uint8 bomb_col)
{
    size_t read_index = 0U;
    size_t write_index = 0U;
    size_t old_count = 0U;

    if (map == NULL || map->obstacles_count > MAX_OBSTACLES)
    {
        return;
    }
    old_count = map->obstacles_count;
    for (read_index = 0U; read_index < old_count; read_index++)
    {
        int32 d_row = (int32)map->obstacles_buf[read_index].row - (int32)bomb_row;
        int32 d_col = (int32)map->obstacles_buf[read_index].col - (int32)bomb_col;

        if (path_abs_i32(d_row) <= 1 && path_abs_i32(d_col) <= 1)
        {
            continue;
        }
        map->obstacles_buf[write_index++] = map->obstacles_buf[read_index];
    }
    map->obstacles_count = write_index;
    while (write_index < old_count)
    {
        memset(&map->obstacles_buf[write_index], 0, sizeof(Position));
        write_index++;
    }
}

/**
 * 按 PUSH_END/BOMB_EXPLOSION 事件从规划前地图重建各阶段占用状态。
 * 重建失败时返回 0，调用方继续采用规划前后快照并集的保守策略。
 */
static uint8 path_build_stage_snapshots(const Position *path,
                                        size_t count,
                                        const path_map_snapshot_t *pre_map,
                                        const path_map_snapshot_t *post_map)
{
    path_map_snapshot_t working;
    size_t i = 0U;
    size_t stage_count = 1U;
    uint8 push_active = 0U;
    uint8 object_type = PATH_DYNAMIC_OBJECT_NONE;
    uint8 object_row = 0U;
    uint8 object_col = 0U;
    int32 last_dir_row = 0;
    int32 last_dir_col = 0;

    s_stage_model_valid = 0U;
    memset(s_raw_stage_index, 0, sizeof(s_raw_stage_index));
    memset(s_stage_maps, 0, sizeof(s_stage_maps));

    if (path == NULL || count == 0U || count > MAX_CAR_PATH ||
        !path_snapshot_counts_valid(pre_map) ||
        !path_snapshot_counts_valid(post_map))
    {
        return 0U;
    }

    working = *pre_map;
    s_stage_maps[0] = working;

    for (i = 0U; i < count; i++)
    {
        uint8 events = (uint8)(path[i].id & PATH_ALL_EVENTS);

        s_raw_stage_index[i] = (uint8)(stage_count - 1U);

        /* 先结束上一段推运，再允许同一点开启下一段推运。 */
        if ((events & PUSH_END_POINT) != 0U)
        {
            int32 object_index = -1;
            int32 final_row = 0;
            int32 final_col = 0;

            if (!push_active || (last_dir_row == 0 && last_dir_col == 0))
            {
                return 0U;
            }
            final_row = (int32)path[i].row + last_dir_row;
            final_col = (int32)path[i].col + last_dir_col;
            if (final_row < 0 || final_col < 0 ||
                final_row >= (int32)MAP_ROWS || final_col >= (int32)MAP_COLS)
            {
                return 0U;
            }

            if (object_type == PATH_DYNAMIC_OBJECT_BOX)
            {
                int32 target_index = -1;

                object_index = path_find_position_index(working.boxes_buf,
                                                        working.boxes_count,
                                                        MAX_BOXES,
                                                        object_row,
                                                        object_col);
                if (object_index < 0 || (events & BOMB_EXPLOSION) != 0U)
                {
                    return 0U;
                }
                target_index = path_find_position_index(working.targets_buf,
                                                        working.targets_count,
                                                        MAX_TARGETS,
                                                        (uint8)final_row,
                                                        (uint8)final_col);
                if (target_index < 0)
                {
                    return 0U;
                }
                /* 与 Game_logic 一致：送达目标的虚拟箱子和目标点同时退场。 */
                path_remove_position_index(working.boxes_buf,
                                           &working.boxes_count,
                                           MAX_BOXES,
                                           (size_t)object_index);
                path_remove_position_index(working.targets_buf,
                                           &working.targets_count,
                                           MAX_TARGETS,
                                           (size_t)target_index);
            }
            else if (object_type == PATH_DYNAMIC_OBJECT_BOMB)
            {
                object_index = path_find_position_index(working.bombs_buf,
                                                        working.bombs_count,
                                                        MAX_BOMBS,
                                                        object_row,
                                                        object_col);
                if (object_index < 0)
                {
                    return 0U;
                }
                working.bombs_buf[object_index].row = (uint8)final_row;
                working.bombs_buf[object_index].col = (uint8)final_col;
                if ((events & BOMB_EXPLOSION) != 0U)
                {
                    path_remove_position_index(working.bombs_buf,
                                               &working.bombs_count,
                                               MAX_BOMBS,
                                               (size_t)object_index);
                    path_apply_bomb_explosion(&working,
                                              (uint8)final_row,
                                              (uint8)final_col);
                }
            }
            else
            {
                return 0U;
            }

            if (stage_count >= PATH_STAGE_SNAPSHOT_CAPACITY)
            {
                return 0U;
            }
            working.car_pose_grid = path[i];
            s_stage_maps[stage_count] = working;
            s_raw_stage_index[i] = (uint8)stage_count;
            stage_count++;
            push_active = 0U;
            object_type = PATH_DYNAMIC_OBJECT_NONE;
            last_dir_row = 0;
            last_dir_col = 0;
        }
        else if ((events & BOMB_EXPLOSION) != 0U)
        {
            /* 当前规划协议中爆炸必须与被推炸弹的 PUSH_END 同点。 */
            return 0U;
        }

        if ((events & PUSH_START_POINT) != 0U)
        {
            int32 d_row = 0;
            int32 d_col = 0;
            int32 ahead_row = 0;
            int32 ahead_col = 0;
            int32 box_index = -1;
            int32 bomb_index = -1;

            if (push_active || i + 1U >= count)
            {
                return 0U;
            }
            d_row = (int32)path[i + 1U].row - (int32)path[i].row;
            d_col = (int32)path[i + 1U].col - (int32)path[i].col;
            if (path_abs_i32(d_row) + path_abs_i32(d_col) != 1)
            {
                return 0U;
            }
            ahead_row = (int32)path[i].row + d_row;
            ahead_col = (int32)path[i].col + d_col;
            if (ahead_row < 0 || ahead_col < 0 ||
                ahead_row >= (int32)MAP_ROWS || ahead_col >= (int32)MAP_COLS)
            {
                return 0U;
            }
            box_index = path_find_position_index(working.boxes_buf,
                                                 working.boxes_count,
                                                 MAX_BOXES,
                                                 (uint8)ahead_row,
                                                 (uint8)ahead_col);
            bomb_index = path_find_position_index(working.bombs_buf,
                                                  working.bombs_count,
                                                  MAX_BOMBS,
                                                  (uint8)ahead_row,
                                                  (uint8)ahead_col);
            if ((box_index >= 0) == (bomb_index >= 0))
            {
                return 0U;
            }
            object_type = (box_index >= 0) ? PATH_DYNAMIC_OBJECT_BOX :
                                             PATH_DYNAMIC_OBJECT_BOMB;
            object_row = (uint8)ahead_row;
            object_col = (uint8)ahead_col;
            last_dir_row = d_row;
            last_dir_col = d_col;
            push_active = 1U;
        }

        if (push_active && i + 1U < count)
        {
            int32 d_row = (int32)path[i + 1U].row - (int32)path[i].row;
            int32 d_col = (int32)path[i + 1U].col - (int32)path[i].col;

            if (path_abs_i32(d_row) + path_abs_i32(d_col) != 1)
            {
                return 0U;
            }
            last_dir_row = d_row;
            last_dir_col = d_col;
        }
    }

    if (push_active)
    {
        return 0U;
    }

    /* 全部动态对象的重建结果必须与规划器最终状态一致。 */
    if (!path_position_sets_equal(working.obstacles_buf,
                                  working.obstacles_count,
                                  post_map->obstacles_buf,
                                  post_map->obstacles_count,
                                  MAX_OBSTACLES) ||
        !path_position_sets_equal(working.bombs_buf,
                                  working.bombs_count,
                                  post_map->bombs_buf,
                                  post_map->bombs_count,
                                  MAX_BOMBS) ||
        !path_position_sets_equal(working.boxes_buf,
                                  working.boxes_count,
                                  post_map->boxes_buf,
                                  post_map->boxes_count,
                                  MAX_BOXES) ||
        !path_position_sets_equal(working.targets_buf,
                                  working.targets_count,
                                  post_map->targets_buf,
                                  post_map->targets_count,
                                  MAX_TARGETS))
    {
        return 0U;
    }

    s_stage_model_valid = 1U;
    return 1U;
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
    uint16 last_required_index = PATH_INDEX_NONE;
    uint8 push_active = 0U;

    memset(s_mandatory_prefix, 0, sizeof(s_mandatory_prefix));
    memset(s_push_edge_prefix, 0, sizeof(s_push_edge_prefix));
    memset(s_last_required_before, 0xFF, sizeof(s_last_required_before));

    if (path == NULL || count == 0U)
    {
        return;
    }

    for (i = 0U; i < count; i++)
    {
        s_last_required_before[i] = last_required_index;
        if (path_is_required_exec_marker(path[i].id))
        {
            mandatory_count++;
            last_required_index = (uint16)i;
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
            if ((path[i].id & PUSH_START_POINT) != 0U)
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
    return (s_mandatory_prefix[end_idx] !=
            s_mandatory_prefix[start_idx + 1U]) ? 1U : 0U;
}

static uint8 path_segment_crosses_push_span(size_t start_idx, size_t end_idx)
{
    if (end_idx <= start_idx)
    {
        return 0U;
    }
    return (s_push_edge_prefix[end_idx] !=
            s_push_edge_prefix[start_idx]) ? 1U : 0U;
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
    const path_map_snapshot_t *clearance_map = current_map;
    const path_map_snapshot_t *clearance_extra_map = extra_map;
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

    crosses_push_span = path_segment_crosses_push_span(start_idx, end_idx);
    follows_raw_axis_run = path_raw_subpath_is_axis_run(path, start_idx, end_idx);

    if (s_stage_model_valid)
    {
        if (s_raw_stage_index[start_idx] >= PATH_STAGE_SNAPSHOT_CAPACITY ||
            s_raw_stage_index[end_idx] >= PATH_STAGE_SNAPSHOT_CAPACITY)
        {
            return 0U;
        }
        if (!crosses_push_span &&
            s_raw_stage_index[start_idx] != s_raw_stage_index[end_idx])
        {
            return 0U;
        }
        clearance_map = &s_stage_maps[s_raw_stage_index[start_idx]];
        clearance_extra_map = NULL;
    }

    if (end_idx == start_idx + 1U)
    {
        return 1U;
    }

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
        return path_slanted_segment_has_clearance(clearance_map,
                                                   clearance_extra_map,
                                                   seg_start,
                                                   seg_end);
    }

    return path_axis_segment_has_clearance(clearance_map,
                                           clearance_extra_map,
                                           seg_start,
                                           seg_end);
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
    memset(s_last_required_before, 0xFF, sizeof(s_last_required_before));
    s_stage_model_valid = 0U;

    /* 第一步：复制原始规划点，并合并连续重复格子的事件 id。 */
    for (i = 0U; i < planner_steps; i++)
    {
        Position raw = planner_path[i];

        if (!path_is_map_cell_valid(&raw))
        {
            return 0U;
        }
        raw.id &= PATH_ALL_EVENTS;

        if (i > 0U)
        {
            int32 d_row = (int32)planner_path[i].row -
                          (int32)planner_path[i - 1U].row;
            int32 d_col = (int32)planner_path[i].col -
                          (int32)planner_path[i - 1U].col;

            /* 原始规划路径只能原地合并事件或移动到四邻接格。 */
            if (path_abs_i32(d_row) + path_abs_i32(d_col) > 1)
            {
                return 0U;
            }
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
    /* 成功时按事件阶段判定捷径；失败时使用规划前/后地图并集安全回退。 */
    (void)path_build_stage_snapshots(s_raw_path,
                                     raw_steps,
                                     extra_map,
                                     current_map);

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
        size_t first_candidate = 0U;
        uint16 required_anchor = s_last_required_before[j];

        if (required_anchor != PATH_INDEX_NONE)
        {
            first_candidate = (size_t)required_anchor;
        }

        for (i = first_candidate; i < j; i++)
        {
            float segment_cost = 0.0f;
            float candidate_cost = 0.0f;

            if (s_path_cost[i] >= PATH_COST_INF * 0.5f)
            {
                continue;
            }

            /*
             * 必经事件点已经把候选起点收窄到 last_required_before[j] 之后。
             * 再先做代价下界剪枝，只有可能优于当前最优解的候选段才进入
             * path_exec_segment_allowed() 的障碍/净空检查，降低长路径 DP 开销。
             */
            segment_cost = path_grid_segment_distance(&s_raw_path[i], &s_raw_path[j]);
            candidate_cost = s_path_cost[i] + segment_cost;
            if (candidate_cost >= s_path_cost[j])
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

static uint8 path_cell_has_adjacent_wall(const path_map_snapshot_t *map,
                                         uint8 row,
                                         uint8 col)
{
    static const int8 d_row[5] = {0, -1, 0, 1, 0};
    static const int8 d_col[5] = {0, 0, 1, 0, -1};
    size_t i = 0U;

    if (!path_snapshot_counts_valid(map) || row >= MAP_ROWS || col >= MAP_COLS)
    {
        return 1U;
    }
    for (i = 0U; i < 5U; i++)
    {
        int32 check_row = (int32)row + (int32)d_row[i];
        int32 check_col = (int32)col + (int32)d_col[i];

        if (check_row < 0 || check_col < 0 ||
            check_row >= (int32)MAP_ROWS || check_col >= (int32)MAP_COLS)
        {
            continue;
        }
        if (path_blocker_list_contains_cell(map->obstacles_buf,
                                            map->obstacles_count,
                                            MAX_OBSTACLES,
                                            (uint8)check_row,
                                            (uint8)check_col))
        {
            return 1U;
        }
    }
    return 0U;
}

uint8 path_build_nearest_wall_clear_exec(const Position *start_map,
                                         const path_map_snapshot_t *map,
                                         Position *exec_path,
                                         size_t exec_capacity,
                                         size_t *exec_steps)
{
    static const int8 d_row[4] = {0, -1, 0, 1};
    static const int8 d_col[4] = {1, 0, -1, 0};
    const size_t cell_count = (size_t)PATH_SAFE_RELOCATION_MAX_POINTS;
    size_t i = 0U;
    size_t head = 0U;
    size_t tail = 0U;
    size_t reverse_count = 0U;
    size_t raw_steps = 0U;
    uint16 start_index = 0U;
    uint16 target_index = PATH_INDEX_NONE;
    uint16 cursor = PATH_INDEX_NONE;

    if (exec_steps != NULL)
    {
        *exec_steps = 0U;
    }
    if (start_map == NULL || map == NULL || exec_path == NULL ||
        exec_steps == NULL || exec_capacity == 0U ||
        !path_is_map_cell_valid(start_map) ||
        !path_snapshot_counts_valid(map) ||
        cell_count == 0U || cell_count > MAX_CAR_PATH ||
        cell_count >= PATH_INDEX_NONE)
    {
        return 0U;
    }

    memset(exec_path, 0, exec_capacity * sizeof(Position));
    memset(s_safe_relocation_raw_path, 0, sizeof(s_safe_relocation_raw_path));
    for (i = 0U; i < cell_count; i++)
    {
        s_prev_index[i] = PATH_INDEX_NONE;
    }

    start_index = (uint16)((uint16)start_map->row * (uint16)MAP_COLS +
                           (uint16)start_map->col);
    s_prev_index[start_index] = start_index;
    s_rebuild_indices[tail++] = start_index;

    while (head < tail)
    {
        uint16 current = s_rebuild_indices[head++];
        uint8 row = (uint8)(current / (uint16)MAP_COLS);
        uint8 col = (uint8)(current % (uint16)MAP_COLS);

        if (!path_cell_has_blocker(map, NULL, row, col, 1U) &&
            !path_cell_has_adjacent_wall(map, row, col))
        {
            target_index = current;
            break;
        }

        for (i = 0U; i < 4U; i++)
        {
            int32 next_row = (int32)row + (int32)d_row[i];
            int32 next_col = (int32)col + (int32)d_col[i];
            uint16 next_index = 0U;

            if (next_row < 0 || next_col < 0 ||
                next_row >= (int32)MAP_ROWS || next_col >= (int32)MAP_COLS ||
                path_cell_has_blocker(map, NULL,
                                      (uint8)next_row,
                                      (uint8)next_col,
                                      1U))
            {
                continue;
            }
            next_index = (uint16)((uint16)next_row * (uint16)MAP_COLS +
                                  (uint16)next_col);
            if (s_prev_index[next_index] != PATH_INDEX_NONE)
            {
                continue;
            }
            if (tail >= cell_count)
            {
                return 0U;
            }
            s_prev_index[next_index] = current;
            s_rebuild_indices[tail++] = next_index;
        }
    }

    if (target_index == PATH_INDEX_NONE)
    {
        return 0U;
    }

    cursor = target_index;
    while (reverse_count < cell_count)
    {
        s_rebuild_indices[reverse_count++] = cursor;
        if (cursor == start_index)
        {
            break;
        }
        cursor = s_prev_index[cursor];
        if (cursor == PATH_INDEX_NONE)
        {
            return 0U;
        }
    }
    if (reverse_count == 0U ||
        s_rebuild_indices[reverse_count - 1U] != start_index)
    {
        return 0U;
    }

    for (i = 0U; i < reverse_count; i++)
    {
        uint16 map_index = s_rebuild_indices[reverse_count - 1U - i];

        s_safe_relocation_raw_path[raw_steps].row =
            (uint8)(map_index / (uint16)MAP_COLS);
        s_safe_relocation_raw_path[raw_steps].col =
            (uint8)(map_index % (uint16)MAP_COLS);
        s_safe_relocation_raw_path[raw_steps].id = PATH_EVENT_NONE;
        raw_steps++;
    }

    if (raw_steps == 1U)
    {
        exec_path[0] = s_safe_relocation_raw_path[0];
        path_remap_exec_point(&exec_path[0]);
        *exec_steps = 1U;
        return 1U;
    }

    return path_build_exec_from_planner(s_safe_relocation_raw_path,
                                        raw_steps,
                                        map,
                                        map,
                                        exec_path,
                                        exec_capacity,
                                        exec_steps);
}
