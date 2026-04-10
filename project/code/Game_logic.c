#include "Game_logic.h"
#include <string.h>
#include <limits.h>

/*
 * Game_logic.c
 * 上层调度模块（对 Algorithm 的封装）：
 * 1) 建立 pair（箱子-目标点）任务表，并维护任务状态；
 * 2) 每轮在“推箱任务”和“炸弹开路任务”之间做统一选择；
 * 3) 执行动作后更新全局地图状态，持续迭代直到任务结束；
 *
 * 设计目标：
 * - 保证必要路径优先被打开，避免关键炸弹被误消耗；
 * - 在可行前提下尽量减少总步数；
 * - 把高耗时搜索交给 Algorithm，本文件只做轻量决策与状态管理。
 */

#define ACTION_BOMB 1
#define ACTION_BOX 2

#define PAIR_STATUS_NORMAL   0
#define PAIR_STATUS_LATE     1
#define PAIR_STATUS_OBSTACLE 2
#define MAX_UNLOCK_WALL_CANDIDATES 8
#define MAX_BOMB_ACTION_CANDIDATES 8
#define FAST_BOX_DIRECT_PICK_STEPS 22
#define FAST_UNLOCK_BOMB_TOPK_PASS1 4
#define FAST_UNLOCK_WALL_TOPK_PASS1 4
#define PAIR_REACH_CACHE_SIZE 96
#define INFER_WALL_CACHE_SIZE 192

typedef struct
{
    Position obstacles_state[MAX_OBSTACLES];
    Position bombs_state[MAX_BOMBS];
    Position boxes_state[MAX_BOXES];
    Position targets_state[MAX_TARGETS];
    int obstacles_cnt;
    int bombs_cnt;
    int boxes_cnt;
    int targets_cnt;
    Position car_state;
} planning_state_t;

typedef struct
{
    uint8 valid;
    uint8 target_done;
    uint8 box_id_ref;
    uint8 status; /* NORMAL / LATE / OBSTACLE */
    Position target_ref;
} pair_task_t;

typedef struct
{
    pair_task_t item[MAX_TARGETS];
    int count;
} pair_task_table_t;

typedef struct
{
    int valid;
    int action_type; /* ACTION_BOMB / ACTION_BOX */
    int steps;
    int box_index;  /* ACTION_BOX 使用 */
    int pair_index; /* 关联的 pair，下标来自 pair_task_table_t */
    path_plan_result plan;

    /* 若动作中消耗了“辅助炸弹”，记录其真实坐标。 */
    uint8 has_support_bomb_pos;
    Position support_bomb_pos;

    /* 炸弹任务中的“主炸弹”（被推到墙上触发爆破的炸弹）。 */
    uint8 has_primary_bomb_pos;
    Position primary_bomb_pos;
} round_action_t;

typedef struct
{
    uint8 valid;
    uint32 state_sig;
    uint8 require_same_id;
    uint8 box_id_ref;
    Position target_ref;
    uint8 reachable;
} pair_reach_cache_entry_t;

typedef struct
{
    uint8 valid;
    uint32 state_sig;
    uint8 require_same_id;
    uint8 box_id_ref;
    Position target_ref;
    Position bomb_ref;
    uint8 found;
    Position wall_ref;
} infer_wall_cache_entry_t;

typedef struct
{
    uint8 valid;
    uint32 state_sig;
    uint32 pairs_sig;
    uint8 require_same_id;
    int critical_owner[MAX_BOMBS];
} critical_owner_cache_entry_t;

static pair_reach_cache_entry_t s_pair_reach_cache[PAIR_REACH_CACHE_SIZE];
static infer_wall_cache_entry_t s_infer_wall_cache[INFER_WALL_CACHE_SIZE];
static critical_owner_cache_entry_t s_critical_owner_cache;
static int s_pair_reach_cache_next = 0;
static int s_infer_wall_cache_next = 0;

static void reset_planning_caches(void)
{
    memset(s_pair_reach_cache, 0, sizeof(s_pair_reach_cache));
    memset(s_infer_wall_cache, 0, sizeof(s_infer_wall_cache));
    memset(&s_critical_owner_cache, 0, sizeof(s_critical_owner_cache));
    s_pair_reach_cache_next = 0;
    s_infer_wall_cache_next = 0;
}

static void apply_box_action_result(planning_state_t *state,
                                    pair_task_table_t *pairs,
                                    const round_action_t *action,
                                    Position *merged_path,
                                    int *merged_len);
static void apply_bomb_action_result(planning_state_t *state,
                                     const round_action_t *action,
                                     Position *merged_path,
                                     int *merged_len);
static int pair_needs_unlock_in_state(const planning_state_t *state,
                                      const pair_task_t *pair,
                                      int require_same_id);

static int same_cell(Position a, Position b)
{
    return (a.row == b.row) && (a.col == b.col);
}

static int is_valid_cell(Position p)
{
    return (p.row < MAP_ROWS) && (p.col < MAP_COLS);
}

static uint8 marker_priority(uint8 marker_id)
{
    if (marker_id == BOMB_EXPLOSION)
        return 3;
    if (marker_id == IDENTIFICATION)
        return 2;
    if (marker_id == TURNING_POINT)
        return 1;
    return 0;
}

static uint8 merge_marker(uint8 old_marker, uint8 new_marker)
{
    return (marker_priority(new_marker) >= marker_priority(old_marker)) ? new_marker : old_marker;
}

static int manhattan_cell_dist(Position a, Position b)
{
    int dr = (int)a.row - (int)b.row;
    int dc = (int)a.col - (int)b.col;
    if (dr < 0)
        dr = -dr;
    if (dc < 0)
        dc = -dc;
    return dr + dc;
}

static int same_position(Position a, Position b)
{
    return (a.row == b.row) && (a.col == b.col) && (a.id == b.id);
}

static uint32 hash_mix_u32(uint32 h, uint32 v)
{
    h ^= v;
    h *= 16777619u;
    return h;
}

static uint32 hash_mix_position(uint32 h, Position p)
{
    uint32 v = ((uint32)p.row << 16) ^ ((uint32)p.col << 8) ^ (uint32)p.id;
    return hash_mix_u32(h, v);
}

static uint32 hash_state_signature(const planning_state_t *state)
{
    int i;
    uint32 h = 2166136261u;
    if (!state)
        return 0;

    h = hash_mix_u32(h, (uint32)state->obstacles_cnt);
    h = hash_mix_u32(h, (uint32)state->bombs_cnt);
    h = hash_mix_u32(h, (uint32)state->boxes_cnt);
    h = hash_mix_u32(h, (uint32)state->targets_cnt);
    h = hash_mix_position(h, state->car_state);

    for (i = 0; i < state->obstacles_cnt; i++)
        h = hash_mix_position(h, state->obstacles_state[i]);
    for (i = 0; i < state->bombs_cnt; i++)
        h = hash_mix_position(h, state->bombs_state[i]);
    for (i = 0; i < state->boxes_cnt; i++)
        h = hash_mix_position(h, state->boxes_state[i]);
    for (i = 0; i < state->targets_cnt; i++)
        h = hash_mix_position(h, state->targets_state[i]);

    return h;
}

static uint32 hash_pairs_signature(const pair_task_table_t *pairs)
{
    int i;
    uint32 h = 2166136261u;
    if (!pairs)
        return 0;

    h = hash_mix_u32(h, (uint32)pairs->count);
    for (i = 0; i < pairs->count; i++)
    {
        const pair_task_t *pair = &pairs->item[i];
        uint32 meta = ((uint32)pair->valid << 24) |
                      ((uint32)pair->target_done << 16) |
                      ((uint32)pair->box_id_ref << 8) |
                      (uint32)pair->status;
        h = hash_mix_u32(h, meta);
        h = hash_mix_position(h, pair->target_ref);
    }
    return h;
}

static int lookup_pair_reach_cache(uint32 state_sig,
                                   uint8 box_id_ref,
                                   Position target_ref,
                                   int require_same_id,
                                   int *out_reachable)
{
    int i;
    if (!out_reachable)
        return 0;

    for (i = 0; i < PAIR_REACH_CACHE_SIZE; i++)
    {
        const pair_reach_cache_entry_t *e = &s_pair_reach_cache[i];
        if (!e->valid)
            continue;
        if (e->state_sig != state_sig)
            continue;
        if (e->require_same_id != (uint8)(require_same_id ? 1 : 0))
            continue;
        if (e->box_id_ref != box_id_ref)
            continue;
        if (!same_position(e->target_ref, target_ref))
            continue;

        *out_reachable = e->reachable ? 1 : 0;
        return 1;
    }
    return 0;
}

static void store_pair_reach_cache(uint32 state_sig,
                                   uint8 box_id_ref,
                                   Position target_ref,
                                   int require_same_id,
                                   int reachable)
{
    pair_reach_cache_entry_t *e;

    if (s_pair_reach_cache_next < 0 || s_pair_reach_cache_next >= PAIR_REACH_CACHE_SIZE)
        s_pair_reach_cache_next = 0;
    e = &s_pair_reach_cache[s_pair_reach_cache_next++];
    if (s_pair_reach_cache_next >= PAIR_REACH_CACHE_SIZE)
        s_pair_reach_cache_next = 0;

    e->valid = 1;
    e->state_sig = state_sig;
    e->require_same_id = (uint8)(require_same_id ? 1 : 0);
    e->box_id_ref = box_id_ref;
    e->target_ref = target_ref;
    e->reachable = (uint8)(reachable ? 1 : 0);
}

static int lookup_infer_wall_cache(uint32 state_sig,
                                   uint8 box_id_ref,
                                   Position target_ref,
                                   int require_same_id,
                                   Position bomb_ref,
                                   int *out_found,
                                   Position *out_wall_ref)
{
    int i;
    if (!out_found || !out_wall_ref)
        return 0;

    for (i = 0; i < INFER_WALL_CACHE_SIZE; i++)
    {
        const infer_wall_cache_entry_t *e = &s_infer_wall_cache[i];
        if (!e->valid)
            continue;
        if (e->state_sig != state_sig)
            continue;
        if (e->require_same_id != (uint8)(require_same_id ? 1 : 0))
            continue;
        if (e->box_id_ref != box_id_ref)
            continue;
        if (!same_position(e->target_ref, target_ref))
            continue;
        if (!same_position(e->bomb_ref, bomb_ref))
            continue;

        *out_found = e->found ? 1 : 0;
        *out_wall_ref = e->wall_ref;
        return 1;
    }
    return 0;
}

static void store_infer_wall_cache(uint32 state_sig,
                                   uint8 box_id_ref,
                                   Position target_ref,
                                   int require_same_id,
                                   Position bomb_ref,
                                   int found,
                                   Position wall_ref)
{
    infer_wall_cache_entry_t *e;

    if (s_infer_wall_cache_next < 0 || s_infer_wall_cache_next >= INFER_WALL_CACHE_SIZE)
        s_infer_wall_cache_next = 0;
    e = &s_infer_wall_cache[s_infer_wall_cache_next++];
    if (s_infer_wall_cache_next >= INFER_WALL_CACHE_SIZE)
        s_infer_wall_cache_next = 0;

    e->valid = 1;
    e->state_sig = state_sig;
    e->require_same_id = (uint8)(require_same_id ? 1 : 0);
    e->box_id_ref = box_id_ref;
    e->target_ref = target_ref;
    e->bomb_ref = bomb_ref;
    e->found = (uint8)(found ? 1 : 0);
    e->wall_ref = wall_ref;
}

static int box_can_match_target(Position box, Position target, int require_same_id)
{
    if (!require_same_id)
        return 1;
    return box.id == target.id;
}

static void remove_position_at(Position *arr, int *cnt, int index)
{
    if (!arr || !cnt || *cnt <= 0)
        return;
    if (index < 0 || index >= *cnt)
        return;

    if (index < *cnt - 1)
    {
        memmove(&arr[index], &arr[index + 1], (size_t)(*cnt - index - 1) * sizeof(Position));
    }

    arr[*cnt - 1].row = 0;
    arr[*cnt - 1].col = 0;
    arr[*cnt - 1].id = 0;
    (*cnt)--;
}

static int find_position_index(const Position *arr, int cnt, Position target)
{
    int i;
    for (i = 0; i < cnt; i++)
    {
        if (same_cell(arr[i], target))
            return i;
    }
    return -1;
}

static int find_box_index_by_id(const Position *boxes_arr, int boxes_cnt, uint8 box_id)
{
    int i;
    for (i = 0; i < boxes_cnt; i++)
    {
        if (boxes_arr[i].id == box_id)
            return i;
    }
    return -1;
}

static int position_in_set(Position p, const Position *set, int set_cnt)
{
    int i;
    if (!set || set_cnt <= 0)
        return 0;
    for (i = 0; i < set_cnt; i++)
    {
        if (same_cell(p, set[i]))
            return 1;
    }
    return 0;
}

static void remove_bomb_by_position(Position *bombs_arr, int *bombs_cnt, Position bomb_ref)
{
    int idx;
    if (!bombs_arr || !bombs_cnt || *bombs_cnt <= 0)
        return;
    idx = find_position_index(bombs_arr, *bombs_cnt, bomb_ref);
    if (idx >= 0)
    {
        remove_position_at(bombs_arr, bombs_cnt, idx);
    }
}

static void append_segment_path(Position *merged_path, int *merged_len,
                                const Position *segment_path, int segment_len)
{
    int start_index = 0;
    int copy_len;
    int available;

    if (!merged_path || !merged_len || !segment_path || segment_len <= 0)
        return;
    if (*merged_len < 0)
        *merged_len = 0;
    if (*merged_len >= MAX_CAR_PATH)
        return;

    if (*merged_len > 0 && same_cell(merged_path[*merged_len - 1], segment_path[0]))
    {
        merged_path[*merged_len - 1].id = merge_marker(merged_path[*merged_len - 1].id, segment_path[0].id);
        start_index = 1;
    }

    copy_len = segment_len - start_index;
    if (copy_len <= 0)
        return;

    available = MAX_CAR_PATH - *merged_len;
    if (copy_len > available)
        copy_len = available;
    if (copy_len <= 0)
        return;

    memcpy(&merged_path[*merged_len],
           &segment_path[start_index],
           (size_t)copy_len * sizeof(Position));
    *merged_len += copy_len;
}

/*
 * 璋冪敤搴曞眰鍗曠瑙勫垝锛? * - allowed_bombs锛氬厑璁稿弬涓庣偢寮规嵎寰勭殑鐐稿脊闆嗗悎锛? * - 鍏朵綑鐐稿脊鎸夐殰纰嶇墿澶勭悊锛岄伩鍏嶁€滆繃婊ゅ悗娑堝け鈥濄€? */
static int plan_box_with_candidates(const planning_state_t *state,
                                    int box_index,
                                    const Position *allowed_bombs, int allowed_bombs_cnt,
                                    const Position *targets, int targets_cnt,
                                    Position car_start,
                                    path_plan_result *out_plan)
{
    Position blocked_obstacles[grid_size];
    int blocked_obstacles_cnt;
    int i;

    if (!state || !targets || targets_cnt <= 0 || !out_plan)
        return -1;
    if (box_index < 0 || box_index >= state->boxes_cnt)
        return -1;

    blocked_obstacles_cnt = state->obstacles_cnt;
    memcpy(blocked_obstacles,
           state->obstacles_state,
           (size_t)blocked_obstacles_cnt * sizeof(Position));

    for (i = 0; i < state->bombs_cnt; i++)
    {
        Position b = state->bombs_state[i];
        if (!position_in_set(b, allowed_bombs, allowed_bombs_cnt))
        {
            if (blocked_obstacles_cnt < grid_size)
                blocked_obstacles[blocked_obstacles_cnt++] = b;
        }
    }

    return integrated_path_output(MAP_ROWS, MAP_COLS,
                                  blocked_obstacles, blocked_obstacles_cnt,
                                  allowed_bombs, allowed_bombs_cnt,
                                  state->boxes_state, state->boxes_cnt,
                                  targets, targets_cnt,
                                  box_index,
                                  car_start,
                                  out_plan);
}

/* 妫€鏌モ€滄寚瀹氱瀛?>鎸囧畾鐩爣鈥濆湪绂佺敤鐐稿脊涓嬫槸鍚﹀彲鐩存帹锛堟彁渚?relaxed car 鍏滃簳锛夈€?*/
static int is_car_start_candidate_free(const planning_state_t *state, Position p)
{
    int i;
    if (!state)
        return 0;
    if (!is_valid_cell(p))
        return 0;

    for (i = 0; i < state->obstacles_cnt; i++)
    {
        if (same_cell(state->obstacles_state[i], p))
            return 0;
    }
    for (i = 0; i < state->bombs_cnt; i++)
    {
        if (same_cell(state->bombs_state[i], p))
            return 0;
    }
    for (i = 0; i < state->boxes_cnt; i++)
    {
        if (same_cell(state->boxes_state[i], p))
            return 0;
    }
    return 1;
}

static int try_reach_with_car_start(const planning_state_t *state,
                                    int box_index,
                                    Position target,
                                    Position car_start)
{
    Position one_target[1];
    path_plan_result plan;
    int steps;

    if (!is_car_start_candidate_free(state, car_start))
        return 0;

    one_target[0] = target;
    steps = plan_box_with_candidates(state, box_index, 0, 0, one_target, 1, car_start, &plan);
    return (steps > 0 && same_cell(plan.box_target, target)) ? 1 : 0;
}

static int box_reachable_no_bomb_relaxed(const planning_state_t *state,
                                         int box_index,
                                         Position target)
{
    Position box_pos;
    const int dr[4] = {-1, 1, 0, 0};
    const int dc[4] = {0, 0, -1, 1};
    Position starts[12];
    int starts_cnt = 0;
    int k;

    if (!state || box_index < 0 || box_index >= state->boxes_cnt)
        return 0;
    box_pos = state->boxes_state[box_index];

    starts[starts_cnt++] = state->car_state;

    /* 箱子四邻：避免当前车位影响可达性判断。 */
    for (k = 0; k < 4; k++)
    {
        Position s = {(uint8)(box_pos.row + dr[k]), (uint8)(box_pos.col + dc[k]), 0};
        starts[starts_cnt++] = s;
    }

    /* 目标四邻：覆盖“能到目标附近但初始车位较远”的场景。 */
    for (k = 0; k < 4; k++)
    {
        Position s = {(uint8)(target.row + dr[k]), (uint8)(target.col + dc[k]), 0};
        starts[starts_cnt++] = s;
    }

    /* 箱子和目标自身（仅在该格可站立时才会生效）。 */
    starts[starts_cnt++] = box_pos;
    starts[starts_cnt++] = target;

    for (k = 0; k < starts_cnt; k++)
    {
        int j;
        int duplicated = 0;
        for (j = 0; j < k; j++)
        {
            if (same_cell(starts[j], starts[k]))
            {
                duplicated = 1;
                break;
            }
        }
        if (duplicated)
            continue;
        if (try_reach_with_car_start(state, box_index, target, starts[k]))
            return 1;
    }

    return 0;
}

static void build_simplified_state_for_pair(const planning_state_t *state,
                                            int box_index,
                                            Position target,
                                            planning_state_t *out_state)
{
    if (!state || !out_state || box_index < 0 || box_index >= state->boxes_cnt)
        return;

    *out_state = *state;
    out_state->bombs_cnt = 0;
    out_state->boxes_cnt = 1;
    out_state->targets_cnt = 1;
    out_state->boxes_state[0] = state->boxes_state[box_index];
    out_state->targets_state[0] = target;
}

/* 璇勪及 pair 鍙揪鎬у綊鍥犮€?*/
static uint8 classify_pair_status(const planning_state_t *state,
                                  int box_index,
                                  Position target)
{
    planning_state_t simplified;
    int full_reachable;
    int simple_reachable;

    if (!state || box_index < 0 || box_index >= state->boxes_cnt)
        return PAIR_STATUS_OBSTACLE;

    full_reachable = box_reachable_no_bomb_relaxed(state, box_index, target);
    if (full_reachable)
        return PAIR_STATUS_NORMAL;

    build_simplified_state_for_pair(state, box_index, target, &simplified);
    simple_reachable = box_reachable_no_bomb_relaxed(&simplified, 0, target);
    if (simple_reachable)
        return PAIR_STATUS_LATE;

    return PAIR_STATUS_OBSTACLE;
}

/* 鍒濆鍖?pair 琛紙mode2锛氭寜 id 閰嶅锛沵ode1 閫€鍖栦负 NORMAL锛夈€?*/
static void build_pair_table(const planning_state_t *state,
                             int require_same_id,
                             pair_task_table_t *table)
{
    int t;

    if (!state || !table)
        return;
    memset(table, 0, sizeof(*table));

    if (!require_same_id)
    {
        return;
    }

    for (t = 0; t < state->targets_cnt && table->count < MAX_TARGETS; t++)
    {
        Position target = state->targets_state[t];
        int box_index = find_box_index_by_id(state->boxes_state, state->boxes_cnt, target.id);
        pair_task_t pair;

        if (box_index < 0)
            continue;

        memset(&pair, 0, sizeof(pair));
        pair.valid = 1;
        pair.target_done = 0;
        pair.box_id_ref = target.id;
        pair.target_ref = target;
        pair.status = classify_pair_status(state, box_index, target);

        table->item[table->count++] = pair;
    }
}

static int state_has_target(const planning_state_t *state, Position target)
{
    return find_position_index(state->targets_state, state->targets_cnt, target) >= 0;
}

/* 姣忚疆鍒锋柊 pair 鐘舵€併€?*/
static void refresh_pair_statuses(const planning_state_t *state,
                                  int require_same_id,
                                  pair_task_table_t *table)
{
    int i;

    if (!state || !table)
        return;

    for (i = 0; i < table->count; i++)
    {
        pair_task_t *pair = &table->item[i];
        int box_index;

        if (!pair->valid || pair->target_done)
            continue;

        if (!state_has_target(state, pair->target_ref))
        {
            pair->target_done = 1;
            continue;
        }

        box_index = find_box_index_by_id(state->boxes_state, state->boxes_cnt, pair->box_id_ref);
        if (box_index < 0)
        {
            pair->target_done = 1;
            continue;
        }

        if (!require_same_id)
        {
            pair->status = PAIR_STATUS_NORMAL;
        }
    }
}

/* 缁熻鏈畬鎴愪笖闈?LATE pair 鏁伴噺銆?*/
static int count_unfinished_non_late_pairs(const pair_task_table_t *table)
{
    int i;
    int cnt = 0;
    if (!table)
        return 0;
    for (i = 0; i < table->count; i++)
    {
        const pair_task_t *pair = &table->item[i];
        if (!pair->valid || pair->target_done)
            continue;
        if (pair->status != PAIR_STATUS_LATE)
            cnt++;
    }
    return cnt;
}

/* 鐢熸垚鈥滈櫎涓荤偢寮瑰鈥濈殑杈呭姪鐐稿脊闆嗗悎銆?*/
static int build_support_bombs_except_primary(const planning_state_t *state,
                                              Position primary_bomb,
                                              Position *out_bombs)
{
    int i;
    int cnt = 0;

    if (!state || !out_bombs)
        return 0;

    for (i = 0; i < state->bombs_cnt; i++)
    {
        Position b = state->bombs_state[i];
        if (!same_cell(b, primary_bomb))
        {
            out_bombs[cnt++] = b;
        }
    }
    return cnt;
}

/*
 * 瑙勫垝鐐稿脊浠诲姟锛? * - 鎶?primary_bomb 褰撲綔铏氭嫙绠卞瓙鎺ㄥ埌 wall_ref锛? * - support_bombs 鍙綔涓鸿緟鍔╃偢寮癸紱
 * - 杩斿洖鏄惁浣跨敤浜嗚緟鍔╃偢寮瑰強鍏剁湡瀹炲潗鏍囥€? */
static int plan_bomb_task_in_state(const planning_state_t *state,
                                   Position primary_bomb,
                                   Position wall_ref,
                                   const Position *support_bombs,
                                   int support_bombs_cnt,
                                   path_plan_result *out_plan,
                                   int *out_used_support_bomb,
                                   Position *out_support_bomb_pos)
{
    Position temp_obstacles[grid_size];
    Position temp_boxes[MAX_BOXES + 1];
    Position one_target[1];
    int temp_obs_cnt = 0;
    int temp_boxes_cnt;
    int i;
    int steps;

    if (!state || !out_plan)
        return -1;
    if (!is_valid_cell(wall_ref))
        return -1;
    if (find_position_index(state->bombs_state, state->bombs_cnt, primary_bomb) < 0)
        return -1;
    if (state->boxes_cnt >= MAX_BOXES + 1)
        return -1;

    for (i = 0; i < state->obstacles_cnt; i++)
    {
        if (!same_cell(state->obstacles_state[i], wall_ref))
        {
            if (temp_obs_cnt < grid_size)
                temp_obstacles[temp_obs_cnt++] = state->obstacles_state[i];
        }
    }

    for (i = 0; i < state->bombs_cnt; i++)
    {
        Position b = state->bombs_state[i];
        if (!same_cell(b, primary_bomb) && !position_in_set(b, support_bombs, support_bombs_cnt))
        {
            if (temp_obs_cnt < grid_size)
                temp_obstacles[temp_obs_cnt++] = b;
        }
    }

    memcpy(temp_boxes, state->boxes_state, (size_t)state->boxes_cnt * sizeof(Position));
    temp_boxes[state->boxes_cnt] = primary_bomb;
    temp_boxes_cnt = state->boxes_cnt + 1;

    one_target[0] = wall_ref;
    steps = integrated_path_output(MAP_ROWS, MAP_COLS,
                                   temp_obstacles, temp_obs_cnt,
                                   support_bombs, support_bombs_cnt,
                                   temp_boxes, temp_boxes_cnt,
                                   one_target, 1,
                                   temp_boxes_cnt - 1,
                                   state->car_state,
                                   out_plan);

    if (out_used_support_bomb)
        *out_used_support_bomb = 0;
    if (out_support_bomb_pos)
        *out_support_bomb_pos = (Position){255, 255, 0};

    if (steps > 0 && out_plan->used_bomb &&
        out_plan->bomb_index >= 0 && out_plan->bomb_index < support_bombs_cnt)
    {
        if (out_used_support_bomb)
            *out_used_support_bomb = 1;
        if (out_support_bomb_pos)
            *out_support_bomb_pos = support_bombs[out_plan->bomb_index];
    }

    if (steps > 0 && out_plan->total_steps > 0)
    {
        out_plan->car_path[out_plan->total_steps - 1].id =
            merge_marker(out_plan->car_path[out_plan->total_steps - 1].id, BOMB_EXPLOSION);
    }
    return steps;
}

/*
 * 涓烘煇 pair + 鏌愮偢寮规帹鏂€滄湁鐢ㄧ偢澧欑偣鈥濓細
 * - 鍦ㄧ畝鍖栧満鏅?浠呰绠卞瓙+璇ョ偢寮?涓婅鍒掞紱
 * - 鑻ョ‘瀹炰娇鐢ㄧ偢寮癸紝鍒欒繑鍥?wall_target銆? */
static int infer_wall_for_pair_with_bomb(const planning_state_t *state,
                                         const pair_task_t *pair,
                                         int require_same_id,
                                         Position bomb,
                                         Position *out_wall)
{
    uint32 state_sig;
    int cached_found = 0;
    Position cached_wall = {255, 255, 0};
    planning_state_t simplified;
    int box_index;
    Position one_bomb[1];
    Position one_target[1];
    path_plan_result plan;
    int steps;

    if (!state || !pair || !out_wall)
        return 0;
    if (!pair->valid || pair->target_done)
        return 0;
    if (!state_has_target(state, pair->target_ref))
        return 0;

    state_sig = hash_state_signature(state);
    if (lookup_infer_wall_cache(state_sig,
                                pair->box_id_ref,
                                pair->target_ref,
                                require_same_id,
                                bomb,
                                &cached_found,
                                &cached_wall))
    {
        if (cached_found)
            *out_wall = cached_wall;
        return cached_found;
    }

    box_index = find_box_index_by_id(state->boxes_state, state->boxes_cnt, pair->box_id_ref);
    if (box_index < 0)
        return 0;

    if (require_same_id && !box_can_match_target(state->boxes_state[box_index], pair->target_ref, require_same_id))
        return 0;

    build_simplified_state_for_pair(state, box_index, pair->target_ref, &simplified);
    one_bomb[0] = bomb;
    one_target[0] = pair->target_ref;
    simplified.bombs_cnt = 1;
    simplified.bombs_state[0] = bomb;

    steps = plan_box_with_candidates(&simplified, 0, one_bomb, 1, one_target, 1, simplified.car_state, &plan);
    if (!(steps > 0 && plan.used_bomb && is_valid_cell(plan.wall_target)))
    {
        steps = plan_box_with_candidates(&simplified, 0, one_bomb, 1, one_target, 1, simplified.boxes_state[0], &plan);
    }

    if (steps > 0 && plan.used_bomb && is_valid_cell(plan.wall_target))
    {
        *out_wall = plan.wall_target;
        store_infer_wall_cache(state_sig,
                               pair->box_id_ref,
                               pair->target_ref,
                               require_same_id,
                               bomb,
                               1,
                               plan.wall_target);
        return 1;
    }
    store_infer_wall_cache(state_sig,
                           pair->box_id_ref,
                           pair->target_ref,
                           require_same_id,
                           bomb,
                           0,
                           (Position){255, 255, 0});
    return 0;
}

static void append_unique_wall_candidate(Position *walls, int *cnt, int max_cnt, Position wall)
{
    int i;
    if (!walls || !cnt || max_cnt <= 0)
        return;
    if (!is_valid_cell(wall))
        return;
    for (i = 0; i < *cnt; i++)
    {
        if (same_cell(walls[i], wall))
            return;
    }
    if (*cnt < max_cnt)
    {
        walls[*cnt] = wall;
        (*cnt)++;
    }
}

static int state_has_obstacle_at(const planning_state_t *state, Position p)
{
    if (!state || !is_valid_cell(p))
        return 0;
    return (find_position_index(state->obstacles_state, state->obstacles_cnt, p) >= 0) ? 1 : 0;
}

static int state_find_bomb_at(const planning_state_t *state, Position p, Position *out_bomb)
{
    int idx;
    if (!state || !is_valid_cell(p))
        return 0;
    idx = find_position_index(state->bombs_state, state->bombs_cnt, p);
    if (idx < 0)
        return 0;
    if (out_bomb)
        *out_bomb = state->bombs_state[idx];
    return 1;
}

/* 针对一颗“阻挡推位”的炸弹，补充其可行爆破墙及其前置站位墙。 */
static void append_bomb_local_unlock_walls(const planning_state_t *state,
                                           Position bomb_pos,
                                           Position *walls,
                                           int *cnt,
                                           int max_cnt)
{
    const int dr[4] = {-1, 1, 0, 0};
    const int dc[4] = {0, 0, -1, 1};
    int k;

    if (!state || !walls || !cnt || max_cnt <= 0)
        return;

    for (k = 0; k < 4; k++)
    {
        Position wall = {(uint8)((int)bomb_pos.row + dr[k]), (uint8)((int)bomb_pos.col + dc[k]), 0};
        Position stand = {(uint8)((int)bomb_pos.row - dr[k]), (uint8)((int)bomb_pos.col - dc[k]), 0};

        if (state_has_obstacle_at(state, wall))
            append_unique_wall_candidate(walls, cnt, max_cnt, wall);
        if (state_has_obstacle_at(state, stand))
            append_unique_wall_candidate(walls, cnt, max_cnt, stand);
    }
}

/* 为某个 pair + 主炸弹收集“可尝试炸开的墙点”：
 * 1) 直接有助于该 pair 的墙点；
 * 2) 其他“中继炸弹”若要执行其关键终推，所需的车位被墙占据时，该墙点也加入。
 *    这使得算法能找到 6,9 -> 6,11 -> 7,11 -> 8,11 这种连锁开路。 */
static int collect_unlock_wall_candidates_for_pair(const planning_state_t *state,
                                                   const pair_task_t *pair,
                                                   int require_same_id,
                                                   Position primary_bomb,
                                                   Position *out_walls,
                                                   int max_walls)
{
    int i;
    int cnt = 0;
    Position wall;
    int box_index;
    Position box_pos;
    const int dr4[4] = {-1, 1, 0, 0};
    const int dc4[4] = {0, 0, -1, 1};

    if (!state || !pair || !out_walls || max_walls <= 0)
        return 0;

    /* 补充主炸弹周边“本地可炸墙点”，避免因推断失败漏掉近处有效开路动作。 */
    append_bomb_local_unlock_walls(state, primary_bomb, out_walls, &cnt, max_walls);

    if (infer_wall_for_pair_with_bomb(state, pair, require_same_id, primary_bomb, &wall))
    {
        append_unique_wall_candidate(out_walls, &cnt, max_walls, wall);
    }

    for (i = 0; i < state->bombs_cnt; i++)
    {
        Position relay_bomb = state->bombs_state[i];
        Position relay_wall;
        int dr, dc;
        int adr, adc;
        Position need_stand;

        if (!infer_wall_for_pair_with_bomb(state, pair, require_same_id, relay_bomb, &relay_wall))
            continue;

        dr = (int)relay_wall.row - (int)relay_bomb.row;
        dc = (int)relay_wall.col - (int)relay_bomb.col;
        adr = (dr >= 0) ? dr : -dr;
        adc = (dc >= 0) ? dc : -dc;
        if (adr + adc != 1)
            continue;

        need_stand.row = (uint8)((int)relay_bomb.row - dr);
        need_stand.col = (uint8)((int)relay_bomb.col - dc);
        need_stand.id = 0;
        if (!is_valid_cell(need_stand))
            continue;
        if (find_position_index(state->obstacles_state, state->obstacles_cnt, need_stand) < 0)
            continue;

        append_unique_wall_candidate(out_walls, &cnt, max_walls, need_stand);
    }

    /* 局部推位依赖：围绕目标箱子的四个方向，补充“前向阻挡墙/后向推位墙”。
     * 这一步不依赖“该炸弹单独就能直达目标”，用于捕获连锁开路第一步。 */
    box_index = find_box_index_by_id(state->boxes_state, state->boxes_cnt, pair->box_id_ref);
    if (box_index >= 0)
    {
        int k;
        box_pos = state->boxes_state[box_index];
        for (k = 0; k < 4; k++)
        {
            Position front = {(uint8)((int)box_pos.row + dr4[k]), (uint8)((int)box_pos.col + dc4[k]), 0};
            Position back = {(uint8)((int)box_pos.row - dr4[k]), (uint8)((int)box_pos.col - dc4[k]), 0};
            Position block_bomb;

            if (state_has_obstacle_at(state, front))
                append_unique_wall_candidate(out_walls, &cnt, max_walls, front);
            if (state_has_obstacle_at(state, back))
                append_unique_wall_candidate(out_walls, &cnt, max_walls, back);

            if (state_find_bomb_at(state, front, &block_bomb))
            {
                append_bomb_local_unlock_walls(state, block_bomb, out_walls, &cnt, max_walls);
            }
            if (state_find_bomb_at(state, back, &block_bomb))
            {
                append_bomb_local_unlock_walls(state, block_bomb, out_walls, &cnt, max_walls);
            }
        }
    }

    return cnt;
}

/* 判断该 pair 在当前状态下是否已经有可执行的“推箱到配对目标”任务（允许用炸弹）。 */
static int pair_has_box_action_now(const planning_state_t *state,
                                   const pair_task_t *pair,
                                   int require_same_id)
{
    uint32 state_sig;
    int cached_reachable = 0;
    int box_index;
    Position one_target[1];
    Position all_bombs[MAX_BOMBS];
    int all_bombs_cnt;
    path_plan_result plan;
    int steps;

    if (!state || !pair || !pair->valid || pair->target_done)
        return 0;
    if (!state_has_target(state, pair->target_ref))
        return 0;

    state_sig = hash_state_signature(state);
    if (lookup_pair_reach_cache(state_sig,
                                pair->box_id_ref,
                                pair->target_ref,
                                require_same_id,
                                &cached_reachable))
    {
        return cached_reachable;
    }

    box_index = find_box_index_by_id(state->boxes_state, state->boxes_cnt, pair->box_id_ref);
    if (box_index < 0)
        return 0;
    if (require_same_id && !box_can_match_target(state->boxes_state[box_index], pair->target_ref, require_same_id))
        return 0;

    one_target[0] = pair->target_ref;
    all_bombs_cnt = state->bombs_cnt;
    memcpy(all_bombs, state->bombs_state, (size_t)all_bombs_cnt * sizeof(Position));

    steps = plan_box_with_candidates(state,
                                     box_index,
                                     all_bombs,
                                     all_bombs_cnt,
                                     one_target,
                                     1,
                                     state->car_state,
                                     &plan);
    cached_reachable = (steps > 0 && same_cell(plan.box_target, pair->target_ref)) ? 1 : 0;
    store_pair_reach_cache(state_sig,
                           pair->box_id_ref,
                           pair->target_ref,
                           require_same_id,
                           cached_reachable);
    return cached_reachable;
}

/* 判断该 pair 是否“当前不可执行”，即需要先开路。 */
static int pair_needs_unlock_in_state(const planning_state_t *state,
                                      const pair_task_t *pair,
                                      int require_same_id)
{
    return pair_has_box_action_now(state, pair, require_same_id) ? 0 : 1;
}

/* 关键炸弹归属分析：
 * 对每颗炸弹判断是否“仅有一个 pair 能用它解锁”，
 * 若是则暂时标记为该 pair 的关键资源，避免被其他任务抢用。 */
static void build_critical_owner_for_state(const planning_state_t *state,
                                           int require_same_id,
                                           const pair_task_table_t *pairs,
                                           int critical_owner[MAX_BOMBS])
{
    int p, b;
    int unlock_pair_cnt = 0;
    uint32 state_sig;
    uint32 pairs_sig;

    for (b = 0; b < MAX_BOMBS; b++)
        critical_owner[b] = -1;
    if (!state || !pairs)
        return;

    state_sig = hash_state_signature(state);
    pairs_sig = hash_pairs_signature(pairs);
    if (s_critical_owner_cache.valid &&
        s_critical_owner_cache.state_sig == state_sig &&
        s_critical_owner_cache.pairs_sig == pairs_sig &&
        s_critical_owner_cache.require_same_id == (uint8)(require_same_id ? 1 : 0))
    {
        memcpy(critical_owner,
               s_critical_owner_cache.critical_owner,
               sizeof(s_critical_owner_cache.critical_owner));
        return;
    }

    /* 快速剪枝：
     * - 炸弹数<=1：不存在“抢同一颗关键炸弹”的冲突；
     * - 需解锁 pair<=1：也不需要做归属判定。 */
    if (state->bombs_cnt <= 1 || pairs->count <= 1)
        goto CACHE_AND_RETURN;

    for (p = 0; p < pairs->count; p++)
    {
        const pair_task_t *pair = &pairs->item[p];
        if (!pair->valid || pair->target_done)
            continue;
        if (!pair_needs_unlock_in_state(state, pair, require_same_id))
            continue;
        unlock_pair_cnt++;
        if (unlock_pair_cnt > 1)
            break;
    }
    if (unlock_pair_cnt <= 1)
        goto CACHE_AND_RETURN;

    for (p = 0; p < pairs->count; p++)
    {
        const pair_task_t *pair = &pairs->item[p];
        int feasible_cnt = 0;
        int unique_bomb_index = -1;

        if (!pair_needs_unlock_in_state(state, pair, require_same_id))
            continue;

        for (b = 0; b < state->bombs_cnt; b++)
        {
            Position primary_bomb = state->bombs_state[b];
            Position wall;

            if (!infer_wall_for_pair_with_bomb(state, pair, require_same_id, primary_bomb, &wall))
                continue;
            /* 性能优化：关键炸弹归属仅做“可解锁可行性”判定，不做完整路径精算。 */
            feasible_cnt++;
            unique_bomb_index = b;
            if (feasible_cnt > 1)
                break;
        }

        if (feasible_cnt == 1 && unique_bomb_index >= 0)
        {
            if (critical_owner[unique_bomb_index] == -1)
            {
                critical_owner[unique_bomb_index] = p;
            }
            else if (critical_owner[unique_bomb_index] != p)
            {
                critical_owner[unique_bomb_index] = -2;
            }
        }
    }

CACHE_AND_RETURN:
    s_critical_owner_cache.valid = 1;
    s_critical_owner_cache.state_sig = state_sig;
    s_critical_owner_cache.pairs_sig = pairs_sig;
    s_critical_owner_cache.require_same_id = (uint8)(require_same_id ? 1 : 0);
    memcpy(s_critical_owner_cache.critical_owner,
           critical_owner,
           sizeof(s_critical_owner_cache.critical_owner));
}

static int action_uses_other_pair_critical_bomb(const planning_state_t *state,
                                                const round_action_t *action,
                                                const int critical_owner[MAX_BOMBS],
                                                int owner_pair_index)
{
    int idx;
    if (!state || !action || !critical_owner)
        return 0;

    if (action->has_primary_bomb_pos)
    {
        idx = find_position_index(state->bombs_state, state->bombs_cnt, action->primary_bomb_pos);
        if (idx >= 0 && critical_owner[idx] >= 0 && critical_owner[idx] != owner_pair_index)
            return 1;
    }

    if (action->has_support_bomb_pos)
    {
        idx = find_position_index(state->bombs_state, state->bombs_cnt, action->support_bomb_pos);
        if (idx >= 0 && critical_owner[idx] >= 0 && critical_owner[idx] != owner_pair_index)
            return 1;
    }
    return 0;
}

static int build_ranked_bomb_indices_for_pair(const planning_state_t *state,
                                              const pair_task_t *pair,
                                              int box_index,
                                              int *out_indices)
{
    int i;
    int cnt;
    int scores[MAX_BOMBS];
    if (!state || !pair || !out_indices)
        return 0;
    if (box_index < 0 || box_index >= state->boxes_cnt)
        return 0;

    cnt = state->bombs_cnt;
    if (cnt > MAX_BOMBS)
        cnt = MAX_BOMBS;
    for (i = 0; i < cnt; i++)
    {
        Position bomb = state->bombs_state[i];
        Position box = state->boxes_state[box_index];
        scores[i] = manhattan_cell_dist(bomb, box) +
                    manhattan_cell_dist(bomb, pair->target_ref) +
                    (manhattan_cell_dist(state->car_state, bomb) >> 1);
        out_indices[i] = i;
    }

    for (i = 0; i < cnt; i++)
    {
        int j;
        int best = i;
        for (j = i + 1; j < cnt; j++)
        {
            if (scores[j] < scores[best])
                best = j;
        }
        if (best != i)
        {
            int tmp_idx = out_indices[i];
            int tmp_score = scores[i];
            out_indices[i] = out_indices[best];
            scores[i] = scores[best];
            out_indices[best] = tmp_idx;
            scores[best] = tmp_score;
        }
    }
    return cnt;
}

static void trim_wall_candidates_fast(const planning_state_t *state,
                                      const pair_task_t *pair,
                                      int box_index,
                                      Position *walls,
                                      int wall_cnt,
                                      int keep_cnt)
{
    int i;
    int scores[MAX_UNLOCK_WALL_CANDIDATES];
    if (!state || !pair || !walls || wall_cnt <= 0)
        return;
    if (box_index < 0 || box_index >= state->boxes_cnt)
        return;
    if (keep_cnt <= 0 || keep_cnt >= wall_cnt)
        return;

    for (i = 0; i < wall_cnt; i++)
    {
        scores[i] = manhattan_cell_dist(walls[i], pair->target_ref) +
                    manhattan_cell_dist(walls[i], state->boxes_state[box_index]);
    }

    for (i = 0; i < keep_cnt; i++)
    {
        int j;
        int best = i;
        for (j = i + 1; j < wall_cnt; j++)
        {
            if (scores[j] < scores[best])
                best = j;
        }
        if (best != i)
        {
            Position tp = walls[i];
            int ts = scores[i];
            walls[i] = walls[best];
            scores[i] = scores[best];
            walls[best] = tp;
            scores[best] = ts;
        }
    }
}

static int collect_bomb_actions_for_pair(const planning_state_t *state,
                                         int require_same_id,
                                         const pair_task_table_t *pairs,
                                         int pair_index,
                                         const int critical_owner[MAX_BOMBS],
                                         round_action_t *out_actions,
                                         int max_actions)
{
    int ri;
    int out_cnt = 0;
    int has_best = 0;
    int best_steps = INT_MAX;
    round_action_t best_action;
    const pair_task_t *pair;
    int box_index;
    int ranked_bombs[MAX_BOMBS];
    int ranked_cnt;
    int pass;

    if (!state || !pairs || !out_actions || max_actions <= 0)
        return 0;
    if (pair_index < 0 || pair_index >= pairs->count)
        return 0;

    pair = &pairs->item[pair_index];
    if (!pair_needs_unlock_in_state(state, pair, require_same_id))
        return 0;
    box_index = find_box_index_by_id(state->boxes_state, state->boxes_cnt, pair->box_id_ref);
    if (box_index < 0)
        return 0;

    ranked_cnt = build_ranked_bomb_indices_for_pair(state, pair, box_index, ranked_bombs);
    if (ranked_cnt <= 0)
        return 0;

    /* 仅保留“该 pair 当前最优炸弹动作”：
     * - 第一轮只看 TopK 炸弹 + TopK 墙点，快速拿到高质量候选；
     * - 若第一轮无解，再扩大搜索范围兜底；
     * - 每个候选先用曼哈顿下界剪枝，再调用底层规划。 */
    for (pass = 0; pass < 2 && !has_best; pass++)
    {
        int bomb_limit = (pass == 0) ? FAST_UNLOCK_BOMB_TOPK_PASS1 : ranked_cnt;
        if (bomb_limit > ranked_cnt)
            bomb_limit = ranked_cnt;

        for (ri = 0; ri < bomb_limit; ri++)
        {
            int b = ranked_bombs[ri];
            Position primary_bomb = state->bombs_state[b];
            Position walls[MAX_UNLOCK_WALL_CANDIDATES];
            int wall_cnt;
            int wall_limit;
            int w;
            Position support_all[MAX_BOMBS];
            int support_all_cnt;

            if (critical_owner && critical_owner[b] >= 0 && critical_owner[b] != pair_index)
                continue;

            wall_cnt = collect_unlock_wall_candidates_for_pair(state,
                                                               pair,
                                                               require_same_id,
                                                               primary_bomb,
                                                               walls,
                                                               MAX_UNLOCK_WALL_CANDIDATES);
            if (wall_cnt <= 0)
                continue;

            support_all_cnt = build_support_bombs_except_primary(state, primary_bomb, support_all);
            wall_limit = (pass == 0) ? FAST_UNLOCK_WALL_TOPK_PASS1 : wall_cnt;
            if (wall_limit > wall_cnt)
                wall_limit = wall_cnt;
            trim_wall_candidates_fast(state, pair, box_index, walls, wall_cnt, wall_limit);

            for (w = 0; w < wall_limit; w++)
            {
                path_plan_result plan;
                int steps;
                int used_support = 0;
                Position support_pos = {255, 255, 0};
                round_action_t candidate;
                Position wall = walls[w];
                int lb;

                /* 曼哈顿下界剪枝：已有更优候选时，提前跳过昂贵规划。 */
                lb = manhattan_cell_dist(state->car_state, primary_bomb) - 1;
                if (lb < 0)
                    lb = 0;
                lb += manhattan_cell_dist(primary_bomb, wall);
                if (has_best && lb >= best_steps)
                    continue;

                steps = plan_bomb_task_in_state(state,
                                                primary_bomb,
                                                wall,
                                                0, 0,
                                                &plan,
                                                &used_support,
                                                &support_pos);
                if (steps <= 0)
                {
                    steps = plan_bomb_task_in_state(state,
                                                    primary_bomb,
                                                    wall,
                                                    support_all,
                                                    support_all_cnt,
                                                    &plan,
                                                    &used_support,
                                                    &support_pos);
                }
                if (steps <= 0)
                    continue;

                memset(&candidate, 0, sizeof(candidate));
                candidate.valid = 1;
                candidate.action_type = ACTION_BOMB;
                candidate.steps = steps;
                candidate.box_index = -1;
                candidate.pair_index = pair_index;
                candidate.plan = plan;
                candidate.has_support_bomb_pos = (uint8)(used_support ? 1 : 0);
                candidate.support_bomb_pos = support_pos;
                candidate.has_primary_bomb_pos = 1;
                candidate.primary_bomb_pos = primary_bomb;

                if (critical_owner &&
                    action_uses_other_pair_critical_bomb(state, &candidate, critical_owner, pair_index))
                {
                    continue;
                }

                if (!has_best ||
                    candidate.steps < best_steps ||
                    (candidate.steps == best_steps &&
                     candidate.has_support_bomb_pos < best_action.has_support_bomb_pos))
                {
                    has_best = 1;
                    best_steps = candidate.steps;
                    best_action = candidate;
                }
            }
        }
    }

    if (has_best && max_actions > 0)
    {
        out_actions[0] = best_action;
        out_cnt = 1;
    }

    /* 兜底：若严格关键炸弹约束下无可行动作，则放宽一次约束避免死锁。 */
    if (out_cnt == 0 && critical_owner != 0)
    {
        return collect_bomb_actions_for_pair(state,
                                             require_same_id,
                                             pairs,
                                             pair_index,
                                             0,
                                             out_actions,
                                             max_actions);
    }

    return out_cnt;
}

/* 选择本轮最优“炸弹开路动作”：
 * - 先按 pair 评估局部最优炸弹动作；
 * - 再跨 pair 做同台比较；
 * - 优先选“能立刻让该 pair 变为可推箱”的动作。 */
static int pick_best_unlock_bomb_action_for_obstacles(const planning_state_t *state,
                                                      int require_same_id,
                                                      const pair_task_table_t *pairs,
                                                      round_action_t *out_action)
{
    int p;
    int has_best = 0;
    int best_improve = -1;
    int best_steps = 0;
    uint8 best_pair_id = 255;
    int critical_owner[MAX_BOMBS];
    const int *critical_owner_ptr = 0;
    int unlock_pair_cnt = 0;
    round_action_t best_action;

    if (!state || !pairs || !out_action)
        return 0;

    memset(&best_action, 0, sizeof(best_action));

    /* 只有“多炸弹 + 多待解锁 pair”时，才计算关键炸弹归属。 */
    if (state->bombs_cnt > 1)
    {
        for (p = 0; p < pairs->count; p++)
        {
            const pair_task_t *pair = &pairs->item[p];
            if (!pair->valid || pair->target_done)
                continue;
            if (!pair_needs_unlock_in_state(state, pair, require_same_id))
                continue;
            unlock_pair_cnt++;
            if (unlock_pair_cnt > 1)
                break;
        }
        if (unlock_pair_cnt > 1)
        {
            build_critical_owner_for_state(state, require_same_id, pairs, critical_owner);
            critical_owner_ptr = critical_owner;
        }
    }

    for (p = 0; p < pairs->count; p++)
    {
        const pair_task_t *pair = &pairs->item[p];
        round_action_t actions[MAX_BOMB_ACTION_CANDIDATES];
        round_action_t local_best;
        int action_cnt;
        int improve = 0;
        uint8 pair_id = pair->box_id_ref;

        if (!pair_needs_unlock_in_state(state, pair, require_same_id))
            continue;

        action_cnt = collect_bomb_actions_for_pair(state,
                                                   require_same_id,
                                                   pairs,
                                                   p,
                                                   critical_owner_ptr,
                                                   actions,
                                                   MAX_BOMB_ACTION_CANDIDATES);
        if (action_cnt <= 0)
            continue;

        local_best = actions[0];

        {
            planning_state_t shadow_state = *state;
            apply_bomb_action_result(&shadow_state, &local_best, 0, 0);
            if (pair_has_box_action_now(&shadow_state, pair, require_same_id))
                improve = 1;
        }

        if (!has_best ||
            improve > best_improve ||
            (improve == best_improve && local_best.steps < best_steps) ||
            (improve == best_improve && local_best.steps == best_steps && pair_id < best_pair_id))
        {
            has_best = 1;
            best_improve = improve;
            best_steps = local_best.steps;
            best_pair_id = pair_id;
            best_action = local_best;
        }
    }

    if (!has_best)
        return 0;

    *out_action = best_action;
    return 1;
}

/* 收集当前可执行的推箱任务候选（可选：跳过 LATE pair）。 */
static int collect_box_action_candidates(const planning_state_t *state,
                                         int require_same_id,
                                         const pair_task_table_t *pairs,
                                         int skip_late_pairs,
                                         const int critical_owner[MAX_BOMBS],
                                         round_action_t *out_actions,
                                         int max_actions)
{
    int p;
    int out_cnt = 0;
    int best_steps_seen = INT_MAX;
    Position all_bombs[MAX_BOMBS];
    int all_bombs_cnt;

    if (!state || !pairs || !out_actions || max_actions <= 0)
        return 0;

    all_bombs_cnt = state->bombs_cnt;
    memcpy(all_bombs, state->bombs_state, (size_t)all_bombs_cnt * sizeof(Position));

    for (p = 0; p < pairs->count && out_cnt < max_actions; p++)
    {
        const pair_task_t *pair = &pairs->item[p];
        int box_index;
        Position one_target[1];
        path_plan_result plan;
        int steps;
        round_action_t candidate;

        if (!pair->valid || pair->target_done)
            continue;
        if (skip_late_pairs && pair->status == PAIR_STATUS_LATE)
            continue;
        if (!state_has_target(state, pair->target_ref))
            continue;

        box_index = find_box_index_by_id(state->boxes_state, state->boxes_cnt, pair->box_id_ref);
        if (box_index < 0)
            continue;
        if (!box_can_match_target(state->boxes_state[box_index], pair->target_ref, require_same_id))
            continue;

        /* 下界剪枝：若曼哈顿下界已不优于当前最好步数，跳过昂贵规划。 */
        {
            int lb = manhattan_cell_dist(state->car_state, state->boxes_state[box_index]) +
                     manhattan_cell_dist(state->boxes_state[box_index], pair->target_ref);
            if (best_steps_seen < INT_MAX && lb >= best_steps_seen)
                continue;
        }

        one_target[0] = pair->target_ref;
        steps = plan_box_with_candidates(state,
                                         box_index,
                                         all_bombs, all_bombs_cnt,
                                         one_target, 1,
                                         state->car_state,
                                         &plan);
        if (steps <= 0)
            continue;
        if (steps < best_steps_seen)
            best_steps_seen = steps;

        memset(&candidate, 0, sizeof(candidate));
        candidate.valid = 1;
        candidate.action_type = ACTION_BOX;
        candidate.steps = steps;
        candidate.box_index = box_index;
        candidate.pair_index = p;
        candidate.plan = plan;
        candidate.has_support_bomb_pos = 0;
        candidate.has_primary_bomb_pos = 0;

        if (plan.used_bomb && plan.bomb_index >= 0 && plan.bomb_index < all_bombs_cnt)
        {
            candidate.has_support_bomb_pos = 1;
            candidate.support_bomb_pos = all_bombs[plan.bomb_index];
        }

        if (critical_owner &&
            action_uses_other_pair_critical_bomb(state, &candidate, critical_owner, p))
        {
            continue;
        }

        out_actions[out_cnt++] = candidate;
    }

    return out_cnt;
}

/* 选择最短推箱任务（贪心基线）。 */
static int pick_best_box_action_greedy(const planning_state_t *state,
                                       int require_same_id,
                                       const pair_task_table_t *pairs,
                                       int skip_late_pairs,
                                       round_action_t *out_action)
{
    round_action_t actions[MAX_TARGETS];
    int action_cnt;
    int i;
    int best_idx;
    int critical_owner[MAX_BOMBS];

    if (!state || !pairs || !out_action)
        return 0;

    action_cnt = collect_box_action_candidates(state,
                                               require_same_id,
                                               pairs,
                                               skip_late_pairs,
                                               0,
                                               actions,
                                               MAX_TARGETS);
    if (action_cnt <= 0)
        return 0;
    if (action_cnt == 1)
    {
        *out_action = actions[0];
        return 1;
    }

    if (state->bombs_cnt > 0)
    {
        int kept = 0;
        build_critical_owner_for_state(state, require_same_id, pairs, critical_owner);
        for (i = 0; i < action_cnt; i++)
        {
            if (!action_uses_other_pair_critical_bomb(state,
                                                      &actions[i],
                                                      critical_owner,
                                                      actions[i].pair_index))
            {
                actions[kept++] = actions[i];
            }
        }
        if (kept > 0)
            action_cnt = kept;
    }

    best_idx = 0;
    for (i = 1; i < action_cnt; i++)
    {
        if (actions[i].steps < actions[best_idx].steps ||
            (actions[i].steps == actions[best_idx].steps &&
             pairs->item[actions[i].pair_index].box_id_ref < pairs->item[actions[best_idx].pair_index].box_id_ref))
        {
            best_idx = i;
        }
    }

    *out_action = actions[best_idx];
    return 1;
}

/* 每轮统一调度（核心入口）：
 * 1) 先选推箱最优动作；
 * 2) 若推箱动作已经很短，直接执行，减少无意义炸弹评估；
 * 3) 否则再评估炸弹开路动作，与推箱动作同台比较；
 * 4) 最终只看“本轮动作成本”做轻量决策，避免深层前瞻造成指数级耗时。 */
static int pick_best_round_action_balanced(const planning_state_t *state,
                                           int require_same_id,
                                           const pair_task_table_t *pairs,
                                           round_action_t *out_action)
{
    round_action_t unlock_action;
    round_action_t box_action;
    int has_unlock = 0;
    int has_box = 0;
    int non_late_unfinished;

    if (!state || !pairs || !out_action)
        return 0;

    non_late_unfinished = count_unfinished_non_late_pairs(pairs);
    has_box = pick_best_box_action_greedy(state,
                                          require_same_id,
                                          pairs,
                                          (non_late_unfinished > 0) ? 1 : 0,
                                          &box_action);
    if (!has_box && non_late_unfinished > 0)
    {
        has_box = pick_best_box_action_greedy(state, require_same_id, pairs, 0, &box_action);
    }

    if (has_box && box_action.steps <= FAST_BOX_DIRECT_PICK_STEPS)
    {
        *out_action = box_action;
        return 1;
    }

    if (state->bombs_cnt > 0)
    {
        has_unlock = pick_best_unlock_bomb_action_for_obstacles(state, require_same_id, pairs, &unlock_action);
    }
    else
    {
        has_unlock = 0;
    }

    if (!has_unlock && !has_box)
        return 0;
    if (has_box && !has_unlock)
    {
        *out_action = box_action;
        return 1;
    }
    if (has_unlock && !has_box)
    {
        *out_action = unlock_action;
        return 1;
    }

    /* 轻量同台比较：按当前动作代价直接选，避免每轮二次仿真开销。 */
    if (box_action.steps <= unlock_action.steps)
    {
        *out_action = box_action;
    }
    else
    {
        *out_action = unlock_action;
    }
    return 1;
}

static void load_state_from_globals(planning_state_t *state)
{
    if (!state)
        return;

    memset(state, 0, sizeof(*state));

    state->obstacles_cnt = (int)Obstacles_count;
    state->bombs_cnt = (int)Bombs_count;
    state->boxes_cnt = (int)Boxes_count;
    state->targets_cnt = (int)Targets_count;
    state->car_state = car;

    if (state->obstacles_cnt > MAX_OBSTACLES)
        state->obstacles_cnt = MAX_OBSTACLES;
    if (state->bombs_cnt > MAX_BOMBS)
        state->bombs_cnt = MAX_BOMBS;
    if (state->boxes_cnt > MAX_BOXES)
        state->boxes_cnt = MAX_BOXES;
    if (state->targets_cnt > MAX_TARGETS)
        state->targets_cnt = MAX_TARGETS;

    memcpy(state->obstacles_state, obstacles, (size_t)state->obstacles_cnt * sizeof(Position));
    memcpy(state->bombs_state, bombs, (size_t)state->bombs_cnt * sizeof(Position));
    memcpy(state->boxes_state, boxes, (size_t)state->boxes_cnt * sizeof(Position));
    memcpy(state->targets_state, targets, (size_t)state->targets_cnt * sizeof(Position));
}

static void save_state_to_globals(const planning_state_t *state)
{
    if (!state)
        return;

    Obstacles_count = (size_t)state->obstacles_cnt;
    Bombs_count = (size_t)state->bombs_cnt;
    Boxes_count = (size_t)state->boxes_cnt;
    Targets_count = (size_t)state->targets_cnt;
    car = state->car_state;

    memcpy(obstacles, state->obstacles_state, (size_t)state->obstacles_cnt * sizeof(Position));
    memcpy(bombs, state->bombs_state, (size_t)state->bombs_cnt * sizeof(Position));
    memcpy(boxes, state->boxes_state, (size_t)state->boxes_cnt * sizeof(Position));
    memcpy(targets, state->targets_state, (size_t)state->targets_cnt * sizeof(Position));
}

static void save_path_to_globals(const Position *merged_path, int merged_len)
{
    if (!merged_path || merged_len <= 0)
    {
        Car_path_count = 0;
        return;
    }

    if (merged_len > MAX_CAR_PATH)
        merged_len = MAX_CAR_PATH;

    memcpy(car_path, merged_path, (size_t)merged_len * sizeof(Position));
    Car_path_count = (size_t)merged_len;
}

/* 执行“推箱动作”并回写状态：
 * - 合并本段路径；
 * - 若该推箱动作中使用了辅助炸弹，先移除炸弹并炸开对应墙区；
 * - 移除已完成的箱子与目标点；
 * - 更新小车终点。 */
static void apply_box_action_result(planning_state_t *state,
                                    pair_task_table_t *pairs,
                                    const round_action_t *action,
                                    Position *merged_path,
                                    int *merged_len)
{
    Position explode_pos;
    int box_remove_index;
    int target_remove_index;

    if (!state || !action)
        return;
    if (!action->valid || action->action_type != ACTION_BOX)
        return;
    if (action->box_index < 0 || action->box_index >= state->boxes_cnt)
        return;

    if (merged_path && merged_len)
    {
        append_segment_path(merged_path, merged_len, action->plan.car_path, action->plan.total_steps);
    }

    if (action->has_support_bomb_pos)
    {
        remove_bomb_by_position(state->bombs_state, &state->bombs_cnt, action->support_bomb_pos);
        explode_pos = action->plan.bomb_target;
        if (!is_valid_cell(explode_pos))
            explode_pos = action->plan.wall_target;
        if (is_valid_cell(explode_pos))
        {
            simulate_bomb_explosion(state->obstacles_state, &state->obstacles_cnt, explode_pos);
        }
    }

    box_remove_index = find_position_index(state->boxes_state, state->boxes_cnt, state->boxes_state[action->box_index]);
    if (box_remove_index >= 0)
    {
        remove_position_at(state->boxes_state, &state->boxes_cnt, box_remove_index);
    }
    else if (action->box_index >= 0 && action->box_index < state->boxes_cnt)
    {
        remove_position_at(state->boxes_state, &state->boxes_cnt, action->box_index);
    }

    target_remove_index = find_position_index(state->targets_state, state->targets_cnt, action->plan.box_target);
    if (target_remove_index >= 0)
    {
        remove_position_at(state->targets_state, &state->targets_cnt, target_remove_index);
    }

    if (action->plan.total_steps > 0)
    {
        state->car_state = action->plan.car_path[action->plan.total_steps - 1];
    }

    if (pairs && action->pair_index >= 0 && action->pair_index < pairs->count)
    {
        pairs->item[action->pair_index].target_done = 1;
    }
}

/* 执行“炸弹开路动作”并回写状态：
 * - 合并本段路径；
 * - 移除辅助炸弹和主炸弹；
 * - 对爆破点进行地形更新（清除障碍）；
 * - 更新小车终点。 */
static void apply_bomb_action_result(planning_state_t *state,
                                     const round_action_t *action,
                                     Position *merged_path,
                                     int *merged_len)
{
    Position explode_pos;

    if (!state || !action)
        return;
    if (!action->valid || action->action_type != ACTION_BOMB)
        return;

    if (merged_path && merged_len)
    {
        append_segment_path(merged_path, merged_len, action->plan.car_path, action->plan.total_steps);
    }

    if (action->has_support_bomb_pos)
    {
        remove_bomb_by_position(state->bombs_state, &state->bombs_cnt, action->support_bomb_pos);
        explode_pos = action->plan.bomb_target;
        if (!is_valid_cell(explode_pos))
            explode_pos = action->plan.wall_target;
        if (is_valid_cell(explode_pos))
        {
            simulate_bomb_explosion(state->obstacles_state, &state->obstacles_cnt, explode_pos);
        }
    }

    if (action->has_primary_bomb_pos)
    {
        remove_bomb_by_position(state->bombs_state, &state->bombs_cnt, action->primary_bomb_pos);
    }

    explode_pos = action->plan.box_target;
    if (!is_valid_cell(explode_pos))
        explode_pos = action->plan.wall_target;
    if (is_valid_cell(explode_pos))
    {
        simulate_bomb_explosion(state->obstacles_state, &state->obstacles_cnt, explode_pos);
    }

    if (action->plan.total_steps > 0)
    {
        state->car_state = action->plan.car_path[action->plan.total_steps - 1];
    }
}

/* 模式1（不按 ID 配对）：
 * 每轮在所有箱子中选择当前总代价最小的可执行任务。 */
static void plan_mode1_simple(void)
{
    planning_state_t state;
    Position merged_path[MAX_CAR_PATH];
    int merged_len = 0;
    int safety_round = 0;

    reset_planning_caches();
    load_state_from_globals(&state);

    while (state.boxes_cnt > 0 && state.targets_cnt > 0 && safety_round < 128)
    {
        int b;
        int best_valid = 0;
        int best_steps = 0;
        int best_box_index = -1;
        path_plan_result best_plan;

        for (b = 0; b < state.boxes_cnt; b++)
        {
            path_plan_result plan;
            int steps = integrated_path_output(MAP_ROWS, MAP_COLS,
                                               state.obstacles_state, state.obstacles_cnt,
                                               state.bombs_state, state.bombs_cnt,
                                               state.boxes_state, state.boxes_cnt,
                                               state.targets_state, state.targets_cnt,
                                               b,
                                               state.car_state,
                                               &plan);
            if (steps <= 0)
                continue;

            if (!best_valid || steps < best_steps)
            {
                best_valid = 1;
                best_steps = steps;
                best_box_index = b;
                best_plan = plan;
            }
        }

        if (!best_valid)
            break;

        append_segment_path(merged_path, &merged_len, best_plan.car_path, best_plan.total_steps);

        if (best_plan.used_bomb && best_plan.bomb_index >= 0 && best_plan.bomb_index < state.bombs_cnt)
        {
            Position used_bomb_pos = state.bombs_state[best_plan.bomb_index];
            Position explode_pos = best_plan.bomb_target;
            remove_bomb_by_position(state.bombs_state, &state.bombs_cnt, used_bomb_pos);
            if (!is_valid_cell(explode_pos))
                explode_pos = best_plan.wall_target;
            if (is_valid_cell(explode_pos))
            {
                simulate_bomb_explosion(state.obstacles_state, &state.obstacles_cnt, explode_pos);
            }
        }

        if (best_box_index >= 0 && best_box_index < state.boxes_cnt)
        {
            remove_position_at(state.boxes_state, &state.boxes_cnt, best_box_index);
        }

        {
            int target_index = find_position_index(state.targets_state, state.targets_cnt, best_plan.box_target);
            if (target_index >= 0)
            {
                remove_position_at(state.targets_state, &state.targets_cnt, target_index);
            }
        }

        if (best_plan.total_steps > 0)
        {
            state.car_state = best_plan.car_path[best_plan.total_steps - 1];
        }

        safety_round++;
    }

    save_state_to_globals(&state);
    save_path_to_globals(merged_path, merged_len);
}

/*
 * 模式2（按 ID 配对）：
 * 1) 初始化时建立 pair 表，跟踪每个目标点对应箱子；
 * 2) 每轮刷新 pair 状态后，统一比较“开路炸弹任务/推箱任务”；
 * 3) 执行动作并更新地图，直到所有可完成任务结束或无可行动作。 */
static void plan_mode2_pair_first(void)
{
    planning_state_t state;
    pair_task_table_t pairs;
    Position merged_path[MAX_CAR_PATH];
    int merged_len = 0;
    int safety_round = 0;

    reset_planning_caches();
    load_state_from_globals(&state);
    build_pair_table(&state, 1, &pairs);

    while (state.boxes_cnt > 0 && state.targets_cnt > 0 && safety_round < 256)
    {
        round_action_t action;
        int picked = 0;

        refresh_pair_statuses(&state, 1, &pairs);

        if (pick_best_round_action_balanced(&state, 1, &pairs, &action))
        {
            if (action.action_type == ACTION_BOX)
            {
                apply_box_action_result(&state, &pairs, &action, merged_path, &merged_len);
            }
            else
            {
                apply_bomb_action_result(&state, &action, merged_path, &merged_len);
            }
            picked = 1;
        }

        if (!picked)
            break;

        safety_round++;
    }

    save_state_to_globals(&state);
    save_path_to_globals(merged_path, merged_len);
}

void Plan_path_Mode1(void)
{
    plan_mode1_simple();
}

void Plan_path_Mode2(void)
{
    plan_mode2_pair_first();
}

