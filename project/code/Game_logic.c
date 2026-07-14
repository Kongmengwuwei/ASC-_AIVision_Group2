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
#define IDENTIFY_NEED_CACHE_SIZE 96
/* 每项包含完整 path_plan_result（约 3.4 KB）。96 项会占用约 321 KB DTCM，
 * 挤压规划调用栈；循环缓存缩到 16 项只降低命中率，不改变搜索结果。 */
#define IDENTIFY_BOMB_CACHE_SIZE 16

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

typedef struct
{
    uint8 valid;
    uint32 unlock_sig;
    uint8 need_unlock;
} identify_need_cache_entry_t;

typedef struct
{
    uint8 valid;
    uint32 state_sig;
    uint8 has_action;
    round_action_t action;
} identify_bomb_cache_entry_t;

static pair_reach_cache_entry_t s_pair_reach_cache[PAIR_REACH_CACHE_SIZE];
static infer_wall_cache_entry_t s_infer_wall_cache[INFER_WALL_CACHE_SIZE];
static critical_owner_cache_entry_t s_critical_owner_cache;
static identify_need_cache_entry_t s_identify_need_cache[IDENTIFY_NEED_CACHE_SIZE];
static identify_bomb_cache_entry_t s_identify_bomb_cache[IDENTIFY_BOMB_CACHE_SIZE];
static int s_pair_reach_cache_next = 0;
static int s_infer_wall_cache_next = 0;
static int s_identify_need_cache_next = 0;
static int s_identify_bomb_cache_next = 0;

/* 识别访问顺序：kind 0=箱子、1=目标；识别前 id 可能仍为 0。 */
uint8 g_identify_seq_kind[MAX_BOXES + MAX_TARGETS];
uint8 g_identify_seq_id[MAX_BOXES + MAX_TARGETS];
int g_identify_seq_len = 0;
Position g_blown_cell[MAX_OBSTACLES];
int g_blown_count = 0;
static void reset_planning_caches(void)
{
    memset(s_pair_reach_cache, 0, sizeof(s_pair_reach_cache));
    memset(s_infer_wall_cache, 0, sizeof(s_infer_wall_cache));
    memset(&s_critical_owner_cache, 0, sizeof(s_critical_owner_cache));
    memset(s_identify_need_cache, 0, sizeof(s_identify_need_cache));
    memset(s_identify_bomb_cache, 0, sizeof(s_identify_bomb_cache));
    s_pair_reach_cache_next = 0;
    s_infer_wall_cache_next = 0;
    s_identify_need_cache_next = 0;
    s_identify_bomb_cache_next = 0;

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

static uint8 merge_marker(uint8 old_marker, uint8 new_marker)
{
    return (uint8)((old_marker | new_marker) & PATH_ALL_EVENTS);
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

/* 仅用于“是否需开路”判定缓存：忽略 car 位置，聚焦静态地形与物体分布。 */
static uint32 hash_unlock_signature(const planning_state_t *state)
{
    int i;
    uint32 h = 2166136261u;
    if (!state)
        return 0;

    h = hash_mix_u32(h, (uint32)state->obstacles_cnt);
    h = hash_mix_u32(h, (uint32)state->bombs_cnt);
    h = hash_mix_u32(h, (uint32)state->boxes_cnt);
    h = hash_mix_u32(h, (uint32)state->targets_cnt);

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

static int lookup_identify_need_cache(uint32 unlock_sig, int *out_need_unlock)
{
    int i;
    if (!out_need_unlock)
        return 0;
    for (i = 0; i < IDENTIFY_NEED_CACHE_SIZE; i++)
    {
        const identify_need_cache_entry_t *e = &s_identify_need_cache[i];
        if (!e->valid)
            continue;
        if (e->unlock_sig != unlock_sig)
            continue;
        *out_need_unlock = e->need_unlock ? 1 : 0;
        return 1;
    }
    return 0;
}

static void store_identify_need_cache(uint32 unlock_sig, int need_unlock)
{
    identify_need_cache_entry_t *e;
    if (s_identify_need_cache_next < 0 || s_identify_need_cache_next >= IDENTIFY_NEED_CACHE_SIZE)
        s_identify_need_cache_next = 0;
    e = &s_identify_need_cache[s_identify_need_cache_next++];
    if (s_identify_need_cache_next >= IDENTIFY_NEED_CACHE_SIZE)
        s_identify_need_cache_next = 0;

    e->valid = 1;
    e->unlock_sig = unlock_sig;
    e->need_unlock = (uint8)(need_unlock ? 1 : 0);
}

static int lookup_identify_bomb_cache(uint32 state_sig, int *out_has_action, round_action_t *out_action)
{
    int i;
    if (!out_has_action || !out_action)
        return 0;
    for (i = 0; i < IDENTIFY_BOMB_CACHE_SIZE; i++)
    {
        const identify_bomb_cache_entry_t *e = &s_identify_bomb_cache[i];
        if (!e->valid)
            continue;
        if (e->state_sig != state_sig)
            continue;
        *out_has_action = e->has_action ? 1 : 0;
        *out_action = e->action;
        return 1;
    }
    return 0;
}

static void store_identify_bomb_cache(uint32 state_sig, int has_action, const round_action_t *action)
{
    identify_bomb_cache_entry_t *e;
    if (s_identify_bomb_cache_next < 0 || s_identify_bomb_cache_next >= IDENTIFY_BOMB_CACHE_SIZE)
        s_identify_bomb_cache_next = 0;
    e = &s_identify_bomb_cache[s_identify_bomb_cache_next++];
    if (s_identify_bomb_cache_next >= IDENTIFY_BOMB_CACHE_SIZE)
        s_identify_bomb_cache_next = 0;

    e->valid = 1;
    e->state_sig = state_sig;
    e->has_action = (uint8)(has_action ? 1 : 0);
    if (action)
        e->action = *action;
    else
        memset(&e->action, 0, sizeof(e->action));
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
    const Position *plan_obstacles;
    int plan_obstacles_cnt;
    int blocked_obstacles_cnt = 0;
    int has_forbidden_bomb = 0;
    int i;

    if (!state || !targets || targets_cnt <= 0 || !out_plan)
        return -1;
    if (box_index < 0 || box_index >= state->boxes_cnt)
        return -1;

    for (i = 0; i < state->bombs_cnt; i++)
    {
        Position b = state->bombs_state[i];
        if (!position_in_set(b, allowed_bombs, allowed_bombs_cnt))
        {
            if (!has_forbidden_bomb)
            {
                blocked_obstacles_cnt = state->obstacles_cnt;
                if (blocked_obstacles_cnt > grid_size)
                    blocked_obstacles_cnt = grid_size;
                memcpy(blocked_obstacles,
                       state->obstacles_state,
                       (size_t)blocked_obstacles_cnt * sizeof(Position));
                has_forbidden_bomb = 1;
            }
            if (blocked_obstacles_cnt < grid_size)
                blocked_obstacles[blocked_obstacles_cnt++] = b;
        }
    }

    plan_obstacles = has_forbidden_bomb ? blocked_obstacles : state->obstacles_state;
    plan_obstacles_cnt = has_forbidden_bomb ? blocked_obstacles_cnt : state->obstacles_cnt;

    return integrated_path_output(MAP_ROWS, MAP_COLS,
                                  plan_obstacles, plan_obstacles_cnt,
                                  allowed_bombs, allowed_bombs_cnt,
                                  state->boxes_state, state->boxes_cnt,
                                  targets, targets_cnt,
                                  box_index,
                                  car_start,
                                  out_plan);
}

static int plan_box_exists_with_candidates(const planning_state_t *state,
                                           int box_index,
                                           const Position *allowed_bombs, int allowed_bombs_cnt,
                                           const Position *targets, int targets_cnt,
                                           Position car_start)
{
    Position blocked_obstacles[grid_size];
    const Position *plan_obstacles;
    int plan_obstacles_cnt;
    int blocked_obstacles_cnt = 0;
    int has_forbidden_bomb = 0;
    int i;

    if (!state || !targets || targets_cnt <= 0)
        return -1;
    if (box_index < 0 || box_index >= state->boxes_cnt)
        return -1;

    for (i = 0; i < state->bombs_cnt; i++)
    {
        Position b = state->bombs_state[i];
        if (!position_in_set(b, allowed_bombs, allowed_bombs_cnt))
        {
            if (!has_forbidden_bomb)
            {
                blocked_obstacles_cnt = state->obstacles_cnt;
                if (blocked_obstacles_cnt > grid_size)
                    blocked_obstacles_cnt = grid_size;
                memcpy(blocked_obstacles,
                       state->obstacles_state,
                       (size_t)blocked_obstacles_cnt * sizeof(Position));
                has_forbidden_bomb = 1;
            }
            if (blocked_obstacles_cnt < grid_size)
                blocked_obstacles[blocked_obstacles_cnt++] = b;
        }
    }

    plan_obstacles = has_forbidden_bomb ? blocked_obstacles : state->obstacles_state;
    plan_obstacles_cnt = has_forbidden_bomb ? blocked_obstacles_cnt : state->obstacles_cnt;

    return integrated_path_exists(MAP_ROWS, MAP_COLS,
                                  plan_obstacles, plan_obstacles_cnt,
                                  allowed_bombs, allowed_bombs_cnt,
                                  state->boxes_state, state->boxes_cnt,
                                  targets, targets_cnt,
                                  box_index,
                                  car_start);
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
    int steps;

    if (!is_car_start_candidate_free(state, car_start))
        return 0;

    one_target[0] = target;
    steps = plan_box_exists_with_candidates(state, box_index, 0, 0, one_target, 1, car_start);
    return (steps > 0) ? 1 : 0;
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

    /* 推断首先从当前小车位置规划，缓存键必须包含 car_state。 */
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
    int steps;

    if (!state || !pair || !pair->valid || pair->target_done)
        return 0;
    if (!state_has_target(state, pair->target_ref))
        return 0;

    /* “当前可执行”包含小车能否到达起推位，不能跨 car_state 复用。 */
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

    steps = plan_box_exists_with_candidates(state,
                                            box_index,
                                            all_bombs,
                                            all_bombs_cnt,
                                            one_target,
                                            1,
                                            state->car_state);
    cached_reachable = (steps > 0) ? 1 : 0;
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

    /* 归属分析会调用依赖当前小车位置的可达性判断。 */
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

/* 箱子在给定状态下是否至少有一个合法推动方向（站位与落点都空）。 */
static int box_has_any_legal_push(const planning_state_t *state, Position box_pos)
{
    const int dr[4] = {-1, 1, 0, 0};
    const int dc[4] = {0, 0, -1, 1};
    int k;
    if (!state || !is_valid_cell(box_pos))
        return 0;
    for (k = 0; k < 4; k++)
    {
        /* 车站在推动方向的反侧，把箱子推向 dest。 */
        Position stand = {(uint8)((int)box_pos.row - dr[k]), (uint8)((int)box_pos.col - dc[k]), 0};
        Position dest  = {(uint8)((int)box_pos.row + dr[k]), (uint8)((int)box_pos.col + dc[k]), 0};
        if (!is_valid_cell(stand) || !is_valid_cell(dest))
            continue;
        if (find_position_index(state->obstacles_state, state->obstacles_cnt, stand) >= 0) continue;
        if (find_position_index(state->bombs_state, state->bombs_cnt, stand) >= 0) continue;
        if (find_position_index(state->boxes_state, state->boxes_cnt, stand) >= 0) continue;
        if (find_position_index(state->obstacles_state, state->obstacles_cnt, dest) >= 0) continue;
        if (find_position_index(state->bombs_state, state->bombs_cnt, dest) >= 0) continue;
        if (find_position_index(state->boxes_state, state->boxes_cnt, dest) >= 0) continue;
        return 1;
    }
    return 0;
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

		int best_progress = 0;      /* 新增：当前最优动作是否“解放了被卡箱子” */
		int box_stuck_before = 0;   /* 新增：炸前该箱子是否零可推方向 */
		Position pair_box_pos;      /* 新增 */

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

		pair_box_pos = state->boxes_state[box_index];
		box_stuck_before = !box_has_any_legal_push(state, pair_box_pos);

    ranked_cnt = build_ranked_bomb_indices_for_pair(state, pair, box_index, ranked_bombs);
    if (ranked_cnt <= 0)
        return 0;

    /* 仅保留“该 pair 当前最优炸弹动作”：
     * - 第一轮只看 TopK 炸弹 + TopK 墙点，快速拿到高质量候选；
     * - 若第一轮无解，再扩大搜索范围兜底；
     * - 每个候选先用曼哈顿下界剪枝，再调用底层规划。 */
    for (pass = 0; pass < 2 && (!has_best || (box_stuck_before && !best_progress)); pass++)
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

						/* 廉价预筛（仅围死箱子时）：先用一次爆破模拟判断“炸开这堵墙的 3x3
							 是否真能让箱子多出可推方向”，否则跳过昂贵的 plan_bomb_task_in_state，
							 把围死场景下的底层规划调用从“所有候选墙”降到“仅有效墙”。 */
						if (box_stuck_before)
						{
								planning_state_t probe = *state;
								simulate_bomb_explosion(probe.obstacles_state, &probe.obstacles_cnt, wall);
								if (!box_has_any_legal_push(&probe, pair_box_pos))
										continue;
						}
                /* 曼哈顿下界剪枝：已有更优候选时，提前跳过昂贵规划。 */
                lb = manhattan_cell_dist(state->car_state, primary_bomb) - 1;
                if (lb < 0)
                    lb = 0;
                lb += manhattan_cell_dist(primary_bomb, wall);
                /* 被卡箱子且尚未找到解放动作时，不做步数剪枝，避免漏掉步数偏大的解放墙。 */
								if (has_best && lb >= best_steps && (!box_stuck_before || best_progress))
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

                {
										int cand_progress = 0;
										if (box_stuck_before)
										{
												planning_state_t shadow = *state;
												apply_bomb_action_result(&shadow, &candidate, 0, 0);
												if (box_has_any_legal_push(&shadow, pair_box_pos))
														cand_progress = 1;   /* 炸完箱子从“零可推”变“可推” */
										}

										if (!has_best ||
												cand_progress > best_progress ||
												(cand_progress == best_progress && candidate.steps < best_steps) ||
												(cand_progress == best_progress && candidate.steps == best_steps &&
												 candidate.has_support_bomb_pos < best_action.has_support_bomb_pos))
										{
												has_best = 1;
												best_progress = cand_progress;
												best_steps = candidate.steps;
												best_action = candidate;
										}
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
    int oi;
    int eval_cnt = 0;
    int out_cnt = 0;
    int best_steps_seen = INT_MAX;
    Position all_bombs[MAX_BOMBS];
    int all_bombs_cnt;
    int pair_order[MAX_TARGETS];
    int pair_box_index[MAX_TARGETS];
    int pair_lb[MAX_TARGETS];
    int pair_score[MAX_TARGETS];

    if (!state || !pairs || !out_actions || max_actions <= 0)
        return 0;

    all_bombs_cnt = state->bombs_cnt;
    memcpy(all_bombs, state->bombs_state, (size_t)all_bombs_cnt * sizeof(Position));

    for (p = 0; p < pairs->count && eval_cnt < MAX_TARGETS; p++)
    {
        const pair_task_t *pair = &pairs->item[p];
        int box_index;
        int lb;

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

        /* 先按轻量估计排序，让后续安全下界剪枝尽早拿到较小上界。 */
        lb = manhattan_cell_dist(state->boxes_state[box_index], pair->target_ref);
        pair_order[eval_cnt] = p;
        pair_box_index[eval_cnt] = box_index;
        pair_lb[eval_cnt] = lb;
        pair_score[eval_cnt] = lb + manhattan_cell_dist(state->car_state, state->boxes_state[box_index]);
        eval_cnt++;
    }

    for (oi = 0; oi < eval_cnt; oi++)
    {
        int j;
        int best = oi;
        for (j = oi + 1; j < eval_cnt; j++)
        {
            if (pair_score[j] < pair_score[best] ||
                (pair_score[j] == pair_score[best] && pair_order[j] < pair_order[best]))
            {
                best = j;
            }
        }
        if (best != oi)
        {
            int tmp_order = pair_order[oi];
            int tmp_box = pair_box_index[oi];
            int tmp_lb = pair_lb[oi];
            int tmp_score = pair_score[oi];
            pair_order[oi] = pair_order[best];
            pair_box_index[oi] = pair_box_index[best];
            pair_lb[oi] = pair_lb[best];
            pair_score[oi] = pair_score[best];
            pair_order[best] = tmp_order;
            pair_box_index[best] = tmp_box;
            pair_lb[best] = tmp_lb;
            pair_score[best] = tmp_score;
        }
    }

    for (oi = 0; oi < eval_cnt && out_cnt < max_actions; oi++)
    {
        const pair_task_t *pair;
        int box_index;
        Position one_target[1];
        path_plan_result plan;
        int steps;
        round_action_t candidate;

        if (best_steps_seen < INT_MAX && pair_lb[oi] >= best_steps_seen)
            continue;

        p = pair_order[oi];
        pair = &pairs->item[p];
        box_index = pair_box_index[oi];

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

    /* 压缩相邻重复格：去除回放时“原地不动”的步（合并其标记） */
    int uniq_len = 0;
    for (int i = 0; i < merged_len; i++)
    {
        if (uniq_len > 0 && same_cell(car_path[uniq_len - 1], merged_path[i]))
        {
            car_path[uniq_len - 1].id = merge_marker(car_path[uniq_len - 1].id, merged_path[i].id);
        }
        else
        {
            car_path[uniq_len++] = merged_path[i];
        }
    }
    Car_path_count = (size_t)uniq_len;
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

/* 判断返场路径上的某格是否可通行（障碍/炸弹/箱子均视为阻塞）。 */
static int is_blocked_for_return(const planning_state_t *state, int row, int col)
{
    Position p;
    if (!state)
        return 1;
    if (row < 0 || row >= MAP_ROWS || col < 0 || col >= MAP_COLS)
        return 1;

    p.row = (uint8)row;
    p.col = (uint8)col;
    p.id = 0;

    if (find_position_index(state->obstacles_state, state->obstacles_cnt, p) >= 0)
        return 1;
    if (find_position_index(state->bombs_state, state->bombs_cnt, p) >= 0)
        return 1;
    if (find_position_index(state->boxes_state, state->boxes_cnt, p) >= 0)
        return 1;
    return 0;
}

/* 判断最终返场路径上的某格是否可通行。
 * 所有箱子完成后，场地障碍物会消失，因此可按需忽略 obstacles_state。 */
static int is_blocked_for_final_return(const planning_state_t *state,
                                       int row,
                                       int col,
                                       int ignore_obstacles)
{
    Position p;
    if (!state)
        return 1;
    if (row < 0 || row >= MAP_ROWS || col < 0 || col >= MAP_COLS)
        return 1;

    p.row = (uint8)row;
    p.col = (uint8)col;
    p.id = 0;

    if (!ignore_obstacles &&
        find_position_index(state->obstacles_state, state->obstacles_cnt, p) >= 0)
    {
        return 1;
    }
    if (find_position_index(state->bombs_state, state->bombs_cnt, p) >= 0)
        return 1;
    if (find_position_index(state->boxes_state, state->boxes_cnt, p) >= 0)
        return 1;
    return 0;
}

/* 规划返回左侧发车区的最短车行路径。
 * 控制层下一次发车默认向右离开发车区，因此不能为缩短路径改停到右边界。 */
static int build_shortest_return_path_to_depot(const planning_state_t *state,
                                                Position *out_path,
                                                int max_path,
                                                Position *out_depot,
                                                int ignore_obstacles)
{
    const Position depots[2] = {
        {4, 0, 0},
        {5, 0, 0}
    };
    const int dr[4] = {-1, 1, 0, 0};
    const int dc[4] = {0, 0, -1, 1};
    int dist[MAP_ROWS * MAP_COLS];
    int prev[MAP_ROWS * MAP_COLS];
    int queue[MAP_ROWS * MAP_COLS];
    int qh = 0;
    int qt = 0;
    int start_idx;
    int i;
    int best_depot_idx = -1;
    int best_dist = INT_MAX;
    int path_len;
    int cur;
    Position reverse_path[MAP_ROWS * MAP_COLS];

    if (!state || !out_path || max_path <= 0)
        return 0;
    if (state->car_state.row >= MAP_ROWS || state->car_state.col >= MAP_COLS)
        return 0;

    for (i = 0; i < MAP_ROWS * MAP_COLS; i++)
    {
        dist[i] = -1;
        prev[i] = -1;
    }

    start_idx = (int)state->car_state.row * MAP_COLS + (int)state->car_state.col;
    dist[start_idx] = 0;
    queue[qt++] = start_idx;

    while (qh < qt)
    {
        int idx = queue[qh++];
        int row = idx / MAP_COLS;
        int col = idx % MAP_COLS;
        int k;

        for (k = 0; k < 4; k++)
        {
            int nr = row + dr[k];
            int nc = col + dc[k];
            int nidx;
            if (nr < 0 || nr >= MAP_ROWS || nc < 0 || nc >= MAP_COLS)
                continue;
            if (is_blocked_for_final_return(state, nr, nc, ignore_obstacles))
                continue;

            nidx = nr * MAP_COLS + nc;
            if (dist[nidx] >= 0)
                continue;
            dist[nidx] = dist[idx] + 1;
            prev[nidx] = idx;
            queue[qt++] = nidx;
        }
    }

    for (i = 0; i < (int)(sizeof(depots) / sizeof(depots[0])); i++)
    {
        int drow = (int)depots[i].row;
        int dcol = (int)depots[i].col;
        int didx;
        if (drow < 0 || drow >= MAP_ROWS || dcol < 0 || dcol >= MAP_COLS)
            continue;
        if (is_blocked_for_final_return(state, drow, dcol, ignore_obstacles))
            continue;
        didx = drow * MAP_COLS + dcol;
        if (dist[didx] < 0)
            continue;
        if (dist[didx] < best_dist)
        {
            best_dist = dist[didx];
            best_depot_idx = i;
        }
    }

    if (best_depot_idx < 0)
        return 0;

    if (out_depot)
        *out_depot = depots[best_depot_idx];

    cur = ((int)depots[best_depot_idx].row) * MAP_COLS + (int)depots[best_depot_idx].col;
    path_len = 0;
    while (cur >= 0 && path_len < MAP_ROWS * MAP_COLS)
    {
        reverse_path[path_len].row = (uint8)(cur / MAP_COLS);
        reverse_path[path_len].col = (uint8)(cur % MAP_COLS);
        reverse_path[path_len].id = 0;
        path_len++;
        if (cur == start_idx)
            break;
        cur = prev[cur];
    }

    if (path_len <= 0 || reverse_path[path_len - 1].row != state->car_state.row ||
        reverse_path[path_len - 1].col != state->car_state.col)
    {
        return 0;
    }

    if (path_len > max_path)
        path_len = max_path;
    for (i = 0; i < path_len; i++)
    {
        out_path[i] = reverse_path[path_len - 1 - i];
    }
    return path_len;
}

/* 在现有规划末尾追加返回左侧发车区的路径。 */
static int append_return_to_depot(planning_state_t *state,
                                  Position *merged_path,
                                  int *merged_len)
{
    Position ret_path[MAP_ROWS * MAP_COLS];
    Position depot;
    int ret_len;
    int ignore_obstacles;

    if (!state || !merged_path || !merged_len)
        return 0;

    ignore_obstacles = (state->boxes_cnt <= 0) ? 1 : 0;
    ret_len = build_shortest_return_path_to_depot(state,
                                                  ret_path,
                                                  MAP_ROWS * MAP_COLS,
                                                  &depot,
                                                  ignore_obstacles);
    if (ret_len <= 0)
        return 0;

    append_segment_path(merged_path, merged_len, ret_path, ret_len);
    state->car_state = depot;
    return 1;
}

/* 仅小车移动的最短路径（4邻域 BFS）。 */
static int build_car_shortest_path(const planning_state_t *state,
                                   Position start,
                                   Position goal,
                                   Position *out_path,
                                   int max_path)
{
    const int dr[4] = {-1, 1, 0, 0};
    const int dc[4] = {0, 0, -1, 1};
    int dist[MAP_ROWS * MAP_COLS];
    int prev[MAP_ROWS * MAP_COLS];
    int queue[MAP_ROWS * MAP_COLS];
    int qh = 0;
    int qt = 0;
    int start_idx;
    int goal_idx;
    int i;
    int cur;
    Position reverse_path[MAP_ROWS * MAP_COLS];
    int path_len = 0;

    if (!state || !out_path || max_path <= 0)
        return 0;
    if (!is_valid_cell(start) || !is_valid_cell(goal))
        return 0;
    if (is_blocked_for_return(state, goal.row, goal.col) && !same_cell(start, goal))
        return 0;

    for (i = 0; i < MAP_ROWS * MAP_COLS; i++)
    {
        dist[i] = -1;
        prev[i] = -1;
    }

    start_idx = (int)start.row * MAP_COLS + (int)start.col;
    goal_idx = (int)goal.row * MAP_COLS + (int)goal.col;
    dist[start_idx] = 0;
    queue[qt++] = start_idx;

    while (qh < qt)
    {
        int idx = queue[qh++];
        int row = idx / MAP_COLS;
        int col = idx % MAP_COLS;
        int k;

        if (idx == goal_idx)
            break;

        for (k = 0; k < 4; k++)
        {
            int nr = row + dr[k];
            int nc = col + dc[k];
            int nidx;
            if (nr < 0 || nr >= MAP_ROWS || nc < 0 || nc >= MAP_COLS)
                continue;
            if (is_blocked_for_return(state, nr, nc))
                continue;
            nidx = nr * MAP_COLS + nc;
            if (dist[nidx] >= 0)
                continue;
            dist[nidx] = dist[idx] + 1;
            prev[nidx] = idx;
            queue[qt++] = nidx;
        }
    }

    if (dist[goal_idx] < 0)
        return 0;

    cur = goal_idx;
    while (cur >= 0 && path_len < MAP_ROWS * MAP_COLS)
    {
        reverse_path[path_len].row = (uint8)(cur / MAP_COLS);
        reverse_path[path_len].col = (uint8)(cur % MAP_COLS);
        reverse_path[path_len].id = 0;
        path_len++;
        if (cur == start_idx)
            break;
        cur = prev[cur];
    }

    if (path_len <= 0)
        return 0;
    if (path_len > max_path)
        path_len = max_path;

    for (i = 0; i < path_len; i++)
    {
        out_path[i] = reverse_path[path_len - 1 - i];
    }
    return path_len;
}

/* 单次 BFS 生成从 start 到全图的最短步数与前驱表，用于复用。 */
static int build_car_distance_prev_map(const planning_state_t *state,
                                       Position start,
                                       int *dist,
                                       int *prev)
{
    const int dr[4] = {-1, 1, 0, 0};
    const int dc[4] = {0, 0, -1, 1};
    int queue[MAP_ROWS * MAP_COLS];
    int qh = 0;
    int qt = 0;
    int i;
    int start_idx;

    if (!state || !dist || !prev)
        return 0;
    if (!is_valid_cell(start))
        return 0;

    for (i = 0; i < MAP_ROWS * MAP_COLS; i++)
    {
        dist[i] = -1;
        prev[i] = -1;
    }

    start_idx = (int)start.row * MAP_COLS + (int)start.col;
    dist[start_idx] = 0;
    queue[qt++] = start_idx;

    while (qh < qt)
    {
        int idx = queue[qh++];
        int row = idx / MAP_COLS;
        int col = idx % MAP_COLS;
        int k;

        for (k = 0; k < 4; k++)
        {
            int nr = row + dr[k];
            int nc = col + dc[k];
            int nidx;
            if (nr < 0 || nr >= MAP_ROWS || nc < 0 || nc >= MAP_COLS)
                continue;
            if (is_blocked_for_return(state, nr, nc))
                continue;
            nidx = nr * MAP_COLS + nc;
            if (dist[nidx] >= 0)
                continue;
            dist[nidx] = dist[idx] + 1;
            prev[nidx] = idx;
            queue[qt++] = nidx;
        }
    }
    return 1;
}

/* 由 prev 前驱表回溯一条最短路径。 */
static int rebuild_car_path_from_prev(Position start,
                                      Position goal,
                                      const int *prev,
                                      Position *out_path,
                                      int max_path)
{
    Position reverse_path[MAP_ROWS * MAP_COLS];
    int start_idx;
    int goal_idx;
    int cur;
    int len = 0;
    int i;

    if (!prev || !out_path || max_path <= 0)
        return 0;
    if (!is_valid_cell(start) || !is_valid_cell(goal))
        return 0;

    start_idx = (int)start.row * MAP_COLS + (int)start.col;
    goal_idx = (int)goal.row * MAP_COLS + (int)goal.col;
    cur = goal_idx;

    while (cur >= 0 && len < MAP_ROWS * MAP_COLS)
    {
        reverse_path[len].row = (uint8)(cur / MAP_COLS);
        reverse_path[len].col = (uint8)(cur % MAP_COLS);
        reverse_path[len].id = 0;
        len++;
        if (cur == start_idx)
            break;
        cur = prev[cur];
    }

    if (len <= 0 || cur != start_idx)
        return 0;
    if (len > max_path)
        len = max_path;

    for (i = 0; i < len; i++)
    {
        out_path[i] = reverse_path[len - 1 - i];
    }
    return len;
}

static int identify_stand_has_clear_view(const planning_state_t *state,
                                         Position object_pos,
                                         Position stand)
{
    int dr = (int)stand.row - (int)object_pos.row;
    int dc = (int)stand.col - (int)object_pos.col;
    int abs_dr = dr;
    int abs_dc = dc;
    int dist;
    Position mid;

    if (!state || !is_valid_cell(object_pos) || !is_valid_cell(stand))
        return 0;
    if (abs_dr < 0)
        abs_dr = -abs_dr;
    if (abs_dc < 0)
        abs_dc = -abs_dc;
    dist = abs_dr + abs_dc;
    if ((dist != 1 && dist != 2) || (abs_dr != 0 && abs_dc != 0))
        return 0;
    if (dist == 1)
        return 1;

    mid.row = (uint8)((int)object_pos.row + ((dr > 0) ? 1 : ((dr < 0) ? -1 : 0)));
    mid.col = (uint8)((int)object_pos.col + ((dc > 0) ? 1 : ((dc < 0) ? -1 : 0)));
    mid.id = 0;

    if (find_position_index(state->obstacles_state, state->obstacles_cnt, mid) >= 0)
        return 0;
    if (find_position_index(state->bombs_state, state->bombs_cnt, mid) >= 0)
        return 0;
    if (find_position_index(state->boxes_state, state->boxes_cnt, mid) >= 0)
        return 0;
    if (find_position_index(state->targets_state, state->targets_cnt, mid) >= 0)
        return 0;
    return 1;
}

/* 在 dist 图上选择“对象四方向 1/2 格中最近站位”，避免重复求最短路。 */
static int pick_best_adjacent_stand_by_dist(const planning_state_t *state,
                                            Position object_pos,
                                            const int *dist,
                                            Position *out_stand,
                                            int *out_steps)
{
    const int dr[4] = {-1, 1, 0, 0};
    const int dc[4] = {0, 0, -1, 1};
    int k;
    int distance;
    int best_steps = INT_MAX;
    int best_distance = INT_MAX;
    Position best_stand = {0, 0, 0};

    if (!state || !dist || !is_valid_cell(object_pos))
        return 0;

    for (distance = 1; distance <= 2; distance++)  /* 识别允许1/2格（直线、中间无遮挡），与 plus 的 REACH_DIST_BOTH 对齐 */
    {
        for (k = 0; k < 4; k++)
        {
            Position stand = {(uint8)((int)object_pos.row + dr[k] * distance),
                              (uint8)((int)object_pos.col + dc[k] * distance),
                              0};
            int idx;
            int steps;
            if (!is_valid_cell(stand))
                continue;
            if (!identify_stand_has_clear_view(state, object_pos, stand))
                continue;
            idx = (int)stand.row * MAP_COLS + (int)stand.col;
            steps = dist[idx];
            if (steps < 0)
                continue;
            if (steps < best_steps ||
                (steps == best_steps && distance < best_distance))
            {
                best_steps = steps;
                best_distance = distance;
                best_stand = stand;
            }
        }
    }

    if (best_steps == INT_MAX)
        return 0;
    if (out_stand)
        *out_stand = best_stand;
    if (out_steps)
        *out_steps = best_steps;
    return 1;
}

/* 基于当前 dist 图给炸弹动作一个安全下界：
 * 炸弹动作至少要先到达某颗炸弹可推站位，因此下界为该最小站位距离。 */
static int estimate_bomb_action_lb_by_dist(const planning_state_t *state, const int *dist)
{
    const int dr[4] = {-1, 1, 0, 0};
    const int dc[4] = {0, 0, -1, 1};
    int b, k;
    int best = INT_MAX;

    if (!state || !dist || state->bombs_cnt <= 0)
        return INT_MAX;

    for (b = 0; b < state->bombs_cnt; b++)
    {
        Position bomb = state->bombs_state[b];
        for (k = 0; k < 4; k++)
        {
            Position stand = {(uint8)((int)bomb.row + dr[k]), (uint8)((int)bomb.col + dc[k]), 0};
            int idx;
            int d;
            if (!is_valid_cell(stand))
                continue;
            idx = (int)stand.row * MAP_COLS + (int)stand.col;
            d = dist[idx];
            if (d < 0)
                continue;
            if (d < best)
                best = d;
        }
    }
    return best;
}

/* 识别点统一标记，具体一格/两格距离交给控制流程实时判断。 */
static uint8 identify_marker_by_relative(Position stand, Position object_pos)
{
    (void)stand;
    (void)object_pos;
    return IDENTIFICATION;
}

/* 无炸弹前提下，判断该格是否可作为推箱相关通行格。 */
static int is_push_blocked_cell_no_bomb(const planning_state_t *state, Position p)
{
    if (!state || !is_valid_cell(p))
        return 1;
    if (find_position_index(state->obstacles_state, state->obstacles_cnt, p) >= 0)
        return 1;
    if (find_position_index(state->bombs_state, state->bombs_cnt, p) >= 0)
        return 1;
    return 0;
}

/* 判断箱子是否被“墙/边界”卡住到无法起推（无炸弹视角）。 */
static int box_is_wall_stuck_no_bomb(const planning_state_t *state, Position box_pos)
{
    const int dr[4] = {-1, 1, 0, 0};
    const int dc[4] = {0, 0, -1, 1};
    int k;
    if (!state || !is_valid_cell(box_pos))
        return 1;

    for (k = 0; k < 4; k++)
    {
        Position front = {(uint8)((int)box_pos.row + dr[k]), (uint8)((int)box_pos.col + dc[k]), 0};
        Position back = {(uint8)((int)box_pos.row - dr[k]), (uint8)((int)box_pos.col - dc[k]), 0};
        if (!is_valid_cell(front) || !is_valid_cell(back))
            continue;
        if (!is_push_blocked_cell_no_bomb(state, front) &&
            !is_push_blocked_cell_no_bomb(state, back))
        {
            return 0;
        }
    }
    return 1;
}

/* 判断某目标点是否存在至少一个箱子可在无炸弹条件下推达。 */
static int target_reachable_by_any_box_no_bomb(const planning_state_t *state,
                                               Position target,
                                               const uint8 *box_done,
                                               int box_cnt)
{
    int i;
    (void)box_done;
    (void)box_cnt;
    if (!state || !is_valid_cell(target))
        return 0;

    for (i = 0; i < state->boxes_cnt; i++)
    {
        planning_state_t simplified;

        simplified = *state;
        simplified.bombs_cnt = 0;
        simplified.boxes_cnt = 1;
        simplified.targets_cnt = 1;
        simplified.boxes_state[0] = state->boxes_state[i];
        simplified.targets_state[0] = target;

        if (box_reachable_no_bomb_relaxed(&simplified, 0, target))
            return 1;
    }
    return 0;
}

/* 识别阶段是否“应提前开路”：
 * - 仍有目标点在无炸弹条件下不可由任一箱子推达；
 * - 或仍有箱子被墙/边界卡到无法起推。 */
static int identify_need_proactive_unlock(const planning_state_t *state,
                                          const uint8 *box_done,
                                          int box_cnt,
                                          const uint8 *target_done,
                                          int target_cnt)
{
    int i;
    uint32 unlock_sig;
    int cached_need = 0;
    (void)box_done;
    (void)box_cnt;
    (void)target_done;
    (void)target_cnt;
    if (!state)
        return 0;

    unlock_sig = hash_unlock_signature(state);
    if (lookup_identify_need_cache(unlock_sig, &cached_need))
        return cached_need;

    for (i = 0; i < state->targets_cnt; i++)
    {
        if (!target_reachable_by_any_box_no_bomb(state,
                                                 state->targets_state[i],
                                                 box_done,
                                                 box_cnt))
        {
            store_identify_need_cache(unlock_sig, 1);
            return 1;
        }
    }

    for (i = 0; i < state->boxes_cnt; i++)
    {
        if (box_is_wall_stuck_no_bomb(state, state->boxes_state[i]))
        {
            store_identify_need_cache(unlock_sig, 1);
            return 1;
        }
    }

    store_identify_need_cache(unlock_sig, 0);
    return 0;
}

/* 规划小车到“目标点四方向 1/2 格任一可达格”的最短路径。 */
static int build_best_adjacent_identify_path(const planning_state_t *state,
                                             Position object_pos,
                                             Position *out_path,
                                             int max_path,
                                             Position *out_stand)
{
    const int dr[4] = {-1, 1, 0, 0};
    const int dc[4] = {0, 0, -1, 1};
    int k;
    int distance;
    int best_len = 0;
    int best_distance = INT_MAX;
    Position best_path[MAP_ROWS * MAP_COLS];
    Position stand;

    if (!state || !out_path || max_path <= 0)
        return 0;
    if (!is_valid_cell(state->car_state) || !is_valid_cell(object_pos))
        return 0;

    for (distance = 1; distance <= 2; distance++)  /* 识别允许1/2格（直线、中间无遮挡），与 plus 的 REACH_DIST_BOTH 对齐 */
    {
        for (k = 0; k < 4; k++)
        {
            Position tmp_path[MAP_ROWS * MAP_COLS];
            int len;

            stand.row = (uint8)((int)object_pos.row + dr[k] * distance);
            stand.col = (uint8)((int)object_pos.col + dc[k] * distance);
            stand.id = 0;
            if (!is_valid_cell(stand))
                continue;
            if (!identify_stand_has_clear_view(state, object_pos, stand))
                continue;

            len = build_car_shortest_path(state,
                                          state->car_state,
                                          stand,
                                          tmp_path,
                                          MAP_ROWS * MAP_COLS);
            if (len <= 0)
                continue;

            if (best_len == 0 ||
                len < best_len ||
                (len == best_len && distance < best_distance))
            {
                best_len = len;
                best_distance = distance;
                memcpy(best_path, tmp_path, (size_t)len * sizeof(Position));
                if (out_stand)
                    *out_stand = stand;
            }
        }
    }

    if (best_len <= 0)
        return 0;
    if (best_len > max_path)
        best_len = max_path;
    memcpy(out_path, best_path, (size_t)best_len * sizeof(Position));
    return best_len;
}

/* 为识别模式收集“值得炸开”的墙点候选。 */
static int collect_identify_wall_candidates(const planning_state_t *state,
                                            const uint8 *box_done,
                                            int box_cnt,
                                            const uint8 *target_done,
                                            int target_cnt,
                                            Position *out_walls,
                                            int max_walls)
{
    const int dr[4] = {-1, 1, 0, 0};
    const int dc[4] = {0, 0, -1, 1};
    int i, k;
    int distance;
    int cnt = 0;
    (void)box_done;
    (void)box_cnt;
    (void)target_done;
    (void)target_cnt;

    if (!state || !out_walls || max_walls <= 0)
        return 0;

    for (i = 0; i < state->boxes_cnt; i++)
    {
        Position obj = state->boxes_state[i];
        for (distance = 1; distance <= 2; distance++)
        {
            for (k = 0; k < 4; k++)
            {
                Position stand = {(uint8)((int)obj.row + dr[k] * distance),
                                  (uint8)((int)obj.col + dc[k] * distance),
                                  0};
                Position block_bomb;
                if (!is_valid_cell(stand))
                    continue;
                if (state_has_obstacle_at(state, stand))
                    append_unique_wall_candidate(out_walls, &cnt, max_walls, stand);
                if (state_find_bomb_at(state, stand, &block_bomb))
                    append_bomb_local_unlock_walls(state, block_bomb, out_walls, &cnt, max_walls);
            }
        }
    }

    for (i = 0; i < state->targets_cnt; i++)
    {
        Position obj = state->targets_state[i];
        for (distance = 1; distance <= 2; distance++)
        {
            for (k = 0; k < 4; k++)
            {
                Position stand = {(uint8)((int)obj.row + dr[k] * distance),
                                  (uint8)((int)obj.col + dc[k] * distance),
                                  0};
                Position block_bomb;
                if (!is_valid_cell(stand))
                    continue;
                if (state_has_obstacle_at(state, stand))
                    append_unique_wall_candidate(out_walls, &cnt, max_walls, stand);
                if (state_find_bomb_at(state, stand, &block_bomb))
                    append_bomb_local_unlock_walls(state, block_bomb, out_walls, &cnt, max_walls);
            }
        }
    }

    /* 局部候选优先，剩余容量用全局障碍补齐。某些开路墙并不紧邻箱子/目标，
     * 只在局部候选为空时兜底会漏掉这类必须经过的中间通道。 */
    for (i = 0; i < state->obstacles_cnt && cnt < max_walls; i++)
        append_unique_wall_candidate(out_walls, &cnt, max_walls, state->obstacles_state[i]);

    if (cnt <= 0)
    {
        return 0;
    }

    return cnt;
}

/* 识别模式炸弹动作选择：
 * 当识别点都不可达时，按“最短可行动作”挑一条炸弹开路任务。 */
static int pick_best_unlock_bomb_action_for_identify(const planning_state_t *state,
                                                     const uint8 *box_done,
                                                     int box_cnt,
                                                     const uint8 *target_done,
                                                     int target_cnt,
                                                     round_action_t *out_action)
{
    int b;
    int border[MAX_BOMBS];
    int border_lb[MAX_BOMBS];
    static int car_dist[MAP_ROWS * MAP_COLS];
    static int car_prev[MAP_ROWS * MAP_COLS];
    int has_best = 0;
    int best_steps = INT_MAX;
    round_action_t best_action;
    round_action_t cached_action;
    Position walls[MAX_UNLOCK_WALL_CANDIDATES * 2];
    int wall_cnt;
    int cached_has_action = 0;
    uint32 state_sig;

    if (!state || !out_action)
        return 0;
    if (state->bombs_cnt <= 0)
        return 0;

    state_sig = hash_state_signature(state);
    if (lookup_identify_bomb_cache(state_sig, &cached_has_action, &cached_action))
    {
        if (cached_has_action)
        {
            *out_action = cached_action;
            return 1;
        }
        return 0;
    }

    wall_cnt = collect_identify_wall_candidates(state,
                                                box_done,
                                                box_cnt,
                                                target_done,
                                                target_cnt,
                                                walls,
                                                MAX_UNLOCK_WALL_CANDIDATES * 2);

    if (wall_cnt <= 0)
    {
        store_identify_bomb_cache(state_sig, 0, 0);
        return 0;
    }

    /* 计划级单炸弹优先: 按 (car->bomb + bomb->最近墙) 下界升序评估炸弹,
       尽早收紧 best_steps, 使后续炸弹的墙循环被已有的 lb>=best_steps 剪枝更早跳出.
       纯评估顺序, 不改变步数最优解. */
    {
        const int ddr[4] = {-1, 1, 0, 0};
        const int ddc[4] = {0, 0, -1, 1};
        int bo, bc, bk;
        build_car_distance_prev_map(state, state->car_state, car_dist, car_prev);
        for (bo = 0; bo < state->bombs_cnt; bo++)
        {
            Position bomb = state->bombs_state[bo];
            int minw = INT_MAX;
            int car_reach = INT_MAX;
            for (bk = 0; bk < wall_cnt; bk++)
            {
                int d = manhattan_cell_dist(bomb, walls[bk]);
                if (d < minw) minw = d;
            }
            /* 用真实BFS距离(小车到炸弹4邻域可达格)做排序键, 墙多时比曼哈顿更准;
               仅用于炸弹评估顺序, 不参与下界剪枝, 故不影响正确性/步数. */
            for (bk = 0; bk < 4; bk++)
            {
                int nr = (int)bomb.row + ddr[bk];
                int nc = (int)bomb.col + ddc[bk];
                if (nr < 0 || nr >= MAP_ROWS || nc < 0 || nc >= MAP_COLS)
                    continue;
                {
                    int di = car_dist[nr * MAP_COLS + nc];
                    if (di >= 0 && di < car_reach) car_reach = di;
                }
            }
            if (car_reach == INT_MAX)
                car_reach = manhattan_cell_dist(state->car_state, bomb) + MAP_ROWS + MAP_COLS;
            border[bo] = bo;
            border_lb[bo] = car_reach + (minw == INT_MAX ? 0 : minw);
        }
        for (bo = 0; bo + 1 < state->bombs_cnt; bo++)
        {
            int m = bo;
            for (bc = bo + 1; bc < state->bombs_cnt; bc++)
                if (border_lb[border[bc]] < border_lb[border[m]]) m = bc;
            if (m != bo) { int t = border[bo]; border[bo] = border[m]; border[m] = t; }
        }
    }

    for (b = 0; b < state->bombs_cnt; b++)
    {
        Position primary_bomb = state->bombs_state[border[b]];
        Position support_all[MAX_BOMBS];
        int support_all_cnt = build_support_bombs_except_primary(state, primary_bomb, support_all);
        int w;

				/* 按 炸弹->墙 距离升序评估,使下界剪枝尽早命中。
					 只改评估顺序,不改变最终选出的最优动作。 */
				{
						int a, c;
						for (a = 0; a + 1 < wall_cnt; a++)
						{
								int m = a;
								for (c = a + 1; c < wall_cnt; c++)
								{
										if (manhattan_cell_dist(primary_bomb, walls[c]) <
												manhattan_cell_dist(primary_bomb, walls[m]))
												m = c;
								}
								if (m != a)
								{
										Position tmp = walls[a];
										walls[a] = walls[m];
										walls[m] = tmp;
								}
						}
				}

        for (w = 0; w < wall_cnt; w++)
        {
            path_plan_result plan;
            int steps;
            int used_support = 0;
            Position support_pos = {255, 255, 0};
            round_action_t candidate;
            int lb = manhattan_cell_dist(state->car_state, primary_bomb) +
                     manhattan_cell_dist(primary_bomb, walls[w]);

            if (has_best && lb >= best_steps)
                break; /* 墙已按距离升序,后续 lb 只会更大,可直接跳出 */

            /* 先尝试不使用辅助炸弹。辅助炸弹在底层同时也是占据格，并非“约束更松”；
             * 直接只传 support_all 会把原本可走的格堵住，可能漏掉可行解。 */
            steps = plan_bomb_task_in_state(state,
                                            primary_bomb,
                                            walls[w],
                                            0,
                                            0,
                                            &plan,
                                            &used_support,
                                            &support_pos);
            if (steps <= 0 && support_all_cnt > 0)
            {
                steps = plan_bomb_task_in_state(state,
                                                primary_bomb,
                                                walls[w],
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
            candidate.pair_index = -1;
            candidate.plan = plan;
            candidate.has_support_bomb_pos = (uint8)(used_support ? 1 : 0);
            candidate.support_bomb_pos = support_pos;
            candidate.has_primary_bomb_pos = 1;
            candidate.primary_bomb_pos = primary_bomb;

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

    if (!has_best)
    {
        store_identify_bomb_cache(state_sig, 0, 0);
        return 0;
    }

    *out_action = best_action;
    store_identify_bomb_cache(state_sig, 1, &best_action);
    return 1;
}

/* 执行一次识别规划（可指定跳过一个箱子和一个目标点）。 */
static int execute_identify_plan_with_skip(const planning_state_t *initial_state,
                                           int skip_box_index,
                                           int skip_target_index,
                                           int max_len_bound,
                                           Position *out_path,
                                           int *out_len,
                                           planning_state_t *out_state,
                                           uint8 *seq_kind,
                                           uint8 *seq_id,
                                           int *seq_len)
{
    planning_state_t state;
    Position merged_path[MAX_CAR_PATH];
    int merged_len = 0;
    int seq_n = 0;
    uint8 box_done[MAX_BOXES];
    uint8 target_done[MAX_TARGETS];
    int pending_boxes;
    int pending_targets;
    int safety_round = 0;

    if (!initial_state || !out_path || !out_len || !out_state)
        return 0;
    if (max_len_bound <= 0)
        max_len_bound = INT_MAX;

    state = *initial_state;
    memset(box_done, 0, sizeof(box_done));
    memset(target_done, 0, sizeof(target_done));

    pending_boxes = state.boxes_cnt;
    pending_targets = state.targets_cnt;

    if (skip_box_index >= 0 && skip_box_index < state.boxes_cnt)
    {
        box_done[skip_box_index] = 1;
        pending_boxes--;
    }
    if (skip_target_index >= 0 && skip_target_index < state.targets_cnt)
    {
        target_done[skip_target_index] = 1;
        pending_targets--;
    }

    while ((pending_boxes > 0 || pending_targets > 0) && safety_round < 512)
    {
        int i;
        int dist[MAP_ROWS * MAP_COLS];
        int prev[MAP_ROWS * MAP_COLS];
        int found_identify = 0;
        int need_unlock = 0;
        int has_bomb_action = 0;
        round_action_t bomb_action;
        int best_steps = INT_MAX;
        int best_kind = -1; /* 0:box, 1:target */
        int best_index = -1;
        Position best_path[MAP_ROWS * MAP_COLS];
        int best_len = 0;
        Position best_stand = {0, 0, 0};
        Position best_object = {0, 0, 0};

        if (merged_len >= max_len_bound)
            return 0;
        if (!build_car_distance_prev_map(&state, state.car_state, dist, prev))
            break;

        for (i = 0; i < state.boxes_cnt; i++)
        {
            Position stand;
            int steps;

            if (box_done[i])
                continue;
            if (!pick_best_adjacent_stand_by_dist(&state,
                                                  state.boxes_state[i],
                                                  dist,
                                                  &stand,
                                                  &steps))
                continue;
            if (!found_identify || steps < best_steps)
            {
                found_identify = 1;
                best_steps = steps;
                best_kind = 0;
                best_index = i;
                best_len = 0;
                best_stand = stand;
                best_object = state.boxes_state[i];
            }
        }

        for (i = 0; i < state.targets_cnt; i++)
        {
            Position stand;
            int steps;

            if (target_done[i])
                continue;
            if (!pick_best_adjacent_stand_by_dist(&state,
                                                  state.targets_state[i],
                                                  dist,
                                                  &stand,
                                                  &steps))
                continue;
            if (!found_identify || steps < best_steps)
            {
                found_identify = 1;
                best_steps = steps;
                best_kind = 1;
                best_index = i;
                best_len = 0;
                best_stand = stand;
                best_object = state.targets_state[i];
            }
        }

        if (found_identify)
        {
            best_len = rebuild_car_path_from_prev(state.car_state,
                                                  best_stand,
                                                  prev,
                                                  best_path,
                                                  MAP_ROWS * MAP_COLS);
            if (best_len <= 0)
            {
                /* 兜底：极端情况下退回旧实现，保证功能稳定。 */
                best_len = build_best_adjacent_identify_path(&state,
                                                             best_object,
                                                             best_path,
                                                             MAP_ROWS * MAP_COLS,
                                                             &best_stand);
            }
            if (best_len <= 0)
            {
                found_identify = 0;
            }
            else
            {
                best_steps = best_len - 1;
            }
        }

        /* 与推箱模式对齐：若检测到未来推箱关键瓶颈，则提前把炸弹开路纳入候选。 */
        need_unlock = identify_need_proactive_unlock(&state,
                                                     box_done,
                                                     state.boxes_cnt,
                                                     target_done,
                                                     state.targets_cnt);
        if (state.bombs_cnt > 0 && (need_unlock || !found_identify))
        {
            int should_plan_bomb = 1;
            if (found_identify)
            {
                int bomb_lb = estimate_bomb_action_lb_by_dist(&state, dist);
                if (bomb_lb != INT_MAX && best_steps <= bomb_lb)
                    should_plan_bomb = 0;
            }
            if (should_plan_bomb)
            {
                has_bomb_action = pick_best_unlock_bomb_action_for_identify(&state,
                                                                             box_done,
                                                                             state.boxes_cnt,
                                                                             target_done,
                                                                             state.targets_cnt,
                                                                             &bomb_action);
            }
        }

        /* 识别动作与炸弹动作同优先级：都可执行时按步数最短先做。 */
        if (found_identify && best_len > 0 &&
            (!has_bomb_action || best_steps <= bomb_action.steps))
        {
            uint8 id_mark = identify_marker_by_relative(best_stand, best_object);
            /* 朝向由 Control 的识别分段状态机按物体相对位置处理；规划层只标记识别端点。 */
            best_path[best_len - 1].id = merge_marker(best_path[best_len - 1].id, id_mark);
            append_segment_path(merged_path, &merged_len, best_path, best_len);
            state.car_state = best_path[best_len - 1];

            if (best_kind == 0 && best_index >= 0 && best_index < state.boxes_cnt && !box_done[best_index])
            {
                box_done[best_index] = 1;
                pending_boxes--;
            }
            else if (best_kind == 1 && best_index >= 0 && best_index < state.targets_cnt && !target_done[best_index])
            {
                target_done[best_index] = 1;
                pending_targets--;
            }

            if (seq_kind && seq_id && seq_n < (MAX_BOXES + MAX_TARGETS))
            {
                seq_kind[seq_n] = (uint8)best_kind;
                seq_id[seq_n] = best_object.id;
                seq_n++;
            }

            if (merged_len >= max_len_bound)
                return 0;
            safety_round++;
            continue;
        }

        if (has_bomb_action)
        {
            apply_bomb_action_result(&state, &bomb_action, merged_path, &merged_len);
            if (merged_len >= max_len_bound)
                return 0;
            safety_round++;
            continue;
        }

        break;
    }

    if (pending_boxes > 0 || pending_targets > 0)
        return 0;

    *out_len = merged_len;
    memcpy(out_path, merged_path, (size_t)merged_len * sizeof(Position));
    *out_state = state;
    if (seq_len)
        *seq_len = seq_n;
    return 1;
}

/* 识别模式收尾约束：
 * 终点不能与任何目标点重合；若重合则按来路退回一格。 */
static void adjust_identify_end_avoid_target_overlap(planning_state_t *state,
                                                     Position *path,
                                                     int *path_len)
{
    Position last;
    Position prev;

    if (!state || !path || !path_len || *path_len <= 0)
        return;

    last = path[*path_len - 1];
    if (find_position_index(state->targets_state, state->targets_cnt, last) < 0)
        return;

    if (*path_len >= 2 && *path_len < MAX_CAR_PATH)
    {
        prev = path[*path_len - 2];
        prev.id = 0;
        path[*path_len] = prev;
        (*path_len)++;
        state->car_state = prev;
    }
}

/* 模式3：自动识别路径规划
 * 目标：
 * 1) 结束后不返场；
 * 2) 若开始有 n 对箱子/目标，则仅识别 n-1 个箱子和 n-1 个目标；
 * 3) 在所有“跳过1箱子+1目标”的组合里，选择总路径点最短的方案。 */
/* 构建纯识别路径。识别阶段不能提前按 ID 推箱：相机地图中的 ID 初始为 0，
 * 必须等 Control 完成实物识别后再进入 Mode2。 */
static int build_identify_walk_plan(const planning_state_t *start_state,
                                    Position *out_path, int *out_len,
                                    planning_state_t *out_state,
                                    uint8 *out_seq_kind, uint8 *out_seq_id,
                                    int *out_seq_len)
{
    typedef struct
    {
        uint8 b_skip;
        uint8 t_skip;
        int score;
    } identify_skip_combo_t;

    planning_state_t init_state;
    planning_state_t best_state;
    int best_len = INT_MAX;
    int has_best = 0;
    identify_skip_combo_t combos[MAX_BOXES * MAX_TARGETS];
    int combo_cnt = 0;
    uint8 tmp_seq_kind[MAX_BOXES + MAX_TARGETS];
    uint8 tmp_seq_id[MAX_BOXES + MAX_TARGETS];
    int tmp_seq_len = 0;
    uint8 best_seq_kind[MAX_BOXES + MAX_TARGETS];
    uint8 best_seq_id[MAX_BOXES + MAX_TARGETS];
    int best_seq_len = 0;
    int dist[MAP_ROWS * MAP_COLS];
    int prev_dummy[MAP_ROWS * MAP_COLS];
    int box_score[MAX_BOXES];
    int target_score[MAX_TARGETS];
    int b_skip, t_skip;
    int i;

    if (!start_state || !out_path || !out_len || !out_state)
        return 0;
    init_state = *start_state;
    if (init_state.boxes_cnt <= 0 || init_state.targets_cnt <= 0)
        return 0;

    for (i = 0; i < MAX_BOXES; i++)
        box_score[i] = 10000;
    for (i = 0; i < MAX_TARGETS; i++)
        target_score[i] = 10000;

    if (build_car_distance_prev_map(&init_state, init_state.car_state, dist, prev_dummy))
    {
        for (i = 0; i < init_state.boxes_cnt; i++)
        {
            Position stand;
            int steps;
            if (pick_best_adjacent_stand_by_dist(&init_state, init_state.boxes_state[i], dist, &stand, &steps))
                box_score[i] = steps;
        }
        for (i = 0; i < init_state.targets_cnt; i++)
        {
            Position stand;
            int steps;
            if (pick_best_adjacent_stand_by_dist(&init_state, init_state.targets_state[i], dist, &stand, &steps))
                target_score[i] = steps;
        }
    }
    else
    {
        for (i = 0; i < init_state.boxes_cnt; i++)
            box_score[i] = manhattan_cell_dist(init_state.car_state, init_state.boxes_state[i]);
        for (i = 0; i < init_state.targets_cnt; i++)
            target_score[i] = manhattan_cell_dist(init_state.car_state, init_state.targets_state[i]);
    }

    for (b_skip = 0; b_skip < init_state.boxes_cnt; b_skip++)
    {
        for (t_skip = 0; t_skip < init_state.targets_cnt; t_skip++)
        {
            if (combo_cnt >= (MAX_BOXES * MAX_TARGETS))
                break;
            combos[combo_cnt].b_skip = (uint8)b_skip;
            combos[combo_cnt].t_skip = (uint8)t_skip;
            combos[combo_cnt].score = box_score[b_skip] + target_score[t_skip];
            combo_cnt++;
        }
    }

    for (i = 0; i < combo_cnt; i++)
    {
        int j;
        int best = i;
        for (j = i + 1; j < combo_cnt; j++)
        {
            if (combos[j].score > combos[best].score)
                best = j;
        }
        if (best != i)
        {
            identify_skip_combo_t tmp = combos[i];
            combos[i] = combos[best];
            combos[best] = tmp;
        }
    }

    for (i = 0; i < combo_cnt; i++)
    {
        Position tmp_path[MAX_CAR_PATH];
        planning_state_t tmp_state;
        int tmp_len = 0;

        if (!execute_identify_plan_with_skip(&init_state,
                                             (int)combos[i].b_skip,
                                             (int)combos[i].t_skip,
                                             has_best ? best_len : INT_MAX,
                                             tmp_path,
                                             &tmp_len,
                                             &tmp_state,
                                             tmp_seq_kind,
                                             tmp_seq_id,
                                             &tmp_seq_len))
        {
            continue;
        }

        if (!has_best || tmp_len < best_len)
        {
            has_best = 1;
            best_len = tmp_len;
            best_state = tmp_state;
            memcpy(out_path, tmp_path, (size_t)tmp_len * sizeof(Position));
            memcpy(best_seq_kind, tmp_seq_kind, sizeof(best_seq_kind));
            memcpy(best_seq_id, tmp_seq_id, sizeof(best_seq_id));
            best_seq_len = tmp_seq_len;
        }
    }

    if (!has_best)
    {
        Position tmp_path[MAX_CAR_PATH];
        planning_state_t tmp_state;
        int tmp_len = 0;
        if (execute_identify_plan_with_skip(&init_state, -1, -1,
                                            INT_MAX,
                                            tmp_path, &tmp_len, &tmp_state,
                                            tmp_seq_kind, tmp_seq_id, &tmp_seq_len))
        {
            has_best = 1;
            best_len = tmp_len;
            best_state = tmp_state;
            memcpy(out_path, tmp_path, (size_t)tmp_len * sizeof(Position));
            memcpy(best_seq_kind, tmp_seq_kind, sizeof(best_seq_kind));
            memcpy(best_seq_id, tmp_seq_id, sizeof(best_seq_id));
            best_seq_len = tmp_seq_len;
        }
    }

    if (!has_best)
        return 0;

    adjust_identify_end_avoid_target_overlap(&best_state, out_path, &best_len);

    *out_state = best_state;
    *out_len = best_len;
    if (out_seq_len)
        *out_seq_len = best_seq_len;
    if (out_seq_kind && out_seq_id)
    {
        int si;
        for (si = 0; si < best_seq_len && si < (MAX_BOXES + MAX_TARGETS); si++)
        {
            out_seq_kind[si] = best_seq_kind[si];
            out_seq_id[si] = best_seq_id[si];
        }
    }

    return 1;
}

/* 新版曾尝试在识别途中按 ID 预推箱子并一次性完成推箱、返场。
 * 本项目的真实相机地图在识别前把所有 ID 置 0，控制层也要求识别和推箱分阶段执行，
 * 因而该实验实现不接入 Plan_path_Identify；若以后新增“地图预先提供可信 ID”
 * 的独立模式，应通过新入口实现，不能改变现有接口契约。 */

/* 与 Control 的两阶段状态机适配：这里只生成识别路线；实时识别并补齐 ID 后，
 * Control 会重新定位，再调用 Mode2 完成按 ID 配对推箱。 */
static void plan_mode_identify_auto(void)
{
    planning_state_t init_state;
    planning_state_t end_state;
    Position identify_path[MAX_CAR_PATH];
    uint8 seq_kind[MAX_BOXES + MAX_TARGETS];
    uint8 seq_id[MAX_BOXES + MAX_TARGETS];
    int identify_len = 0;
    int seq_len = 0;
    int i;

    reset_planning_caches();
    load_state_from_globals(&init_state);
    g_identify_seq_len = 0;
    memset(g_identify_seq_kind, 0, sizeof(g_identify_seq_kind));
    memset(g_identify_seq_id, 0, sizeof(g_identify_seq_id));

    if (init_state.boxes_cnt <= 0 || init_state.targets_cnt <= 0 ||
        !build_identify_walk_plan(&init_state,
                                  identify_path,
                                  &identify_len,
                                  &end_state,
                                  seq_kind,
                                  seq_id,
                                  &seq_len))
    {
        Car_path_count = 0;
        return;
    }

    if (seq_len > (MAX_BOXES + MAX_TARGETS))
        seq_len = MAX_BOXES + MAX_TARGETS;
    for (i = 0; i < seq_len; i++)
    {
        g_identify_seq_kind[i] = seq_kind[i];
        g_identify_seq_id[i] = seq_id[i];
    }
    g_identify_seq_len = seq_len;

    save_state_to_globals(&end_state);
    save_path_to_globals(identify_path, identify_len);
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
        int oi;
        int box_eval_cnt;
        int box_order[MAX_BOXES];
        int box_lb[MAX_BOXES];
        int box_score[MAX_BOXES];
        int best_valid = 0;
        int best_steps = 0;
        int best_box_index = -1;
        path_plan_result best_plan;

        box_eval_cnt = state.boxes_cnt;
        if (box_eval_cnt > MAX_BOXES)
            box_eval_cnt = MAX_BOXES;
        for (b = 0; b < box_eval_cnt; b++)
        {
            int t;
            int min_target_lb = INT_MAX;
            box_order[b] = b;
            for (t = 0; t < state.targets_cnt; t++)
            {
                int d = manhattan_cell_dist(state.boxes_state[b], state.targets_state[t]);
                if (d < min_target_lb)
                    min_target_lb = d;
            }
            if (min_target_lb == INT_MAX)
                min_target_lb = 0;
            box_lb[b] = min_target_lb;
            box_score[b] = min_target_lb + manhattan_cell_dist(state.car_state, state.boxes_state[b]);
        }

        for (oi = 0; oi < box_eval_cnt; oi++)
        {
            int j;
            int best = oi;
            for (j = oi + 1; j < box_eval_cnt; j++)
            {
                if (box_score[j] < box_score[best])
                    best = j;
            }
            if (best != oi)
            {
                int tmp_order = box_order[oi];
                int tmp_lb = box_lb[oi];
                int tmp_score = box_score[oi];
                box_order[oi] = box_order[best];
                box_lb[oi] = box_lb[best];
                box_score[oi] = box_score[best];
                box_order[best] = tmp_order;
                box_lb[best] = tmp_lb;
                box_score[best] = tmp_score;
            }
        }

        for (oi = 0; oi < box_eval_cnt; oi++)
        {
            path_plan_result plan;
            b = box_order[oi];
            if (best_valid && box_lb[oi] >= best_steps)
                continue;

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

    if (!append_return_to_depot(&state, merged_path, &merged_len))
    {
        Car_path_count = 0U;
        return;
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

    if (!append_return_to_depot(&state, merged_path, &merged_len))
    {
        Car_path_count = 0U;
        return;
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

void Plan_path_Identify(void)
{
    Position init_obs[MAX_OBSTACLES];
    int init_cnt = (int)Obstacles_count;
    if (init_cnt > MAX_OBSTACLES) init_cnt = MAX_OBSTACLES;
    for (int i = 0; i < init_cnt; i++) init_obs[i] = obstacles[i];

    plan_mode_identify_auto();

    g_blown_count = 0;
    for (int i = 0; i < init_cnt; i++)
    {
        int found = 0;
        for (size_t j = 0; j < Obstacles_count; j++)
            if (obstacles[j].row == init_obs[i].row && obstacles[j].col == init_obs[i].col) { found = 1; break; }
        if (!found && g_blown_count < MAX_OBSTACLES) g_blown_cell[g_blown_count++] = init_obs[i];
    }
}
