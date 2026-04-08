#include "Game_logic.h"
#include <string.h>

/*
 * Game_logic.c
 * 涓婂眰璋冨害绛栫暐锛堥噸鏋勭増锛夛細
 * 1) 寮€濮嬪厛鍋氣€滃彲杈炬€у綊鍥犫€濓細
 *    - 鑻ュ叏鍥?绂佺敤鐐稿脊)鍙洿鎺ㄥ埌閰嶅鐩爣锛歂ORMAL
 *    - 鑻ヤ粎鍥犲叾浠栫瀛?鐐稿脊闃绘尅锛堢Щ闄ゅ畠浠悗鍙洿鎺級锛歀ATE
 *    - 鑻ョЩ闄ゅ叾浠栫瀛?鐐稿脊鍚庝粛涓嶅彲鐩存帹锛歄BSTACLE
 * 2) 鍏堝鐞?OBSTACLE锛? *    - 鍙墽琛屸€滃紑璺偢寮逛换鍔♀€濓紝浼樺厛淇濊瘉鎵€鏈夌瀛愰兘鍏峰鍙揪鎬с€? * 3) 鍐嶅鐞嗘帹绠卞瓙锛? *    - 姣忚疆閫夋渶鐭彲琛岀瀛愪换鍔★紱
 *    - 鍒濆 LATE 鐨勭瀛愭渶鍚庡啀鎺ㄣ€? *
 * 璇存槑锛? * - 鐐稿脊浠诲姟鏀寔鈥滆緟鍔╃偢寮光€濆紑璺紝鍙鐞嗙偢寮逛緷璧栫偢寮圭殑閾惧紡鍦烘櫙銆? * - 搴曞眰璺緞鎼滅储鍏ㄩ儴澶嶇敤 Algorithm.integrated_path_output銆? */

#define ACTION_NONE 0
#define ACTION_BOMB 1
#define ACTION_BOX 2

#define PAIR_STATUS_NORMAL   0
#define PAIR_STATUS_LATE     1
#define PAIR_STATUS_OBSTACLE 2
#define MAX_UNLOCK_CHAIN_DEPTH 3
#define MAX_UNLOCK_WALL_CANDIDATES 16
#define MAX_BOMB_ACTION_CANDIDATES 24
#define MAX_UNLOCK_DFS_BRANCH 8

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
    int box_index; /* ACTION_BOX 浣跨敤 */
    int pair_index; /* 瀵瑰簲 pair锛孉CTION_BOMB/ACTION_BOX 閮藉彲鐢?*/
    path_plan_result plan;

    /* 鑻ヤ换鍔″唴浣跨敤浜嗚緟鍔╃偢寮癸紝杩欓噷璁板綍鐪熷疄鍧愭爣銆?*/
    uint8 has_support_bomb_pos;
    Position support_bomb_pos;

    /* 鐐稿脊浠诲姟涓荤偢寮癸紙琚帹鍔ㄥ苟鏈€缁堢垎鐐哥殑鐐稿脊锛夈€?*/
    uint8 has_primary_bomb_pos;
    Position primary_bomb_pos;
} round_action_t;

typedef struct
{
    int valid;
    int chain_len;
    int total_steps;
    round_action_t actions[MAX_UNLOCK_CHAIN_DEPTH];
} unlock_chain_plan_t;

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
        return 1;
    }
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

/* 判断箱子当前是否存在“无需炸弹即可立即执行的一步推箱动作”。 */
static int box_has_immediate_push_move(const planning_state_t *state, Position box_pos)
{
    const int dr[4] = {-1, 1, 0, 0};
    const int dc[4] = {0, 0, -1, 1};
    int k;

    if (!state)
        return 0;

    for (k = 0; k < 4; k++)
    {
        Position front = {(uint8)((int)box_pos.row + dr[k]), (uint8)((int)box_pos.col + dc[k]), 0};
        Position back = {(uint8)((int)box_pos.row - dr[k]), (uint8)((int)box_pos.col - dc[k]), 0};

        if (!is_valid_cell(front) || !is_valid_cell(back))
            continue;
        if (state_has_obstacle_at(state, front) || state_has_obstacle_at(state, back))
            continue;
        if (find_position_index(state->bombs_state, state->bombs_cnt, front) >= 0 ||
            find_position_index(state->bombs_state, state->bombs_cnt, back) >= 0)
            continue;
        if (find_position_index(state->boxes_state, state->boxes_cnt, front) >= 0 ||
            find_position_index(state->boxes_state, state->boxes_cnt, back) >= 0)
            continue;
        return 1;
    }
    return 0;
}

static int pair_is_obstacle_in_state(const planning_state_t *state,
                                     const pair_task_t *pair,
                                     int require_same_id)
{
    int box_index;
    uint8 status;

    if (!state || !pair || !pair->valid || pair->target_done)
        return 0;
    if (!state_has_target(state, pair->target_ref))
        return 0;

    box_index = find_box_index_by_id(state->boxes_state, state->boxes_cnt, pair->box_id_ref);
    if (box_index < 0)
        return 0;
    if (require_same_id && !box_can_match_target(state->boxes_state[box_index], pair->target_ref, require_same_id))
        return 0;

    status = classify_pair_status(state, box_index, pair->target_ref);
    return (status == PAIR_STATUS_OBSTACLE) ? 1 : 0;
}

/* 判断该 pair 在当前状态下是否已经有可执行的“推箱到配对目标”任务（允许用炸弹）。 */
static int pair_has_box_action_now(const planning_state_t *state,
                                   const pair_task_t *pair,
                                   int require_same_id)
{
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
    return (steps > 0 && same_cell(plan.box_target, pair->target_ref)) ? 1 : 0;
}

/* 判断该 pair 是否“当前不可执行”，即需要先开路。 */
static int pair_needs_unlock_in_state(const planning_state_t *state,
                                      const pair_task_t *pair,
                                      int require_same_id)
{
    return pair_has_box_action_now(state, pair, require_same_id) ? 0 : 1;
}

static void build_critical_owner_for_state(const planning_state_t *state,
                                           int require_same_id,
                                           const pair_task_table_t *pairs,
                                           int critical_owner[MAX_BOMBS])
{
    int p, b;

    for (b = 0; b < MAX_BOMBS; b++)
        critical_owner[b] = -1;
    if (!state || !pairs)
        return;

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
            Position support_all[MAX_BOMBS];
            int support_all_cnt;
            path_plan_result plan;
            int steps;
            int used_support = 0;
            Position support_pos = {255, 255, 0};

            if (!infer_wall_for_pair_with_bomb(state, pair, require_same_id, primary_bomb, &wall))
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
                support_all_cnt = build_support_bombs_except_primary(state, primary_bomb, support_all);
                steps = plan_bomb_task_in_state(state,
                                                primary_bomb,
                                                wall,
                                                support_all,
                                                support_all_cnt,
                                                &plan,
                                                &used_support,
                                                &support_pos);
            }
            if (steps > 0)
            {
                feasible_cnt++;
                unique_bomb_index = b;
            }
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

static int collect_bomb_actions_for_pair(const planning_state_t *state,
                                         int require_same_id,
                                         const pair_task_table_t *pairs,
                                         int pair_index,
                                         const int critical_owner[MAX_BOMBS],
                                         round_action_t *out_actions,
                                         int max_actions)
{
    int b;
    int out_cnt = 0;
    const pair_task_t *pair;

    if (!state || !pairs || !out_actions || max_actions <= 0)
        return 0;
    if (pair_index < 0 || pair_index >= pairs->count)
        return 0;

    pair = &pairs->item[pair_index];
    if (!pair_needs_unlock_in_state(state, pair, require_same_id))
        return 0;

    for (b = 0; b < state->bombs_cnt && out_cnt < max_actions; b++)
    {
        Position primary_bomb = state->bombs_state[b];
        Position walls[MAX_UNLOCK_WALL_CANDIDATES];
        int wall_cnt;
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

        for (w = 0; w < wall_cnt && out_cnt < max_actions; w++)
        {
            path_plan_result plan;
            int steps;
            int used_support = 0;
            Position support_pos = {255, 255, 0};
            round_action_t candidate;
            Position wall = walls[w];

            steps = plan_bomb_task_in_state(state,
                                            primary_bomb,
                                            wall,
                                            0, 0,
                                            &plan,
                                            &used_support,
                                            &support_pos);
            if (steps <= 0)
            {
                support_all_cnt = build_support_bombs_except_primary(state, primary_bomb, support_all);
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

            out_actions[out_cnt++] = candidate;
        }
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

static void update_best_unlock_chain(unlock_chain_plan_t *best,
                                     const round_action_t *stack,
                                     int stack_len,
                                     int total_steps)
{
    if (!best || !stack || stack_len <= 0 || stack_len > MAX_UNLOCK_CHAIN_DEPTH)
        return;

    if (!best->valid ||
        stack_len < best->chain_len ||
        (stack_len == best->chain_len && total_steps < best->total_steps))
    {
        best->valid = 1;
        best->chain_len = stack_len;
        best->total_steps = total_steps;
        memcpy(best->actions, stack, (size_t)stack_len * sizeof(round_action_t));
    }
}

static void dfs_unlock_chain_for_pair(const planning_state_t *state,
                                      int require_same_id,
                                      const pair_task_table_t *pairs,
                                      int pair_index,
                                      int depth_left,
                                      round_action_t *stack,
                                      int stack_len,
                                      int steps_sum,
                                      unlock_chain_plan_t *best)
{
    int critical_owner[MAX_BOMBS];
    round_action_t actions[MAX_BOMB_ACTION_CANDIDATES];
    int action_cnt;
    int i;
    int branch_limit;

    if (!state || !pairs || !stack || !best)
        return;
    if (pair_index < 0 || pair_index >= pairs->count)
        return;

    if (pair_has_box_action_now(state, &pairs->item[pair_index], require_same_id))
    {
        update_best_unlock_chain(best, stack, stack_len, steps_sum);
        return;
    }
    if (depth_left <= 0)
        return;
    if (best->valid && stack_len >= best->chain_len)
        return;

    build_critical_owner_for_state(state, require_same_id, pairs, critical_owner);
    action_cnt = collect_bomb_actions_for_pair(state,
                                               require_same_id,
                                               pairs,
                                               pair_index,
                                               critical_owner,
                                               actions,
                                               MAX_BOMB_ACTION_CANDIDATES);

    branch_limit = action_cnt;
    if (branch_limit > MAX_UNLOCK_DFS_BRANCH)
    {
        int a, b;
        for (a = 0; a < MAX_UNLOCK_DFS_BRANCH; a++)
        {
            int best_idx = a;
            for (b = a + 1; b < action_cnt; b++)
            {
                if (actions[b].steps < actions[best_idx].steps)
                    best_idx = b;
            }
            if (best_idx != a)
            {
                round_action_t tmp = actions[a];
                actions[a] = actions[best_idx];
                actions[best_idx] = tmp;
            }
        }
        branch_limit = MAX_UNLOCK_DFS_BRANCH;
    }

    for (i = 0; i < branch_limit; i++)
    {
        planning_state_t shadow_state = *state;
        int next_steps = steps_sum + actions[i].steps;

        if (best->valid && (stack_len + 1) == best->chain_len && next_steps >= best->total_steps)
            continue;

        stack[stack_len] = actions[i];
        apply_bomb_action_result(&shadow_state, &actions[i], 0, 0);
        dfs_unlock_chain_for_pair(&shadow_state,
                                  require_same_id,
                                  pairs,
                                  pair_index,
                                  depth_left - 1,
                                  stack,
                                  stack_len + 1,
                                  next_steps,
                                  best);
    }
}

static int find_best_unlock_chain_for_pair(const planning_state_t *state,
                                           int require_same_id,
                                           const pair_task_table_t *pairs,
                                           int pair_index,
                                           unlock_chain_plan_t *out_chain)
{
    unlock_chain_plan_t best;
    round_action_t stack[MAX_UNLOCK_CHAIN_DEPTH];
    int depth;

    if (!state || !pairs || !out_chain)
        return 0;

    memset(stack, 0, sizeof(stack));

    for (depth = 1; depth <= MAX_UNLOCK_CHAIN_DEPTH; depth++)
    {
        memset(&best, 0, sizeof(best));
        dfs_unlock_chain_for_pair(state,
                                  require_same_id,
                                  pairs,
                                  pair_index,
                                  depth,
                                  stack,
                                  0,
                                  0,
                                  &best);
        if (best.valid && best.chain_len > 0)
        {
            *out_chain = best;
            return 1;
        }
    }

    return 0;
}

/* 选择“最短可行炸弹链”的第一步（支持连锁开路）。 */
static int pick_best_unlock_bomb_action_for_obstacles(const planning_state_t *state,
                                                      int require_same_id,
                                                      const pair_task_table_t *pairs,
                                                      round_action_t *out_action)
{
    int p;
    int has_best = 0;
    int best_pair_priority = -1;
    int best_move_unlock = -1;
    int best_chain_len = 0;
    int best_total_steps = 0;
    int best_first_steps = 0;
    round_action_t best_action;

    if (!state || !pairs || !out_action)
        return 0;

    memset(&best_action, 0, sizeof(best_action));

    for (p = 0; p < pairs->count; p++)
    {
        unlock_chain_plan_t chain;
        const pair_task_t *pair = &pairs->item[p];
        round_action_t first;
        int pair_priority = 0;
        int box_index;
        int move_unlock = 0;

        if (!pair_needs_unlock_in_state(state, pair, require_same_id))
            continue;

        box_index = find_box_index_by_id(state->boxes_state, state->boxes_cnt, pair->box_id_ref);
        if (box_index >= 0 &&
            !box_has_immediate_push_move(state, state->boxes_state[box_index]))
        {
            /* 优先解“当前一步都推不动”的箱子（典型为推位被墙/炸弹卡住）。 */
            pair_priority = 1;
        }

        if (!find_best_unlock_chain_for_pair(state, require_same_id, pairs, p, &chain))
            continue;
        if (chain.chain_len <= 0)
            continue;

        first = chain.actions[0];
        if (box_index >= 0)
        {
            planning_state_t shadow_state = *state;
            int new_box_index;
            apply_bomb_action_result(&shadow_state, &first, 0, 0);
            new_box_index = find_box_index_by_id(shadow_state.boxes_state,
                                                 shadow_state.boxes_cnt,
                                                 pair->box_id_ref);
            if (new_box_index >= 0 &&
                box_has_immediate_push_move(&shadow_state, shadow_state.boxes_state[new_box_index]))
            {
                move_unlock = 1;
            }
        }

        if (!has_best ||
            pair_priority > best_pair_priority ||
            (pair_priority == best_pair_priority && move_unlock > best_move_unlock) ||
            (pair_priority == best_pair_priority && move_unlock == best_move_unlock &&
             chain.chain_len < best_chain_len) ||
            (pair_priority == best_pair_priority && move_unlock == best_move_unlock &&
             chain.chain_len == best_chain_len &&
             chain.total_steps < best_total_steps) ||
            (pair_priority == best_pair_priority && move_unlock == best_move_unlock &&
             chain.chain_len == best_chain_len &&
             chain.total_steps == best_total_steps &&
             first.steps < best_first_steps) ||
            (pair_priority == best_pair_priority && move_unlock == best_move_unlock &&
             chain.chain_len == best_chain_len &&
             chain.total_steps == best_total_steps &&
             first.steps == best_first_steps &&
             pair->box_id_ref < pairs->item[best_action.pair_index].box_id_ref))
        {
            has_best = 1;
            best_pair_priority = pair_priority;
            best_move_unlock = move_unlock;
            best_chain_len = chain.chain_len;
            best_total_steps = chain.total_steps;
            best_first_steps = first.steps;
            best_action = first;
        }
    }

    if (!has_best)
        return 0;

    *out_action = best_action;
    return 1;
}

/* 閫夋嫨鏈€鐭瀛愪换鍔★紙鍙€夛細璺宠繃 LATE pair锛夈€?*/
static int pick_best_box_action(const planning_state_t *state,
                                int require_same_id,
                                const pair_task_table_t *pairs,
                                int skip_late_pairs,
                                round_action_t *out_action)
{
    int p;
    Position all_bombs[MAX_BOMBS];
    int all_bombs_cnt;
    round_action_t best;

    if (!state || !pairs || !out_action)
        return 0;

    memset(&best, 0, sizeof(best));
    all_bombs_cnt = state->bombs_cnt;
    memcpy(all_bombs, state->bombs_state, (size_t)all_bombs_cnt * sizeof(Position));

    for (p = 0; p < pairs->count; p++)
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

        one_target[0] = pair->target_ref;
        steps = plan_box_with_candidates(state,
                                         box_index,
                                         all_bombs, all_bombs_cnt,
                                         one_target, 1,
                                         state->car_state,
                                         &plan);
        if (steps <= 0)
            continue;

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

        if (!best.valid ||
            candidate.steps < best.steps ||
            (candidate.steps == best.steps && pair->box_id_ref < pairs->item[best.pair_index].box_id_ref))
        {
            best = candidate;
        }
    }

    if (!best.valid)
        return 0;

    *out_action = best;
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

static void apply_box_action_result(planning_state_t *state,
                                    pair_task_table_t *pairs,
                                    const round_action_t *action,
                                    Position *merged_path,
                                    int *merged_len)
{
    Position explode_pos;
    int box_remove_index;
    int target_remove_index;

    if (!state || !action || !merged_path || !merged_len)
        return;
    if (!action->valid || action->action_type != ACTION_BOX)
        return;
    if (action->box_index < 0 || action->box_index >= state->boxes_cnt)
        return;

    append_segment_path(merged_path, merged_len, action->plan.car_path, action->plan.total_steps);

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

/* 妯″紡1锛氫笉鍖哄垎 ID銆傛瘡杞€夋渶鐭彲琛岀瀛愪换鍔★紙浠诲姟鍐呴儴鍏佽鐢ㄧ偢寮规嵎寰勶級銆?*/
static void plan_mode1_simple(void)
{
    planning_state_t state;
    Position merged_path[MAX_CAR_PATH];
    int merged_len = 0;
    int safety_round = 0;

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
 * 妯″紡2锛圛D 閰嶅锛夋柊绛栫暐锛? * 1) 鍏堝垎绫?pair锛歂ORMAL / LATE / OBSTACLE锛? * 2) 浼樺厛鎵ц OBSTACLE 鐨勫紑璺偢寮逛换鍔★紱
 * 3) 鍐嶆帹绠卞瓙锛氭瘡杞渶鐭紝涓?LATE 鍦ㄥ叾瀹冩湭瀹屾垚 pair 涔嬪悗澶勭悊銆? */
static void plan_mode2_pair_first(void)
{
    planning_state_t state;
    pair_task_table_t pairs;
    Position merged_path[MAX_CAR_PATH];
    int merged_len = 0;
    int safety_round = 0;

    load_state_from_globals(&state);
    build_pair_table(&state, 1, &pairs);

    while (state.boxes_cnt > 0 && state.targets_cnt > 0 && safety_round < 256)
    {
        round_action_t action;
        int non_late_unfinished;
        int picked = 0;

        refresh_pair_statuses(&state, 1, &pairs);

        if (pick_best_unlock_bomb_action_for_obstacles(&state, 1, &pairs, &action))
        {
            apply_bomb_action_result(&state, &action, merged_path, &merged_len);
            picked = 1;
        }

        if (!picked)
        {
            non_late_unfinished = count_unfinished_non_late_pairs(&pairs);
            if (pick_best_box_action(&state, 1, &pairs, (non_late_unfinished > 0) ? 1 : 0, &action))
            {
                apply_box_action_result(&state, &pairs, &action, merged_path, &merged_len);
                picked = 1;
            }
            else if (non_late_unfinished > 0 &&
                     pick_best_box_action(&state, 1, &pairs, 0, &action))
            {
                apply_box_action_result(&state, &pairs, &action, merged_path, &merged_len);
                picked = 1;
            }
            else
            {
                break;
            }
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

