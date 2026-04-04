#include "Game_logic.h"
#include <string.h>

#define LOOKAHEAD_FAIL_PENALTY 100000

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

static int pos_equal(Position a, Position b)
{
    return (a.row == b.row) && (a.col == b.col);
}

static void remove_position_at(Position *arr, int *cnt, int index)
{
    int i;

    if (arr == 0 || cnt == 0 || *cnt <= 0)
    {
        return;
    }
    if (index < 0 || index >= *cnt)
    {
        return;
    }

    for (i = index; i < (*cnt - 1); i++)
    {
        arr[i] = arr[i + 1];
    }

    arr[*cnt - 1].row = 0;
    arr[*cnt - 1].col = 0;
    (*cnt)--;
}

static int find_position_index(const Position *arr, int cnt, Position target)
{
    int i;

    for (i = 0; i < cnt; i++)
    {
        if (pos_equal(arr[i], target))
        {
            return i;
        }
    }
    return -1;
}

static int find_position_index_by_id(const Position *arr, int cnt, int id)
{
    int i;

    for (i = 0; i < cnt; i++)
    {
        if (arr[i].id == id)
        {
            return i;
        }
    }
    return -1;
}

void append_segment_path(Position *merged_path, int *merged_len,
                         const Position *segment_path, int segment_len)
{
    int start_index = 0;
    int copy_len;
    int available;

    if (merged_path == 0 || merged_len == 0 || segment_path == 0 || segment_len <= 0)
    {
        return;
    }
    if (*merged_len < 0)
    {
        *merged_len = 0;
    }
    if (*merged_len >= MAX_CAR_PATH)
    {
        return;
    }

    if (*merged_len > 0 && pos_equal(merged_path[*merged_len - 1], segment_path[0]))
    {
        start_index = 1;
    }

    copy_len = segment_len - start_index;
    if (copy_len <= 0)
    {
        return;
    }

    available = MAX_CAR_PATH - *merged_len;
    if (copy_len > available)
    {
        copy_len = available;
    }
    if (copy_len <= 0)
    {
        return;
    }

    memcpy(&merged_path[*merged_len],
           &segment_path[start_index],
           (size_t)copy_len * sizeof(Position));
    *merged_len += copy_len;
}

void apply_round_result(const path_plan_result *plan,
                        int selected_box_index,
                        Position *local_obstacles, int *local_obstacles_cnt,
                        Position *local_bombs, int *local_bombs_cnt,
                        Position *local_boxes, int *local_boxes_cnt,
                        Position *local_targets, int *local_targets_cnt,
                        Position *io_car)
{
    int target_index;

    if (plan == 0 || io_car == 0 || plan->total_steps <= 0)
    {
        return;
    }

    if (plan->used_bomb)
    {
        if (plan->bomb_index >= 0 && plan->bomb_index < *local_bombs_cnt)
        {
            remove_position_at(local_bombs, local_bombs_cnt, plan->bomb_index);
        }

        if (plan->bomb_target.row >= 0 && plan->bomb_target.col >= 0)
        {
            simulate_bomb_explosion(local_obstacles, local_obstacles_cnt, plan->bomb_target);
        }
    }

    remove_position_at(local_boxes, local_boxes_cnt, selected_box_index);

    target_index = find_position_index(local_targets, *local_targets_cnt, plan->box_target);
    if (target_index >= 0)
    {
        remove_position_at(local_targets, local_targets_cnt, target_index);
    }
    else if (*local_targets_cnt > 0)
    {
        remove_position_at(local_targets, local_targets_cnt, 0);
    }

    *io_car = plan->car_path[plan->total_steps - 1];
}

/*
 * 为指定箱子构造候选目标集合。
 * mode1: 不区分 id，目标集合为全部剩余目标。
 * mode2: 只保留与 box.id 相同的目标。
 */
static int prepare_targets_for_box(const Position *targets_arr, int targets_cnt,
                                   Position box,
                                   int require_same_id,
                                   Position *out_targets,
                                   int *out_targets_cnt)
{
    if (targets_arr == 0 || out_targets == 0 || out_targets_cnt == 0 || targets_cnt <= 0)
    {
        return 0;
    }

    if (!require_same_id)
    {
        memcpy(out_targets, targets_arr, (size_t)targets_cnt * sizeof(Position));
        *out_targets_cnt = targets_cnt;
        return 1;
    }
    else
    {
        int idx = find_position_index_by_id(targets_arr, targets_cnt, box.id);
        if (idx < 0)
        {
            *out_targets_cnt = 0;
            return 0;
        }
        out_targets[0] = targets_arr[idx];
        *out_targets_cnt = 1;
        return 1;
    }
}

static int plan_single_box_in_state(const planning_state_t *state,
                                    int box_index,
                                    int require_same_id,
                                    path_plan_result *out_plan)
{
    Position candidate_targets[MAX_TARGETS];
    int candidate_targets_cnt = 0;
    int steps;

    if (state == 0 || out_plan == 0)
    {
        return -1;
    }
    if (box_index < 0 || box_index >= state->boxes_cnt)
    {
        return -1;
    }

    if (!prepare_targets_for_box(state->targets_state, state->targets_cnt,
                                 state->boxes_state[box_index],
                                 require_same_id,
                                 candidate_targets, &candidate_targets_cnt))
    {
        return -1;
    }

    steps = integrated_path_output(MAP_ROWS, MAP_COLS,
                                   state->obstacles_state, state->obstacles_cnt,
                                   state->bombs_state, state->bombs_cnt,
                                   state->boxes_state, state->boxes_cnt,
                                   candidate_targets, candidate_targets_cnt,
                                   box_index,
                                   state->car_state,
                                   out_plan);
    return steps;
}

/*
 * 评估当前状态下“下一轮”最短可行步数（用于两轮前瞻融合）。
 */
static int estimate_next_round_best_steps(const planning_state_t *state,
                                          int require_same_id)
{
    int i;
    int best_steps = -1;

    for (i = 0; i < state->boxes_cnt; i++)
    {
        path_plan_result plan;
        int steps = plan_single_box_in_state(state, i, require_same_id, &plan);
        if (steps > 0 && (best_steps < 0 || steps < best_steps))
        {
            best_steps = steps;
        }
    }
    return best_steps;
}

/*
 * 融合策略：
 * 不只看“当前这一个箱子的最短步数”，而是评估“当前 + 下一轮最优”的总成本。
 * 这样会优先选择可与后续任务共享路段、减少来回折返的首个箱子。
 */
static int pick_best_round_plan(const planning_state_t *state,
                                int require_same_id,
                                int *out_box_index,
                                path_plan_result *out_plan)
{
    int i;
    int best_score = -1;
    int best_steps = -1;
    int best_idx = -1;
    path_plan_result best_plan;

    for (i = 0; i < state->boxes_cnt; i++)
    {
        path_plan_result first_plan;
        int first_steps = plan_single_box_in_state(state, i, require_same_id, &first_plan);
        int score;

        if (first_steps <= 0)
        {
            continue;
        }

        score = first_steps;
        if (state->boxes_cnt > 1)
        {
            planning_state_t sim = *state;
            int second_steps;

            apply_round_result(&first_plan,
                               i,
                               sim.obstacles_state, &sim.obstacles_cnt,
                               sim.bombs_state, &sim.bombs_cnt,
                               sim.boxes_state, &sim.boxes_cnt,
                               sim.targets_state, &sim.targets_cnt,
                               &sim.car_state);

            second_steps = estimate_next_round_best_steps(&sim, require_same_id);
            if (second_steps > 0)
            {
                score += second_steps;
            }
            else
            {
                /* 惩罚会让选择器更倾向于“第一步后仍容易继续推进”的方案。 */
                score += LOOKAHEAD_FAIL_PENALTY;
            }
        }

        if (best_score < 0 || score < best_score || (score == best_score && first_steps < best_steps))
        {
            best_score = score;
            best_steps = first_steps;
            best_idx = i;
            best_plan = first_plan;
        }
    }

    if (best_idx < 0)
    {
        return 0;
    }

    *out_box_index = best_idx;
    *out_plan = best_plan;
    return 1;
}

static void plan_path_with_policy(int require_same_id)
{
    planning_state_t state;
    Position merged_path[MAX_CAR_PATH];
    int merged_len = 0;

    state.obstacles_cnt = (int)Obstacles_count;
    state.bombs_cnt = (int)Bombs_count;
    state.boxes_cnt = (int)Boxes_count;
    state.targets_cnt = (int)Targets_count;
    state.car_state = car;

    memcpy(state.obstacles_state, obstacles, (size_t)state.obstacles_cnt * sizeof(Position));
    memcpy(state.bombs_state, bombs, (size_t)state.bombs_cnt * sizeof(Position));
    memcpy(state.boxes_state, boxes, (size_t)state.boxes_cnt * sizeof(Position));
    memcpy(state.targets_state, targets, (size_t)state.targets_cnt * sizeof(Position));
    memset(merged_path, 0, sizeof(merged_path));

    while (state.boxes_cnt > 0)
    {
        int selected_box_index = -1;
        path_plan_result selected_plan;

        if (!pick_best_round_plan(&state, require_same_id, &selected_box_index, &selected_plan))
        {
            break;
        }

        append_segment_path(merged_path, &merged_len, selected_plan.car_path, selected_plan.total_steps);

        apply_round_result(&selected_plan,
                           selected_box_index,
                           state.obstacles_state, &state.obstacles_cnt,
                           state.bombs_state, &state.bombs_cnt,
                           state.boxes_state, &state.boxes_cnt,
                           state.targets_state, &state.targets_cnt,
                           &state.car_state);
    }

    memset(car_path, 0, sizeof(car_path));
    if (merged_len > 0)
    {
        memcpy(car_path, merged_path, (size_t)merged_len * sizeof(Position));
    }
    Car_path_count = (size_t)merged_len;
}

void Plan_path_Mode1(void)
{
    plan_path_with_policy(0);
}

void Plan_path_Mode2(void)
{
    plan_path_with_policy(1);
}
