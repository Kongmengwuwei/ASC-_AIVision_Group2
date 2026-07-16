#include "TopPlanner.h"
#include "TopGrid.h"
#include "TopPath.h"
#include "TopVerify.h"

#include <limits.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

#define TOP_INTERNAL_MAX_NODES 4096U
#define TOP_HASH_CAPACITY 8192U
#define TOP_MAX_SOLUTIONS 8U
#define TOP_PARENT_NONE UINT16_MAX
#define TOP_HEAP_NONE INT16_C(-1)
#define TOP_COST_INF UINT32_C(0x3fffffff)
#define TOP_INFERRED_ID 0xFEU

typedef struct
{
    top_wall_bits_t walls;
    uint8_t car;
    uint8_t heading;
    uint8_t box_pos[TOP_MAX_BOXES];
    uint8_t bomb_pos[TOP_MAX_BOMBS];
    top_object_mask_t target_active_mask;
} top_search_state_t;

typedef struct
{
    top_search_state_t state;
    uint32_t g_ms;
    uint32_t h_ms;
    uint16_t parent;
    uint8_t action_kind;
    uint8_t object_index;
    uint8_t direction;
    uint8_t exploded;
    uint8_t open;
    uint8_t closed;
} top_search_node_t;

typedef struct
{
    uint16_t node[TOP_INTERNAL_MAX_NODES];
    int16_t position[TOP_INTERNAL_MAX_NODES];
    uint16_t size;
} top_node_heap_t;

typedef struct
{
    uint8_t solution_count;
    uint8_t timed_out;
    uint8_t node_limit_hit;
    uint16_t solution_node[TOP_MAX_SOLUTIONS];
    uint32_t solution_cost[TOP_MAX_SOLUTIONS];
    uint16_t expanded;
    uint16_t generated;
    uint32_t start_ms;
} top_search_run_t;

typedef struct
{
    uint8_t valid;
    top_object_kind_t kind;
    uint8_t index;
    top_cell_t stand;
    top_heading_t heading;
    uint8_t far_mode;
    uint32_t cost_ms;
} top_identify_candidate_t;

#if defined(__ARMCOMPILER_VERSION) || defined(__ARMCC_VERSION)
#define TOP_WORKSPACE_STORAGE __attribute__((section(".bss.OCRAM_CACHE")))
#else
#define TOP_WORKSPACE_STORAGE
#endif

/* The RT1064 scatter file provides a dedicated 512 KiB cached OCRAM region.
 * Keeping the search arena there avoids exhausting the smaller DTCM data
 * region while leaving the public result/control objects under caller control. */
static TOP_WORKSPACE_STORAGE top_search_node_t s_nodes[TOP_INTERNAL_MAX_NODES];
static TOP_WORKSPACE_STORAGE uint16_t s_hash[TOP_HASH_CAPACITY];
static TOP_WORKSPACE_STORAGE top_node_heap_t s_heap;
static TOP_WORKSPACE_STORAGE top_grid_paths_t s_grid_paths;
static TOP_WORKSPACE_STORAGE top_cell_t s_walk_cells[TOP_CELL_COUNT];
static TOP_WORKSPACE_STORAGE uint16_t s_chain[TOP_INTERNAL_MAX_NODES];

static const int8_t s_dir_row[4] = {0, 1, 0, -1};
static const int8_t s_dir_col[4] = {1, 0, -1, 0};

static int top_any_active_bomb(const top_search_state_t *state,
                               const top_problem_t *problem);
static int top_box_reverse_reachable(const top_problem_t *problem,
                                     const top_search_state_t *state,
                                     uint8_t box_index);

static uint32_t top_now(const top_config_t *config)
{
    if (config != NULL && config->now_ms != NULL)
    {
        return config->now_ms(config->now_user);
    }
    return 0U;
}

static uint32_t top_elapsed(uint32_t start, const top_config_t *config)
{
    return top_now(config) - start;
}

void top_config_default(top_config_t *config)
{
    if (config == NULL)
    {
        return;
    }
    memset(config, 0, sizeof(*config));
    config->cell_size_mm = 200U;
    config->translation_speed_mmps = 500U;
    config->rotate_90_ms = 1000U;
    config->identify_near_ms = 200U;
    config->identify_far_ms = 200U;
    config->bomb_wait_ms = 500U;
    config->planning_budget_ms = 1000U;
    config->max_expansions = 4000U;
    config->max_nodes = TOP_INTERNAL_MAX_NODES;
    config->heuristic_weight_permille = 8000U;
    config->enable_diagonal = 1U;
    config->enable_bombs = 1U;
}

void top_problem_clear(top_problem_t *problem)
{
    uint8_t i;
    if (problem == NULL)
    {
        return;
    }
    memset(problem, 0, sizeof(*problem));
    problem->heading = TOP_HEADING_RIGHT;
    for (i = 0U; i < TOP_MAX_BOXES; ++i)
    {
        problem->boxes[i].cell.row = -1;
        problem->boxes[i].cell.col = -1;
        problem->boxes[i].id = TOP_ID_UNKNOWN;
    }
    for (i = 0U; i < TOP_MAX_TARGETS; ++i)
    {
        problem->targets[i].cell.row = -1;
        problem->targets[i].cell.col = -1;
        problem->targets[i].id = TOP_ID_UNKNOWN;
    }
    for (i = 0U; i < TOP_MAX_BOMBS; ++i)
    {
        problem->bombs[i].row = -1;
        problem->bombs[i].col = -1;
    }
}

int top_problem_set_wall(top_problem_t *problem, int row, int col, int blocked)
{
    uint8_t index;
    uint8_t word;
    uint8_t bit;
    if (problem == NULL || row < 0 || row >= (int)TOP_ROWS ||
        col < 0 || col >= (int)TOP_COLS)
    {
        return 0;
    }
    index = (uint8_t)(row * (int)TOP_COLS + col);
    word = (uint8_t)(index >> 5);
    bit = (uint8_t)(index & 31U);
    if (blocked)
    {
        problem->walls.word[word] |= UINT32_C(1) << bit;
    }
    else
    {
        problem->walls.word[word] &= ~(UINT32_C(1) << bit);
    }
    return 1;
}

int top_problem_has_wall(const top_problem_t *problem, int row, int col)
{
    uint8_t index;
    if (problem == NULL || row < 0 || row >= (int)TOP_ROWS ||
        col < 0 || col >= (int)TOP_COLS)
    {
        return 0;
    }
    index = (uint8_t)(row * (int)TOP_COLS + col);
    return (problem->walls.word[index >> 5] &
            (UINT32_C(1) << (index & 31U))) != 0U;
}

static int top_bits_test(const top_wall_bits_t *bits, uint8_t index)
{
    return bits != NULL && index < TOP_CELL_COUNT &&
           (bits->word[index >> 5] & (UINT32_C(1) << (index & 31U))) != 0U;
}

static void top_bits_clear(top_wall_bits_t *bits, uint8_t index)
{
    if (bits != NULL && index < TOP_CELL_COUNT)
    {
        bits->word[index >> 5] &= ~(UINT32_C(1) << (index & 31U));
    }
}

static int top_active_object_cell(const top_cell_t *cells, uint8_t count, top_cell_t cell)
{
    uint8_t i;
    for (i = 0U; i < count; ++i)
    {
        if (top_cell_valid(cells[i]) && top_cell_equal(cells[i], cell))
        {
            return 1;
        }
    }
    return 0;
}

top_status_t top_problem_validate(const top_problem_t *problem)
{
    top_cell_t boxes[TOP_MAX_BOXES];
    uint8_t active_boxes = 0U;
    uint8_t active_targets = 0U;
    uint8_t i;
    uint8_t j;

    if (problem == NULL || problem->box_count > TOP_MAX_BOXES ||
        problem->target_count > TOP_MAX_TARGETS ||
        problem->bomb_count > TOP_MAX_BOMBS ||
        problem->box_count != problem->target_count ||
        !top_cell_valid(problem->car) || problem->heading > TOP_HEADING_UP)
    {
        return TOP_STATUS_INVALID_INPUT;
    }
    if (top_problem_has_wall(problem, problem->car.row, problem->car.col))
    {
        return TOP_STATUS_INVALID_INPUT;
    }
    for (i = 0U; i < problem->box_count; ++i)
    {
        boxes[i] = problem->boxes[i].cell;
        if (!top_cell_valid(boxes[i]))
        {
            continue;
        }
        ++active_boxes;
        if (top_problem_has_wall(problem, boxes[i].row, boxes[i].col) ||
            top_cell_equal(problem->car, boxes[i]))
        {
            return TOP_STATUS_INVALID_INPUT;
        }
        for (j = 0U; j < i; ++j)
        {
            if (top_cell_valid(boxes[j]) && top_cell_equal(boxes[i], boxes[j]))
            {
                return TOP_STATUS_INVALID_INPUT;
            }
        }
    }
    for (i = 0U; i < problem->target_count; ++i)
    {
        if (!top_cell_valid(problem->targets[i].cell))
        {
            continue;
        }
        ++active_targets;
        if (top_problem_has_wall(problem,
                                 problem->targets[i].cell.row,
                                 problem->targets[i].cell.col))
        {
            return TOP_STATUS_INVALID_INPUT;
        }
        for (j = 0U; j < i; ++j)
        {
            if (top_cell_valid(problem->targets[j].cell) &&
                top_cell_equal(problem->targets[i].cell, problem->targets[j].cell))
            {
                return TOP_STATUS_INVALID_INPUT;
            }
        }
    }
    if (active_boxes != active_targets)
    {
        return TOP_STATUS_INVALID_INPUT;
    }
    for (i = 0U; i < problem->bomb_count; ++i)
    {
        if (!top_cell_valid(problem->bombs[i]))
        {
            continue;
        }
        if (top_problem_has_wall(problem, problem->bombs[i].row, problem->bombs[i].col) ||
            top_cell_equal(problem->car, problem->bombs[i]) ||
            top_active_object_cell(boxes, problem->box_count, problem->bombs[i]))
        {
            return TOP_STATUS_INVALID_INPUT;
        }
        for (j = 0U; j < i; ++j)
        {
            if (top_cell_valid(problem->bombs[j]) &&
                top_cell_equal(problem->bombs[i], problem->bombs[j]))
            {
                return TOP_STATUS_INVALID_INPUT;
            }
        }
    }
    return TOP_STATUS_OK;
}

static uint8_t top_active_box_count(const top_search_state_t *state,
                                    const top_problem_t *problem)
{
    uint8_t i;
    uint8_t count = 0U;
    for (i = 0U; i < problem->box_count; ++i)
    {
        if (state->box_pos[i] != TOP_INVALID_CELL)
        {
            ++count;
        }
    }
    return count;
}

static int top_ids_compatible(const top_problem_t *problem,
                              uint8_t box_index,
                              uint8_t target_index,
                              int require_known)
{
    if (problem->match_mode == TOP_MODE_FREE_MATCH)
    {
        return 1;
    }
    if (!problem->boxes[box_index].id_known ||
        !problem->targets[target_index].id_known)
    {
        return require_known ? 0 : 1;
    }
    return problem->boxes[box_index].id == problem->targets[target_index].id;
}

static int top_all_active_ids_known(const top_problem_t *problem)
{
    uint8_t i;
    if (problem->match_mode == TOP_MODE_FREE_MATCH)
    {
        return 1;
    }
    for (i = 0U; i < problem->box_count; ++i)
    {
        if (top_cell_valid(problem->boxes[i].cell) && !problem->boxes[i].id_known)
        {
            return 0;
        }
    }
    for (i = 0U; i < problem->target_count; ++i)
    {
        if (top_cell_valid(problem->targets[i].cell) && !problem->targets[i].id_known)
        {
            return 0;
        }
    }
    return 1;
}

static void top_infer_last_pair(top_problem_t *problem)
{
    uint8_t box_unknown = 0U;
    uint8_t target_unknown = 0U;
    uint8_t box_index = 0U;
    uint8_t target_index = 0U;
    uint8_t i;

    if (problem == NULL || problem->match_mode != TOP_MODE_ID_MATCH)
    {
        return;
    }
    for (i = 0U; i < problem->box_count; ++i)
    {
        if (top_cell_valid(problem->boxes[i].cell) && !problem->boxes[i].id_known)
        {
            ++box_unknown;
            box_index = i;
        }
    }
    for (i = 0U; i < problem->target_count; ++i)
    {
        if (top_cell_valid(problem->targets[i].cell) && !problem->targets[i].id_known)
        {
            ++target_unknown;
            target_index = i;
        }
    }
    if (box_unknown == 1U && target_unknown == 1U)
    {
        problem->boxes[box_index].id = TOP_INFERRED_ID;
        problem->targets[target_index].id = TOP_INFERRED_ID;
        problem->boxes[box_index].id_known = 1U;
        problem->targets[target_index].id_known = 1U;
    }
}

static void top_state_from_problem(const top_problem_t *problem,
                                   top_search_state_t *state)
{
    uint8_t i;
    memset(state, 0, sizeof(*state));
    state->walls = problem->walls;
    state->car = top_cell_index(problem->car);
    state->heading = (uint8_t)problem->heading;
    state->target_active_mask = 0U;
    for (i = 0U; i < TOP_MAX_BOXES; ++i)
    {
        state->box_pos[i] = TOP_INVALID_CELL;
    }
    for (i = 0U; i < TOP_MAX_BOMBS; ++i)
    {
        state->bomb_pos[i] = TOP_INVALID_CELL;
    }
    for (i = 0U; i < problem->box_count; ++i)
    {
        if (top_cell_valid(problem->boxes[i].cell))
        {
            state->box_pos[i] = top_cell_index(problem->boxes[i].cell);
        }
    }
    for (i = 0U; i < problem->target_count; ++i)
    {
        if (top_cell_valid(problem->targets[i].cell))
        {
            state->target_active_mask |= (top_object_mask_t)(UINT16_C(1) << i);
        }
    }
    for (i = 0U; i < problem->bomb_count; ++i)
    {
        if (top_cell_valid(problem->bombs[i]))
        {
            state->bomb_pos[i] = top_cell_index(problem->bombs[i]);
        }
    }
}

static uint32_t top_hash_state(const top_search_state_t *state,
                               const top_problem_t *problem)
{
    const uint8_t *bytes = (const uint8_t *)state;
    size_t length = sizeof(*state);
    uint32_t hash = UINT32_C(2166136261);
    size_t i;
    (void)problem;
    for (i = 0U; i < length; ++i)
    {
        hash ^= bytes[i];
        hash *= UINT32_C(16777619);
    }
    return hash;
}

static int top_state_equal(const top_search_state_t *a,
                           const top_search_state_t *b)
{
    return memcmp(a, b, sizeof(*a)) == 0;
}

static void top_state_cells(const top_search_state_t *state,
                            const top_problem_t *problem,
                            top_cell_t *boxes,
                            top_cell_t *bombs)
{
    uint8_t i;
    for (i = 0U; i < problem->box_count; ++i)
    {
        boxes[i] = state->box_pos[i] == TOP_INVALID_CELL ?
                   (top_cell_t){-1, -1} : top_index_cell(state->box_pos[i]);
    }
    for (i = 0U; i < problem->bomb_count; ++i)
    {
        bombs[i] = state->bomb_pos[i] == TOP_INVALID_CELL ?
                   (top_cell_t){-1, -1} : top_index_cell(state->bomb_pos[i]);
    }
}

static void top_state_blockers(const top_search_state_t *state,
                               const top_problem_t *problem,
                               top_cell_t *boxes,
                               top_cell_t *bombs,
                               top_grid_blockers_t *blockers)
{
    top_state_cells(state, problem, boxes, bombs);
    blockers->walls = &state->walls;
    blockers->boxes = boxes;
    blockers->box_count = problem->box_count;
    blockers->bombs = bombs;
    blockers->bomb_count = problem->bomb_count;
    blockers->ignored_box = -1;
    blockers->ignored_bomb = -1;
}

static uint32_t top_assignment_recursive(const top_problem_t *problem,
                                         const top_search_state_t *state,
                                         const uint8_t *box_indices,
                                         uint8_t box_count,
                                         uint8_t at,
                                         top_object_mask_t used_targets,
                                         uint32_t step_ms)
{
    uint8_t t;
    uint32_t best = TOP_COST_INF;
    uint8_t b;
    top_cell_t box_cell;

    if (at >= box_count)
    {
        return 0U;
    }
    b = box_indices[at];
    box_cell = top_index_cell(state->box_pos[b]);
    for (t = 0U; t < problem->target_count; ++t)
    {
        top_cell_t target_cell;
        uint32_t rest;
        uint32_t distance;
        int manhattan;
        if ((state->target_active_mask & (top_object_mask_t)(UINT16_C(1) << t)) == 0U ||
            (used_targets & (top_object_mask_t)(UINT16_C(1) << t)) != 0U ||
            !top_ids_compatible(problem, b, t, 1))
        {
            continue;
        }
        target_cell = problem->targets[t].cell;
        manhattan = abs((int)box_cell.row - (int)target_cell.row) +
                    abs((int)box_cell.col - (int)target_cell.col);
        distance = (uint32_t)manhattan * step_ms;
        if (at + 1U == box_count)
        {
            rest = 0U;
        }
        else
        {
            rest = top_assignment_recursive(problem,
                                            state,
                                            box_indices,
                                            box_count,
                                            (uint8_t)(at + 1U),
                                            (top_object_mask_t)(used_targets |
                                                (top_object_mask_t)(UINT16_C(1) << t)),
                                            step_ms);
        }
        if (rest < TOP_COST_INF && distance + rest < best)
        {
            best = distance + rest;
        }
    }
    return best;
}

static uint32_t top_heuristic(const top_problem_t *problem,
                              const top_search_state_t *state,
                              const top_config_t *config)
{
    uint8_t box_indices[TOP_MAX_BOXES];
    uint8_t box_count = 0U;
    uint8_t b;
    uint32_t step_ms;

    step_ms = (uint32_t)config->cell_size_mm * 1000U /
              config->translation_speed_mmps;
    for (b = 0U; b < problem->box_count; ++b)
    {
        if (state->box_pos[b] != TOP_INVALID_CELL)
        {
            box_indices[box_count++] = b;
        }
    }
    if (box_count == 0U)
    {
        return 0U;
    }
    {
        uint32_t estimate = top_assignment_recursive(problem,
                                                     state,
                                                     box_indices,
                                                     box_count,
                                                     0U,
                                                     0U,
                                                     step_ms);
        /* With bombs still available, a box that is unreachable on the current
         * wall layout is not a dead state.  Add a guidance penalty so explosions
         * that actually unlock a pair are explored before aimless bomb motion.
         * This deliberately makes the fast search non-admissible; bounded-time
         * approximate optimality is the requested operating mode. */
        if (estimate < TOP_COST_INF && top_any_active_bomb(state, problem))
        {
            for (b = 0U; b < problem->box_count; ++b)
            {
                if (state->box_pos[b] != TOP_INVALID_CELL &&
                    !top_box_reverse_reachable(problem, state, b))
                {
                    estimate += config->bomb_wait_ms + step_ms * 2U;
                }
            }
        }
        return estimate;
    }
}

static uint32_t top_node_priority(uint16_t node_index, const top_config_t *config)
{
    const top_search_node_t *node = &s_nodes[node_index];
    uint64_t weighted = (uint64_t)node->h_ms * config->heuristic_weight_permille;
    uint64_t value = node->g_ms + weighted / 1000U;
    return value >= TOP_COST_INF ? TOP_COST_INF : (uint32_t)value;
}

static int top_node_less(uint16_t a, uint16_t b, const top_config_t *config)
{
    uint32_t pa = top_node_priority(a, config);
    uint32_t pb = top_node_priority(b, config);
    if (pa != pb)
    {
        return pa < pb;
    }
    if (s_nodes[a].h_ms != s_nodes[b].h_ms)
    {
        return s_nodes[a].h_ms < s_nodes[b].h_ms;
    }
    if (s_nodes[a].g_ms != s_nodes[b].g_ms)
    {
        return s_nodes[a].g_ms < s_nodes[b].g_ms;
    }
    return a < b;
}

static void top_node_heap_swap(uint16_t a, uint16_t b)
{
    uint16_t na = s_heap.node[a];
    uint16_t nb = s_heap.node[b];
    s_heap.node[a] = nb;
    s_heap.node[b] = na;
    s_heap.position[na] = (int16_t)b;
    s_heap.position[nb] = (int16_t)a;
}

static void top_node_heap_up(uint16_t at, const top_config_t *config)
{
    while (at > 0U)
    {
        uint16_t parent = (uint16_t)((at - 1U) >> 1);
        if (!top_node_less(s_heap.node[at], s_heap.node[parent], config))
        {
            break;
        }
        top_node_heap_swap(at, parent);
        at = parent;
    }
}

static void top_node_heap_down(uint16_t at, const top_config_t *config)
{
    for (;;)
    {
        uint16_t left = (uint16_t)(at * 2U + 1U);
        uint16_t right = (uint16_t)(left + 1U);
        uint16_t best = at;
        if (left < s_heap.size &&
            top_node_less(s_heap.node[left], s_heap.node[best], config))
        {
            best = left;
        }
        if (right < s_heap.size &&
            top_node_less(s_heap.node[right], s_heap.node[best], config))
        {
            best = right;
        }
        if (best == at)
        {
            break;
        }
        top_node_heap_swap(at, best);
        at = best;
    }
}

static void top_node_heap_push_or_update(uint16_t node, const top_config_t *config)
{
    int16_t position = s_heap.position[node];
    if (position >= 0)
    {
        top_node_heap_up((uint16_t)position, config);
        top_node_heap_down((uint16_t)position, config);
        return;
    }
    if (s_heap.size >= TOP_INTERNAL_MAX_NODES)
    {
        return;
    }
    s_heap.node[s_heap.size] = node;
    s_heap.position[node] = (int16_t)s_heap.size;
    ++s_heap.size;
    top_node_heap_up((uint16_t)(s_heap.size - 1U), config);
}

static int top_node_heap_pop(const top_config_t *config)
{
    uint16_t result;
    if (s_heap.size == 0U)
    {
        return -1;
    }
    result = s_heap.node[0];
    s_heap.position[result] = TOP_HEAP_NONE;
    --s_heap.size;
    if (s_heap.size > 0U)
    {
        s_heap.node[0] = s_heap.node[s_heap.size];
        s_heap.position[s_heap.node[0]] = 0;
        top_node_heap_down(0U, config);
    }
    return result;
}

static int top_find_state(const top_search_state_t *state,
                          const top_problem_t *problem,
                          uint32_t hash,
                          uint16_t *slot_out)
{
    uint16_t probe;
    uint16_t slot = (uint16_t)(hash & (TOP_HASH_CAPACITY - 1U));
    for (probe = 0U; probe < TOP_HASH_CAPACITY; ++probe)
    {
        uint16_t stored = s_hash[slot];
        if (stored == 0U)
        {
            *slot_out = slot;
            return -1;
        }
        if (top_state_equal(&s_nodes[stored - 1U].state, state))
        {
            *slot_out = slot;
            return (int)(stored - 1U);
        }
        slot = (uint16_t)((slot + 1U) & (TOP_HASH_CAPACITY - 1U));
    }
    (void)problem;
    *slot_out = 0U;
    return -2;
}

static int top_state_cell_has_box(const top_search_state_t *state,
                                  const top_problem_t *problem,
                                  uint8_t cell,
                                  int ignored)
{
    uint8_t i;
    for (i = 0U; i < problem->box_count; ++i)
    {
        if ((int)i != ignored && state->box_pos[i] == cell)
        {
            return 1;
        }
    }
    return 0;
}

static int top_state_cell_has_bomb(const top_search_state_t *state,
                                   const top_problem_t *problem,
                                   uint8_t cell,
                                   int ignored)
{
    uint8_t i;
    for (i = 0U; i < problem->bomb_count; ++i)
    {
        if ((int)i != ignored && state->bomb_pos[i] == cell)
        {
            return 1;
        }
    }
    return 0;
}

static int top_any_active_bomb(const top_search_state_t *state,
                               const top_problem_t *problem)
{
    uint8_t i;
    for (i = 0U; i < problem->bomb_count; ++i)
    {
        if (state->bomb_pos[i] != TOP_INVALID_CELL)
        {
            return 1;
        }
    }
    return 0;
}

static int top_box_reverse_reachable(const top_problem_t *problem,
                                     const top_search_state_t *state,
                                     uint8_t box_index)
{
    uint8_t visited[TOP_CELL_COUNT];
    uint8_t queue[TOP_CELL_COUNT];
    uint16_t head = 0U;
    uint16_t tail = 0U;
    uint8_t t;

    if (state->box_pos[box_index] == TOP_INVALID_CELL)
    {
        return 1;
    }
    memset(visited, 0, sizeof(visited));
    for (t = 0U; t < problem->target_count; ++t)
    {
        if ((state->target_active_mask & (top_object_mask_t)(UINT16_C(1) << t)) != 0U &&
            top_ids_compatible(problem, box_index, t, 1))
        {
            uint8_t index = top_cell_index(problem->targets[t].cell);
            if (!visited[index])
            {
                visited[index] = 1U;
                queue[tail++] = index;
            }
        }
    }
    while (head < tail)
    {
        top_cell_t current = top_index_cell(queue[head++]);
        uint8_t d;
        for (d = 0U; d < 4U; ++d)
        {
            top_cell_t predecessor = {
                (int8_t)(current.row - s_dir_row[d]),
                (int8_t)(current.col - s_dir_col[d])};
            top_cell_t pusher = {
                (int8_t)(predecessor.row - s_dir_row[d]),
                (int8_t)(predecessor.col - s_dir_col[d])};
            uint8_t predecessor_index;
            if (!top_cell_valid(predecessor) || !top_cell_valid(pusher))
            {
                continue;
            }
            predecessor_index = top_cell_index(predecessor);
            if (visited[predecessor_index] ||
                top_bits_test(&state->walls, predecessor_index) ||
                top_bits_test(&state->walls, top_cell_index(pusher)))
            {
                continue;
            }
            visited[predecessor_index] = 1U;
            queue[tail++] = predecessor_index;
        }
    }
    return visited[state->box_pos[box_index]] != 0U;
}

static int top_box_on_compatible_target(const top_problem_t *problem,
                                        const top_search_state_t *state,
                                        uint8_t box_index,
                                        uint8_t cell,
                                        uint8_t *target_out)
{
    uint8_t t;
    for (t = 0U; t < problem->target_count; ++t)
    {
        if ((state->target_active_mask & (top_object_mask_t)(UINT16_C(1) << t)) != 0U &&
            top_cell_index(problem->targets[t].cell) == cell &&
            top_ids_compatible(problem, box_index, t, 1))
        {
            if (target_out != NULL)
            {
                *target_out = t;
            }
            return 1;
        }
    }
    return 0;
}

static int top_box_permanent_deadlock(const top_problem_t *problem,
                                      const top_search_state_t *state,
                                      uint8_t box_index)
{
    top_cell_t box;
    uint8_t t;
    int boundary_target = 0;
    int up;
    int down;
    int left;
    int right;

    if (state->box_pos[box_index] == TOP_INVALID_CELL)
    {
        return 0;
    }
    box = top_index_cell(state->box_pos[box_index]);
    if (box.row == 0 || box.row == (int8_t)(TOP_ROWS - 1U) ||
        box.col == 0 || box.col == (int8_t)(TOP_COLS - 1U))
    {
        for (t = 0U; t < problem->target_count; ++t)
        {
            top_cell_t target = problem->targets[t].cell;
            if ((state->target_active_mask & (top_object_mask_t)(UINT16_C(1) << t)) == 0U ||
                !top_ids_compatible(problem, box_index, t, 1))
            {
                continue;
            }
            if ((box.row == 0 && target.row == 0) ||
                (box.row == (int8_t)(TOP_ROWS - 1U) && target.row == box.row) ||
                (box.col == 0 && target.col == 0) ||
                (box.col == (int8_t)(TOP_COLS - 1U) && target.col == box.col))
            {
                boundary_target = 1;
                break;
            }
        }
        if (!boundary_target)
        {
            return 1;
        }
    }
    if (top_any_active_bomb(state, problem))
    {
        return 0;
    }
    up = box.row == 0 || top_bits_test(&state->walls, (uint8_t)(state->box_pos[box_index] - TOP_COLS));
    down = box.row == (int8_t)(TOP_ROWS - 1U) ||
           top_bits_test(&state->walls, (uint8_t)(state->box_pos[box_index] + TOP_COLS));
    left = box.col == 0 || top_bits_test(&state->walls, (uint8_t)(state->box_pos[box_index] - 1U));
    right = box.col == (int8_t)(TOP_COLS - 1U) ||
            top_bits_test(&state->walls, (uint8_t)(state->box_pos[box_index] + 1U));
    if ((up || down) && (left || right))
    {
        return 1;
    }
    return !top_box_reverse_reachable(problem, state, box_index);
}

static void top_explode(top_search_state_t *state, top_cell_t center)
{
    int dr;
    int dc;
    for (dr = -1; dr <= 1; ++dr)
    {
        for (dc = -1; dc <= 1; ++dc)
        {
            top_cell_t cell = {(int8_t)(center.row + dr),
                               (int8_t)(center.col + dc)};
            if (top_cell_valid(cell))
            {
                top_bits_clear(&state->walls, top_cell_index(cell));
            }
        }
    }
}

static int top_add_or_relax_node(const top_problem_t *problem,
                                 const top_config_t *config,
                                 top_search_run_t *run,
                                 const top_search_state_t *state,
                                 uint32_t g_ms,
                                 uint16_t parent,
                                 uint8_t action_kind,
                                 uint8_t object_index,
                                 uint8_t direction,
                                 uint8_t exploded)
{
    uint32_t hash = top_hash_state(state, problem);
    uint16_t slot;
    int existing = top_find_state(state, problem, hash, &slot);
    uint16_t index;
    uint32_t h_ms;

    if (existing == -2)
    {
        run->node_limit_hit = 1U;
        return 0;
    }
    h_ms = top_heuristic(problem, state, config);
    if (h_ms >= TOP_COST_INF)
    {
        return 0;
    }
    if (existing >= 0)
    {
        top_search_node_t *node = &s_nodes[existing];
        if (g_ms >= node->g_ms)
        {
            return 0;
        }
        node->g_ms = g_ms;
        node->h_ms = h_ms;
        node->parent = parent;
        node->action_kind = action_kind;
        node->object_index = object_index;
        node->direction = direction;
        node->exploded = exploded;
        node->closed = 0U;
        node->open = 1U;
        top_node_heap_push_or_update((uint16_t)existing, config);
        return 1;
    }
    if (run->generated >= config->max_nodes || run->generated >= TOP_INTERNAL_MAX_NODES)
    {
        run->node_limit_hit = 1U;
        return 0;
    }
    index = run->generated++;
    memset(&s_nodes[index], 0, sizeof(s_nodes[index]));
    s_nodes[index].state = *state;
    s_nodes[index].g_ms = g_ms;
    s_nodes[index].h_ms = h_ms;
    s_nodes[index].parent = parent;
    s_nodes[index].action_kind = action_kind;
    s_nodes[index].object_index = object_index;
    s_nodes[index].direction = direction;
    s_nodes[index].exploded = exploded;
    s_nodes[index].open = 1U;
    s_hash[slot] = (uint16_t)(index + 1U);
    s_heap.position[index] = TOP_HEAP_NONE;
    top_node_heap_push_or_update(index, config);
    return 1;
}

static uint32_t top_goal_return_cost(const top_problem_t *problem,
                                     const top_search_state_t *state,
                                     const top_config_t *config,
                                     top_cell_t *depot_out)
{
    top_cell_t boxes[TOP_MAX_BOXES];
    top_cell_t bombs[TOP_MAX_BOMBS];
    top_grid_blockers_t blockers;
    top_wall_bits_t no_walls;
    top_cell_t depots[2] = {{4, 0}, {5, 0}};
    top_cell_t start = top_index_cell(state->car);
    uint32_t best = TOP_COST_INF;
    uint8_t i;

    memset(&no_walls, 0, sizeof(no_walls));
    top_state_cells(state, problem, boxes, bombs);
    memset(boxes, 0xFF, sizeof(boxes));
    blockers.walls = &no_walls;
    blockers.boxes = boxes;
    blockers.box_count = problem->box_count;
    blockers.bombs = bombs;
    blockers.bomb_count = problem->bomb_count;
    blockers.ignored_box = -1;
    blockers.ignored_bomb = -1;
    if (!top_grid_shortest_paths(start, &blockers, config, &s_grid_paths))
    {
        return TOP_COST_INF;
    }
    for (i = 0U; i < 2U; ++i)
    {
        uint32_t cost = s_grid_paths.dist_ms[top_cell_index(depots[i])];
        if (cost < best)
        {
            best = cost;
            if (depot_out != NULL)
            {
                *depot_out = depots[i];
            }
        }
    }
    if (best >= TOP_COST_INF)
    {
        return TOP_COST_INF;
    }
    return best + top_rotation_ms((top_heading_t)state->heading,
                                  TOP_HEADING_RIGHT,
                                  config);
}

static void top_record_solution(top_search_run_t *run,
                                uint16_t node,
                                uint32_t cost)
{
    uint8_t i;
    uint8_t at;

    for (i = 0U; i < run->solution_count; ++i)
    {
        if (run->solution_node[i] == node)
        {
            if (cost < run->solution_cost[i])
            {
                run->solution_cost[i] = cost;
            }
            return;
        }
    }
    if (run->solution_count < TOP_MAX_SOLUTIONS)
    {
        at = run->solution_count++;
    }
    else
    {
        uint8_t worst = 0U;
        for (i = 1U; i < TOP_MAX_SOLUTIONS; ++i)
        {
            if (run->solution_cost[i] > run->solution_cost[worst])
            {
                worst = i;
            }
        }
        if (cost >= run->solution_cost[worst])
        {
            return;
        }
        at = worst;
    }
    run->solution_node[at] = node;
    run->solution_cost[at] = cost;
}

static void top_expand_node(const top_problem_t *problem,
                            const top_config_t *config,
                            top_search_run_t *run,
                            uint16_t node_index)
{
    const top_search_node_t *node = &s_nodes[node_index];
    top_cell_t boxes[TOP_MAX_BOXES];
    top_cell_t bombs[TOP_MAX_BOMBS];
    top_grid_blockers_t blockers;
    top_cell_t car = top_index_cell(node->state.car);
    uint32_t push_ms = (uint32_t)config->cell_size_mm * 1000U /
                       config->translation_speed_mmps;
    uint8_t i;
    uint8_t d;

    top_state_blockers(&node->state, problem, boxes, bombs, &blockers);
    if (!top_grid_shortest_paths(car, &blockers, config, &s_grid_paths))
    {
        return;
    }

    for (i = 0U; i < problem->box_count; ++i)
    {
        top_cell_t object;
        if (node->state.box_pos[i] == TOP_INVALID_CELL)
        {
            continue;
        }
        object = top_index_cell(node->state.box_pos[i]);
        for (d = 0U; d < 4U; ++d)
        {
            top_cell_t behind = {(int8_t)(object.row - s_dir_row[d]),
                                 (int8_t)(object.col - s_dir_col[d])};
            top_cell_t destination = {(int8_t)(object.row + s_dir_row[d]),
                                      (int8_t)(object.col + s_dir_col[d])};
            uint8_t destination_index;
            uint32_t walk_ms;
            top_search_state_t next;
            uint8_t target_index;

            if (!top_cell_valid(behind) || !top_cell_valid(destination))
            {
                continue;
            }
            destination_index = top_cell_index(destination);
            if (top_bits_test(&node->state.walls, destination_index) ||
                top_state_cell_has_box(&node->state, problem, destination_index, i) ||
                top_state_cell_has_bomb(&node->state, problem, destination_index, -1))
            {
                continue;
            }
            walk_ms = s_grid_paths.dist_ms[top_cell_index(behind)];
            if (walk_ms >= TOP_GRID_INF_MS)
            {
                continue;
            }
            next = node->state;
            next.car = node->state.box_pos[i];
            next.box_pos[i] = destination_index;
            if (top_box_on_compatible_target(problem,
                                             &next,
                                             i,
                                             destination_index,
                                             &target_index))
            {
                next.box_pos[i] = TOP_INVALID_CELL;
                next.target_active_mask &= (top_object_mask_t)~
                    (top_object_mask_t)(UINT16_C(1) << target_index);
            }
            else if (top_box_permanent_deadlock(problem, &next, i))
            {
                continue;
            }
            (void)top_add_or_relax_node(problem,
                                        config,
                                        run,
                                        &next,
                                        node->g_ms + walk_ms + push_ms,
                                        node_index,
                                        TOP_OBJECT_BOX,
                                        i,
                                        d,
                                        0U);
        }
    }

    if (!config->enable_bombs)
    {
        return;
    }
    for (i = 0U; i < problem->bomb_count; ++i)
    {
        top_cell_t object;
        if (node->state.bomb_pos[i] == TOP_INVALID_CELL)
        {
            continue;
        }
        object = top_index_cell(node->state.bomb_pos[i]);
        for (d = 0U; d < 4U; ++d)
        {
            top_cell_t behind = {(int8_t)(object.row - s_dir_row[d]),
                                 (int8_t)(object.col - s_dir_col[d])};
            top_cell_t destination = {(int8_t)(object.row + s_dir_row[d]),
                                      (int8_t)(object.col + s_dir_col[d])};
            uint8_t destination_index;
            uint32_t walk_ms;
            top_search_state_t next;
            uint8_t exploded;
            uint32_t extra_ms;

            if (!top_cell_valid(behind) || !top_cell_valid(destination))
            {
                continue;
            }
            destination_index = top_cell_index(destination);
            if (top_state_cell_has_box(&node->state, problem, destination_index, -1) ||
                top_state_cell_has_bomb(&node->state, problem, destination_index, i))
            {
                continue;
            }
            walk_ms = s_grid_paths.dist_ms[top_cell_index(behind)];
            if (walk_ms >= TOP_GRID_INF_MS)
            {
                continue;
            }
            next = node->state;
            next.car = node->state.bomb_pos[i];
            exploded = top_bits_test(&next.walls, destination_index) ? 1U : 0U;
            extra_ms = 0U;
            if (exploded)
            {
                next.bomb_pos[i] = TOP_INVALID_CELL;
                top_explode(&next, destination);
                extra_ms = config->bomb_wait_ms;
            }
            else
            {
                next.bomb_pos[i] = destination_index;
            }
            (void)top_add_or_relax_node(problem,
                                        config,
                                        run,
                                        &next,
                                        node->g_ms + walk_ms + push_ms + extra_ms,
                                        node_index,
                                        TOP_OBJECT_BOMB,
                                        i,
                                        d,
                                        exploded);
        }
    }
}

static int top_search(const top_problem_t *problem,
                      const top_config_t *config,
                      uint32_t start_ms,
                      top_search_run_t *run)
{
    top_search_state_t initial;
    uint16_t i;

    memset(run, 0, sizeof(*run));
    memset(s_nodes, 0, sizeof(s_nodes));
    memset(s_hash, 0, sizeof(s_hash));
    memset(&s_heap, 0, sizeof(s_heap));
    for (i = 0U; i < TOP_INTERNAL_MAX_NODES; ++i)
    {
        s_heap.position[i] = TOP_HEAP_NONE;
    }
    run->start_ms = start_ms;
    top_state_from_problem(problem, &initial);
    if (!top_add_or_relax_node(problem,
                               config,
                               run,
                               &initial,
                               0U,
                               TOP_PARENT_NONE,
                               TOP_OBJECT_NONE,
                               0U,
                               0U,
                               0U))
    {
        return 0;
    }

    while (s_heap.size > 0U)
    {
        int popped;
        top_search_node_t *node;
        uint8_t active_boxes;

        if (config->now_ms != NULL && config->planning_budget_ms > 0U &&
            top_elapsed(start_ms, config) >= config->planning_budget_ms)
        {
            run->timed_out = 1U;
            break;
        }
        if (config->max_expansions > 0U && run->expanded >= config->max_expansions)
        {
            run->node_limit_hit = 1U;
            break;
        }
        popped = top_node_heap_pop(config);
        if (popped < 0)
        {
            break;
        }
        node = &s_nodes[popped];
        if (node->closed)
        {
            continue;
        }
        node->open = 0U;
        node->closed = 1U;
        ++run->expanded;
        active_boxes = top_active_box_count(&node->state, problem);
        if (active_boxes == 0U)
        {
            uint32_t total = node->g_ms;
            uint32_t return_ms = top_goal_return_cost(problem,
                                                      &node->state,
                                                      config,
                                                      NULL);
            if (return_ms >= TOP_COST_INF)
            {
                continue;
            }
            total += return_ms;
            top_record_solution(run, (uint16_t)popped, total);
            continue;
        }
        top_expand_node(problem, config, run, (uint16_t)popped);
    }
    return run->solution_count > 0U;
}

static uint16_t top_best_solution(const top_search_run_t *run)
{
    uint8_t i;
    uint8_t best = 0U;
    for (i = 1U; i < run->solution_count; ++i)
    {
        if (run->solution_cost[i] < run->solution_cost[best])
        {
            best = i;
        }
    }
    return run->solution_node[best];
}

static void top_fill_end_state(top_end_state_t *out,
                               const top_problem_t *problem,
                               const top_search_state_t *state,
                               top_cell_t car,
                               top_heading_t heading)
{
    uint8_t i;
    memset(out, 0, sizeof(*out));
    out->car = car;
    out->heading = heading;
    out->walls = state->walls;
    for (i = 0U; i < TOP_MAX_BOXES; ++i)
    {
        out->box_cells[i].row = -1;
        out->box_cells[i].col = -1;
    }
    for (i = 0U; i < TOP_MAX_BOMBS; ++i)
    {
        out->bomb_cells[i].row = -1;
        out->bomb_cells[i].col = -1;
    }
    for (i = 0U; i < problem->box_count; ++i)
    {
        if (state->box_pos[i] != TOP_INVALID_CELL)
        {
            out->box_active_mask |= (top_object_mask_t)(UINT16_C(1) << i);
            out->box_cells[i] = top_index_cell(state->box_pos[i]);
        }
    }
    out->target_active_mask = state->target_active_mask;
    for (i = 0U; i < problem->bomb_count; ++i)
    {
        if (state->bomb_pos[i] != TOP_INVALID_CELL)
        {
            out->bomb_active_mask |= (uint16_t)(UINT16_C(1) << i);
            out->bomb_cells[i] = top_index_cell(state->bomb_pos[i]);
        }
    }
}

static int top_reconstruct_solution(const top_problem_t *problem,
                                    const top_config_t *config,
                                    const top_search_run_t *run,
                                    uint16_t solution_node,
                                    top_result_t *result)
{
    top_path_builder_t builder;
    uint16_t chain_count = 0U;
    uint16_t current = solution_node;
    uint16_t i;
    uint32_t push_ms = (uint32_t)config->cell_size_mm * 1000U /
                       config->translation_speed_mmps;
    top_search_state_t final_state;
    top_cell_t final_car;
    top_heading_t final_heading;

    memset(result, 0, sizeof(*result));
    while (chain_count < TOP_INTERNAL_MAX_NODES)
    {
        s_chain[chain_count++] = current;
        if (s_nodes[current].parent == TOP_PARENT_NONE)
        {
            break;
        }
        current = s_nodes[current].parent;
    }
    if (chain_count == 0U ||
        s_nodes[s_chain[chain_count - 1U]].parent != TOP_PARENT_NONE)
    {
        return 0;
    }
    top_path_builder_init(&builder,
                          result,
                          config,
                          top_index_cell(s_nodes[s_chain[chain_count - 1U]].state.car),
                          (top_heading_t)s_nodes[s_chain[chain_count - 1U]].state.heading);
    if (builder.failed)
    {
        return 0;
    }

    for (i = (uint16_t)(chain_count - 1U); i > 0U; --i)
    {
        const top_search_node_t *parent = &s_nodes[s_chain[i]];
        const top_search_node_t *child = &s_nodes[s_chain[i - 1U]];
        top_cell_t boxes[TOP_MAX_BOXES];
        top_cell_t bombs[TOP_MAX_BOMBS];
        top_grid_blockers_t blockers;
        top_cell_t object;
        top_cell_t behind;
        top_cell_t destination;
        uint16_t walk_count = 0U;
        top_event_mask_t events;

        top_state_blockers(&parent->state, problem, boxes, bombs, &blockers);
        if (child->action_kind == TOP_OBJECT_BOX)
        {
            object = top_index_cell(parent->state.box_pos[child->object_index]);
        }
        else if (child->action_kind == TOP_OBJECT_BOMB)
        {
            object = top_index_cell(parent->state.bomb_pos[child->object_index]);
        }
        else
        {
            return 0;
        }
        behind.row = (int8_t)(object.row - s_dir_row[child->direction]);
        behind.col = (int8_t)(object.col - s_dir_col[child->direction]);
        destination.row = (int8_t)(object.row + s_dir_row[child->direction]);
        destination.col = (int8_t)(object.col + s_dir_col[child->direction]);
        if (!top_grid_shortest_paths(top_index_cell(parent->state.car),
                                     &blockers,
                                     config,
                                     &s_grid_paths) ||
            !top_grid_reconstruct(top_index_cell(parent->state.car),
                                  behind,
                                  &s_grid_paths,
                                  s_walk_cells,
                                  TOP_CELL_COUNT,
                                  &walk_count) ||
            !top_path_append_walk(&builder,
                                  s_walk_cells,
                                  walk_count,
                                  &blockers,
                                  TOP_EVENT_MOVE_END))
        {
            return 0;
        }
        if (child->action_kind == TOP_OBJECT_BOX)
        {
            events = TOP_EVENT_PUSH_BOX_END;
            if (parent->state.box_pos[child->object_index] != TOP_INVALID_CELL &&
                child->state.box_pos[child->object_index] == TOP_INVALID_CELL)
            {
                events |= TOP_EVENT_BOX_DELIVERED | TOP_EVENT_MAP_CHANGED;
            }
            if (!top_path_append_push(&builder,
                                      TOP_ACTION_PUSH_BOX,
                                      TOP_OBJECT_BOX,
                                      child->object_index,
                                      object,
                                      destination,
                                      events,
                                      push_ms))
            {
                return 0;
            }
        }
        else
        {
            events = TOP_EVENT_PUSH_BOMB_END;
            if (child->exploded)
            {
                events |= TOP_EVENT_BOMB_EXPLODED | TOP_EVENT_MAP_CHANGED;
            }
            if (!top_path_append_push(&builder,
                                      TOP_ACTION_PUSH_BOMB,
                                      TOP_OBJECT_BOMB,
                                      child->object_index,
                                      object,
                                      destination,
                                      events,
                                      push_ms))
            {
                return 0;
            }
            if (child->exploded &&
                !top_path_append_wait(&builder,
                                      config->bomb_wait_ms,
                                      TOP_EVENT_BOMB_EXPLODED | TOP_EVENT_MAP_CHANGED,
                                      destination))
            {
                return 0;
            }
        }
    }

    final_state = s_nodes[solution_node].state;
    final_car = top_index_cell(final_state.car);
    final_heading = (top_heading_t)final_state.heading;
    {
        top_cell_t boxes[TOP_MAX_BOXES];
        top_cell_t bombs[TOP_MAX_BOMBS];
        top_grid_blockers_t blockers;
        top_wall_bits_t no_walls;
        top_cell_t depots[2] = {{4, 0}, {5, 0}};
        top_cell_t depot;
        uint32_t best = TOP_COST_INF;
        uint16_t walk_count = 0U;
        uint8_t d;

        memset(&no_walls, 0, sizeof(no_walls));
        top_state_cells(&final_state, problem, boxes, bombs);
        memset(boxes, 0xFF, sizeof(boxes));
        blockers.walls = &no_walls;
        blockers.boxes = boxes;
        blockers.box_count = problem->box_count;
        blockers.bombs = bombs;
        blockers.bomb_count = problem->bomb_count;
        blockers.ignored_box = -1;
        blockers.ignored_bomb = -1;
        if (!top_grid_shortest_paths(final_car, &blockers, config, &s_grid_paths))
        {
            return 0;
        }
        depot = depots[0];
        for (d = 0U; d < 2U; ++d)
        {
            uint32_t cost = s_grid_paths.dist_ms[top_cell_index(depots[d])];
            if (cost < best)
            {
                best = cost;
                depot = depots[d];
            }
        }
        if (best >= TOP_COST_INF ||
            !top_grid_reconstruct(final_car,
                                  depot,
                                  &s_grid_paths,
                                  s_walk_cells,
                                  TOP_CELL_COUNT,
                                  &walk_count) ||
            !top_path_append_walk(&builder,
                                  s_walk_cells,
                                  walk_count,
                                  &blockers,
                                  TOP_EVENT_RETURNED) ||
            !top_path_append_rotate(&builder, TOP_HEADING_RIGHT) ||
            !top_path_finish(&builder, TOP_EVENT_RETURNED))
        {
            return 0;
        }
        final_state.walls = no_walls;
        final_car = depot;
        final_heading = TOP_HEADING_RIGHT;
    }
    if (builder.failed)
    {
        return 0;
    }
    top_fill_end_state(&result->end_state,
                       problem,
                       &final_state,
                       final_car,
                       final_heading);
    result->expanded_nodes = run->expanded;
    result->generated_nodes = run->generated;
    result->timed_out = run->timed_out;
    result->node_limit_hit = run->node_limit_hit;
    result->complete = 1U;
    result->needs_replan = 0U;
    result->status = TOP_STATUS_OK;
    return 1;
}

static int top_target_blocks_sight(const top_problem_t *problem,
                                   top_object_kind_t observed_kind,
                                   uint8_t observed_index,
                                   top_cell_t cell)
{
    uint8_t i;
    for (i = 0U; i < problem->target_count; ++i)
    {
        if (observed_kind == TOP_OBJECT_TARGET && i == observed_index)
        {
            continue;
        }
        if (top_cell_valid(problem->targets[i].cell) &&
            top_cell_equal(problem->targets[i].cell, cell))
        {
            return 1;
        }
    }
    return 0;
}

static int top_identify_sight_clear(const top_problem_t *problem,
                                    const top_grid_blockers_t *blockers,
                                    top_object_kind_t kind,
                                    uint8_t index,
                                    top_cell_t stand,
                                    uint8_t direction,
                                    uint8_t distance)
{
    uint8_t step;
    for (step = 1U; step < distance; ++step)
    {
        top_cell_t cell = {(int8_t)(stand.row + s_dir_row[direction] * (int8_t)step),
                           (int8_t)(stand.col + s_dir_col[direction] * (int8_t)step)};
        if (top_grid_cell_blocked(blockers, cell) ||
            top_target_blocks_sight(problem, kind, index, cell))
        {
            return 0;
        }
    }
    return 1;
}

static int top_find_identification(const top_problem_t *problem,
                                   const top_config_t *config,
                                   top_identify_candidate_t *candidate)
{
    top_search_state_t state;
    top_cell_t boxes[TOP_MAX_BOXES];
    top_cell_t bombs[TOP_MAX_BOMBS];
    top_grid_blockers_t blockers;
    top_cell_t car;
    uint8_t kind_value;

    memset(candidate, 0, sizeof(*candidate));
    candidate->cost_ms = TOP_COST_INF;
    top_state_from_problem(problem, &state);
    top_state_blockers(&state, problem, boxes, bombs, &blockers);
    car = problem->car;
    if (!top_grid_shortest_paths(car, &blockers, config, &s_grid_paths))
    {
        return 0;
    }

    for (kind_value = TOP_OBJECT_BOX; kind_value <= TOP_OBJECT_TARGET; ++kind_value)
    {
        top_object_kind_t kind = (top_object_kind_t)kind_value;
        uint8_t count = kind == TOP_OBJECT_BOX ? problem->box_count : problem->target_count;
        uint8_t i;
        for (i = 0U; i < count; ++i)
        {
            const top_labeled_object_t *object = kind == TOP_OBJECT_BOX ?
                                                  &problem->boxes[i] :
                                                  &problem->targets[i];
            uint8_t direction;
            if (!top_cell_valid(object->cell) || object->id_known)
            {
                continue;
            }
            for (direction = 0U; direction < 4U; ++direction)
            {
                uint8_t distance;
                for (distance = 1U; distance <= 3U; ++distance)
                {
                    top_cell_t stand = {
                        (int8_t)(object->cell.row - s_dir_row[direction] * (int8_t)distance),
                        (int8_t)(object->cell.col - s_dir_col[direction] * (int8_t)distance)};
                    uint32_t walk_ms;
                    uint32_t cost;
                    uint32_t identify_ms;
                    if (!top_cell_valid(stand) || top_grid_cell_blocked(&blockers, stand) ||
                        top_target_blocks_sight(problem, kind, i, stand) ||
                        !top_identify_sight_clear(problem,
                                                  &blockers,
                                                  kind,
                                                  i,
                                                  stand,
                                                  direction,
                                                  distance))
                    {
                        continue;
                    }
                    walk_ms = s_grid_paths.dist_ms[top_cell_index(stand)];
                    if (walk_ms >= TOP_GRID_INF_MS)
                    {
                        continue;
                    }
                    identify_ms = distance == 1U ? config->identify_near_ms :
                                                   config->identify_far_ms;
                    cost = walk_ms +
                           top_rotation_ms(problem->heading,
                                           (top_heading_t)direction,
                                           config) +
                           identify_ms;
                    if (!candidate->valid || cost < candidate->cost_ms ||
                        (cost == candidate->cost_ms && distance < (candidate->far_mode ? 2U : 1U)))
                    {
                        candidate->valid = 1U;
                        candidate->kind = kind;
                        candidate->index = i;
                        candidate->stand = stand;
                        candidate->heading = (top_heading_t)direction;
                        candidate->far_mode = distance > 1U ? 1U : 0U;
                        candidate->cost_ms = cost;
                    }
                }
            }
        }
    }
    return candidate->valid;
}

static int top_build_identification_result(const top_problem_t *problem,
                                           const top_config_t *config,
                                           const top_identify_candidate_t *candidate,
                                           top_result_t *result)
{
    top_search_state_t state;
    top_cell_t boxes[TOP_MAX_BOXES];
    top_cell_t bombs[TOP_MAX_BOMBS];
    top_grid_blockers_t blockers;
    top_path_builder_t builder;
    uint16_t walk_count = 0U;

    memset(result, 0, sizeof(*result));
    top_state_from_problem(problem, &state);
    top_state_blockers(&state, problem, boxes, bombs, &blockers);
    if (!top_grid_shortest_paths(problem->car, &blockers, config, &s_grid_paths) ||
        !top_grid_reconstruct(problem->car,
                              candidate->stand,
                              &s_grid_paths,
                              s_walk_cells,
                              TOP_CELL_COUNT,
                              &walk_count))
    {
        return 0;
    }
    top_path_builder_init(&builder, result, config, problem->car, problem->heading);
    if (!top_path_append_walk(&builder,
                              s_walk_cells,
                              walk_count,
                              &blockers,
                              TOP_EVENT_MOVE_END) ||
        !top_path_append_rotate(&builder, candidate->heading) ||
        !top_path_append_identify(&builder,
                                  candidate->kind,
                                  candidate->index,
                                  candidate->far_mode) ||
        !top_path_finish(&builder, TOP_EVENT_NONE))
    {
        return 0;
    }
    state.car = top_cell_index(candidate->stand);
    state.heading = (uint8_t)candidate->heading;
    top_fill_end_state(&result->end_state,
                       problem,
                       &state,
                       candidate->stand,
                       candidate->heading);
    result->status = TOP_STATUS_PARTIAL_REPLAN;
    result->needs_replan = 1U;
    result->requested_identify_kind = candidate->kind;
    result->requested_identify_index = candidate->index;
    return 1;
}

static top_status_t top_finalize_verified(const top_problem_t *problem,
                                          const top_config_t *config,
                                          uint32_t start_ms,
                                          top_result_t *result)
{
    top_verify_report_t report;
    if (top_verify_result(problem, config, result, &report) != TOP_VERIFY_OK)
    {
        result->status = TOP_STATUS_INTERNAL_ERROR;
        result->complete = 0U;
        result->needs_replan = 0U;
    }
    result->planning_ms = top_elapsed(start_ms, config);
    result->predicted_total_ms = result->motion_ms + result->planning_ms;
    return result->status;
}

top_status_t top_plan(const top_problem_t *problem,
                      const top_config_t *config,
                      top_result_t *result)
{
    top_problem_t working;
    top_config_t effective;
    top_status_t validation;
    top_search_run_t run;
    top_identify_candidate_t identify;
    uint32_t start_ms;
    int have_identify;
    int all_known;

    if (result == NULL || problem == NULL)
    {
        return TOP_STATUS_INVALID_INPUT;
    }
    memset(result, 0, sizeof(*result));
    if (config == NULL)
    {
        top_config_default(&effective);
    }
    else
    {
        effective = *config;
    }
    if (effective.translation_speed_mmps == 0U || effective.cell_size_mm == 0U)
    {
        result->status = TOP_STATUS_INVALID_INPUT;
        return result->status;
    }
    if (effective.max_nodes == 0U || effective.max_nodes > TOP_INTERNAL_MAX_NODES)
    {
        effective.max_nodes = TOP_INTERNAL_MAX_NODES;
    }
    if (effective.heuristic_weight_permille < 1000U)
    {
        effective.heuristic_weight_permille = 1000U;
    }
    working = *problem;
    top_infer_last_pair(&working);
    validation = top_problem_validate(&working);
    if (validation != TOP_STATUS_OK)
    {
        result->status = validation;
        return validation;
    }
    start_ms = top_now(&effective);
    all_known = top_all_active_ids_known(&working);
    have_identify = all_known ? 0 : top_find_identification(&working, &effective, &identify);

    if (!all_known)
    {
        if (have_identify &&
            top_build_identification_result(&working, &effective, &identify, result))
        {
            return top_finalize_verified(&working, &effective, start_ms, result);
        }
        result->status = TOP_STATUS_NO_SOLUTION;
        result->planning_ms = top_elapsed(start_ms, &effective);
        result->predicted_total_ms = result->planning_ms;
        return result->status;
    }

    if (!top_search(&working, &effective, start_ms, &run))
    {
        result->status = run.timed_out ? TOP_STATUS_TIMEOUT_NO_SOLUTION :
                         run.node_limit_hit ? TOP_STATUS_NODE_LIMIT_NO_SOLUTION :
                                              TOP_STATUS_NO_SOLUTION;
        result->timed_out = run.timed_out;
        result->node_limit_hit = run.node_limit_hit;
        result->expanded_nodes = run.expanded;
        result->generated_nodes = run.generated;
        result->planning_ms = top_elapsed(start_ms, &effective);
        result->predicted_total_ms = result->planning_ms;
        return result->status;
    }
    if (!top_reconstruct_solution(&working,
                                  &effective,
                                  &run,
                                  top_best_solution(&run),
                                  result))
    {
        memset(result, 0, sizeof(*result));
        result->status = TOP_STATUS_INTERNAL_ERROR;
        result->planning_ms = top_elapsed(start_ms, &effective);
        result->predicted_total_ms = result->planning_ms;
        return result->status;
    }
    return top_finalize_verified(&working, &effective, start_ms, result);
}

top_status_t top_problem_apply_result(top_problem_t *problem,
                                      const top_result_t *result)
{
    uint8_t i;
    if (problem == NULL || result == NULL ||
        (result->status != TOP_STATUS_OK && result->status != TOP_STATUS_PARTIAL_REPLAN))
    {
        return TOP_STATUS_INVALID_INPUT;
    }
    problem->car = result->end_state.car;
    problem->heading = result->end_state.heading;
    problem->walls = result->end_state.walls;
    for (i = 0U; i < problem->box_count; ++i)
    {
        problem->boxes[i].cell = (result->end_state.box_active_mask &
                                  (top_object_mask_t)(UINT16_C(1) << i)) != 0U ?
                                 result->end_state.box_cells[i] : (top_cell_t){-1, -1};
    }
    for (i = 0U; i < problem->target_count; ++i)
    {
        if ((result->end_state.target_active_mask &
             (top_object_mask_t)(UINT16_C(1) << i)) == 0U)
        {
            problem->targets[i].cell = (top_cell_t){-1, -1};
        }
    }
    for (i = 0U; i < problem->bomb_count; ++i)
    {
        problem->bombs[i] = (result->end_state.bomb_active_mask &
                             (uint16_t)(UINT16_C(1) << i)) != 0U ?
                            result->end_state.bomb_cells[i] : (top_cell_t){-1, -1};
    }
    return top_problem_validate(problem);
}

top_status_t top_problem_set_object_id(top_problem_t *problem,
                                       top_object_kind_t kind,
                                       uint8_t index,
                                       uint8_t id)
{
    if (problem == NULL || id == TOP_ID_UNKNOWN)
    {
        return TOP_STATUS_INVALID_INPUT;
    }
    if (kind == TOP_OBJECT_BOX && index < problem->box_count)
    {
        problem->boxes[index].id = id;
        problem->boxes[index].id_known = 1U;
        return TOP_STATUS_OK;
    }
    if (kind == TOP_OBJECT_TARGET && index < problem->target_count)
    {
        problem->targets[index].id = id;
        problem->targets[index].id_known = 1U;
        return TOP_STATUS_OK;
    }
    return TOP_STATUS_INVALID_INPUT;
}

const char *top_status_string(top_status_t status)
{
    switch (status)
    {
    case TOP_STATUS_OK: return "ok";
    case TOP_STATUS_PARTIAL_REPLAN: return "partial_replan";
    case TOP_STATUS_INVALID_INPUT: return "invalid_input";
    case TOP_STATUS_NO_SOLUTION: return "no_solution";
    case TOP_STATUS_TIMEOUT_NO_SOLUTION: return "timeout_no_solution";
    case TOP_STATUS_NODE_LIMIT_NO_SOLUTION: return "node_limit_no_solution";
    case TOP_STATUS_OUTPUT_OVERFLOW: return "output_overflow";
    case TOP_STATUS_INTERNAL_ERROR: return "internal_error";
    default: return "unknown";
    }
}

size_t top_planner_workspace_bytes(void)
{
    return sizeof(s_nodes) + sizeof(s_hash) + sizeof(s_heap) +
           sizeof(s_grid_paths) + sizeof(s_walk_cells) + sizeof(s_chain);
}
