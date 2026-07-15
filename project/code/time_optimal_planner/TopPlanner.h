#ifndef TOP_PLANNER_H
#define TOP_PLANNER_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* The competition image contains a 12x16 board including the permanent wall.
 * Planning uses the 10x14 playable interior. */
#define TOP_ROWS 10U
#define TOP_COLS 14U
#define TOP_CELL_COUNT (TOP_ROWS * TOP_COLS)

#define TOP_MAX_BOXES 5U
#define TOP_MAX_TARGETS 5U
#define TOP_MAX_BOMBS 10U
#define TOP_MAX_RAW_POINTS 1024U
#define TOP_MAX_EXEC_POINTS 384U
#define TOP_MAX_SEGMENTS 256U
#define TOP_INVALID_CELL 0xFFU
#define TOP_ID_UNKNOWN 0xFFU

/* New event protocol.  It is deliberately wider than Position.id: events are
 * not object IDs and a point may carry several independent events. */
typedef uint32_t top_event_mask_t;
#define TOP_EVENT_NONE             UINT32_C(0)
#define TOP_EVENT_ROUTE_START      (UINT32_C(1) << 0)
#define TOP_EVENT_MOVE_END         (UINT32_C(1) << 1)
#define TOP_EVENT_ROTATE           (UINT32_C(1) << 2)
#define TOP_EVENT_IDENTIFY         (UINT32_C(1) << 3)
#define TOP_EVENT_PUSH_BOX_START   (UINT32_C(1) << 4)
#define TOP_EVENT_PUSH_BOX_END     (UINT32_C(1) << 5)
#define TOP_EVENT_BOX_DELIVERED    (UINT32_C(1) << 6)
#define TOP_EVENT_PUSH_BOMB_START  (UINT32_C(1) << 7)
#define TOP_EVENT_PUSH_BOMB_END    (UINT32_C(1) << 8)
#define TOP_EVENT_BOMB_EXPLODED    (UINT32_C(1) << 9)
#define TOP_EVENT_MAP_CHANGED      (UINT32_C(1) << 10)
#define TOP_EVENT_WAIT             (UINT32_C(1) << 11)
#define TOP_EVENT_RETURNED         (UINT32_C(1) << 12)
#define TOP_EVENT_ROUTE_END        (UINT32_C(1) << 13)

typedef enum
{
    TOP_HEADING_RIGHT = 0,
    TOP_HEADING_DOWN = 1,
    TOP_HEADING_LEFT = 2,
    TOP_HEADING_UP = 3
} top_heading_t;

typedef enum
{
    TOP_MODE_FREE_MATCH = 0,
    TOP_MODE_ID_MATCH = 1
} top_match_mode_t;

typedef enum
{
    TOP_OBJECT_NONE = 0,
    TOP_OBJECT_BOX = 1,
    TOP_OBJECT_TARGET = 2,
    TOP_OBJECT_BOMB = 3
} top_object_kind_t;

typedef enum
{
    TOP_ACTION_NONE = 0,
    TOP_ACTION_MOVE = 1,
    TOP_ACTION_PUSH_BOX = 2,
    TOP_ACTION_PUSH_BOMB = 3,
    TOP_ACTION_ROTATE = 4,
    TOP_ACTION_IDENTIFY = 5,
    TOP_ACTION_WAIT = 6
} top_action_t;

typedef enum
{
    TOP_STATUS_OK = 0,
    TOP_STATUS_PARTIAL_REPLAN = 1,
    TOP_STATUS_INVALID_INPUT = 2,
    TOP_STATUS_NO_SOLUTION = 3,
    TOP_STATUS_TIMEOUT_NO_SOLUTION = 4,
    TOP_STATUS_NODE_LIMIT_NO_SOLUTION = 5,
    TOP_STATUS_OUTPUT_OVERFLOW = 6,
    TOP_STATUS_INTERNAL_ERROR = 7
} top_status_t;

typedef struct
{
    int8_t row;
    int8_t col;
} top_cell_t;

typedef struct
{
    top_cell_t cell;
    uint8_t id;
    uint8_t id_known;
} top_labeled_object_t;

typedef struct
{
    uint32_t word[5];
} top_wall_bits_t;

typedef struct
{
    top_match_mode_t match_mode;
    top_cell_t car;
    top_heading_t heading;
    top_wall_bits_t walls;
    uint8_t box_count;
    uint8_t target_count;
    uint8_t bomb_count;
    top_labeled_object_t boxes[TOP_MAX_BOXES];
    top_labeled_object_t targets[TOP_MAX_TARGETS];
    top_cell_t bombs[TOP_MAX_BOMBS];
} top_problem_t;

typedef uint32_t (*top_now_ms_fn)(void *user);

typedef struct
{
    uint16_t cell_size_mm;
    uint16_t translation_speed_mmps;
    uint16_t rotate_90_ms;
    uint16_t identify_near_ms;
    uint16_t identify_far_ms;
    uint16_t bomb_wait_ms;
    uint16_t planning_budget_ms;
    uint16_t interleave_bias_ms;
    uint16_t max_expansions;
    uint16_t max_nodes;
    uint16_t heuristic_weight_permille;
    uint8_t enable_diagonal;
    uint8_t enable_bombs;
    top_now_ms_fn now_ms;
    void *now_user;
} top_config_t;

typedef struct
{
    top_cell_t cell;
    top_heading_t heading;
    top_action_t action_from_previous;
    top_event_mask_t events;
    top_object_kind_t object_kind;
    uint8_t object_index;
} top_path_point_t;

typedef struct
{
    top_action_t action;
    uint16_t first_point;
    uint16_t point_count;
    uint32_t duration_ms;
    top_event_mask_t end_events;
    top_object_kind_t object_kind;
    uint8_t object_index;
    top_cell_t effect_cell;
    top_heading_t target_heading;
} top_segment_t;

/* State after the returned plan has been executed.  Stable object indices are
 * preserved so an identification result can be applied and planning resumed. */
typedef struct
{
    top_cell_t car;
    top_heading_t heading;
    top_wall_bits_t walls;
    uint8_t box_active_mask;
    uint8_t target_active_mask;
    uint16_t bomb_active_mask;
    top_cell_t box_cells[TOP_MAX_BOXES];
    top_cell_t bomb_cells[TOP_MAX_BOMBS];
} top_end_state_t;

typedef struct
{
    top_status_t status;
    uint8_t complete;
    uint8_t needs_replan;
    uint8_t timed_out;
    uint8_t node_limit_hit;
    uint16_t expanded_nodes;
    uint16_t generated_nodes;
    uint32_t planning_ms;
    uint32_t motion_ms;
    uint32_t predicted_total_ms;
    uint16_t raw_point_count;
    uint16_t exec_point_count;
    uint16_t segment_count;
    top_object_kind_t requested_identify_kind;
    uint8_t requested_identify_index;
    top_end_state_t end_state;
    top_path_point_t raw_points[TOP_MAX_RAW_POINTS];
    top_path_point_t exec_points[TOP_MAX_EXEC_POINTS];
    top_segment_t segments[TOP_MAX_SEGMENTS];
} top_result_t;

void top_config_default(top_config_t *config);
void top_problem_clear(top_problem_t *problem);
int top_problem_set_wall(top_problem_t *problem, int row, int col, int blocked);
int top_problem_has_wall(const top_problem_t *problem, int row, int col);
top_status_t top_problem_validate(const top_problem_t *problem);

/* Plans either a complete task, one interleaved known delivery, or the next
 * identification action.  A PARTIAL_REPLAN result is intentionally executable
 * and asks the caller to update observations/state before calling again. */
top_status_t top_plan(const top_problem_t *problem,
                      const top_config_t *config,
                      top_result_t *result);

/* Applies result.end_state to the mutable problem while preserving labels and
 * known flags.  Use this after executing a PARTIAL_REPLAN result. */
top_status_t top_problem_apply_result(top_problem_t *problem,
                                      const top_result_t *result);

/* Records a camera result at the stable object index requested by the plan. */
top_status_t top_problem_set_object_id(top_problem_t *problem,
                                       top_object_kind_t kind,
                                       uint8_t index,
                                       uint8_t id);

const char *top_status_string(top_status_t status);
size_t top_planner_workspace_bytes(void);

#ifdef __cplusplus
}
#endif

#endif
