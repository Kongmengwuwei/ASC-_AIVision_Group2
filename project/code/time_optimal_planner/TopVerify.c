#include "TopVerify.h"

#include "TopGrid.h"

#include <stdlib.h>
#include <string.h>

static int verify_compatible(const top_problem_t *state, uint8_t b, uint8_t t)
{
    if (state->match_mode == TOP_MODE_FREE_MATCH)
    {
        return 1;
    }
    return state->boxes[b].id_known && state->targets[t].id_known &&
           state->boxes[b].id == state->targets[t].id;
}

static int verify_cell_occupied(const top_problem_t *state,
                                top_cell_t cell,
                                top_object_kind_t ignored_kind,
                                uint8_t ignored_index)
{
    uint8_t i;
    for (i = 0U; i < state->box_count; ++i)
    {
        if (!(ignored_kind == TOP_OBJECT_BOX && i == ignored_index) &&
            top_cell_valid(state->boxes[i].cell) &&
            top_cell_equal(state->boxes[i].cell, cell))
        {
            return 1;
        }
    }
    for (i = 0U; i < state->bomb_count; ++i)
    {
        if (!(ignored_kind == TOP_OBJECT_BOMB && i == ignored_index) &&
            top_cell_valid(state->bombs[i]) && top_cell_equal(state->bombs[i], cell))
        {
            return 1;
        }
    }
    return 0;
}

static void verify_blockers(const top_problem_t *state,
                            top_grid_blockers_t *blockers,
                            top_cell_t *boxes)
{
    uint8_t i;
    for (i = 0U; i < state->box_count; ++i)
    {
        boxes[i] = state->boxes[i].cell;
    }
    blockers->walls = &state->walls;
    blockers->boxes = boxes;
    blockers->box_count = state->box_count;
    blockers->bombs = state->bombs;
    blockers->bomb_count = state->bomb_count;
    blockers->ignored_box = -1;
    blockers->ignored_bomb = -1;
}

static void verify_explode(top_problem_t *state, top_cell_t center)
{
    int dr;
    int dc;
    for (dr = -1; dr <= 1; ++dr)
    {
        for (dc = -1; dc <= 1; ++dc)
        {
            (void)top_problem_set_wall(state,
                                       center.row + dr,
                                       center.col + dc,
                                       0);
        }
    }
}

static int verify_all_boxes_done(const top_problem_t *state)
{
    uint8_t i;
    for (i = 0U; i < state->box_count; ++i)
    {
        if (top_cell_valid(state->boxes[i].cell))
        {
            return 0;
        }
    }
    return 1;
}

static void verify_clear_internal_walls(top_problem_t *state)
{
    memset(&state->walls, 0, sizeof(state->walls));
}

static int verify_identification(const top_problem_t *state,
                                 const top_segment_t *segment)
{
    top_cell_t object;
    int dr;
    int dc;
    int distance;
    int step;
    uint8_t i;

    if (segment->object_kind == TOP_OBJECT_BOX &&
        segment->object_index < state->box_count)
    {
        object = state->boxes[segment->object_index].cell;
    }
    else if (segment->object_kind == TOP_OBJECT_TARGET &&
             segment->object_index < state->target_count)
    {
        object = state->targets[segment->object_index].cell;
    }
    else
    {
        return 0;
    }
    if (!top_cell_valid(object))
    {
        return 0;
    }
    dr = (int)object.row - (int)state->car.row;
    dc = (int)object.col - (int)state->car.col;
    if (dr != 0 && dc != 0)
    {
        return 0;
    }
    distance = abs(dr) + abs(dc);
    if (distance < 1 || distance > 3)
    {
        return 0;
    }
    if ((dr == 0 && dc > 0 && state->heading != TOP_HEADING_RIGHT) ||
        (dr > 0 && dc == 0 && state->heading != TOP_HEADING_DOWN) ||
        (dr == 0 && dc < 0 && state->heading != TOP_HEADING_LEFT) ||
        (dr < 0 && dc == 0 && state->heading != TOP_HEADING_UP))
    {
        return 0;
    }
    for (step = 1; step < distance; ++step)
    {
        top_cell_t cell = {
            (int8_t)(state->car.row + (dr == 0 ? 0 : (dr > 0 ? step : -step))),
            (int8_t)(state->car.col + (dc == 0 ? 0 : (dc > 0 ? step : -step)))};
        if (top_problem_has_wall(state, cell.row, cell.col) ||
            verify_cell_occupied(state, cell, TOP_OBJECT_NONE, 0U))
        {
            return 0;
        }
        for (i = 0U; i < state->target_count; ++i)
        {
            if (!(segment->object_kind == TOP_OBJECT_TARGET &&
                  segment->object_index == i) &&
                top_cell_valid(state->targets[i].cell) &&
                top_cell_equal(state->targets[i].cell, cell))
            {
                return 0;
            }
        }
    }
    return 1;
}

static int verify_end_state(const top_problem_t *state,
                            const top_result_t *result)
{
    uint8_t i;
    if (!top_cell_equal(state->car, result->end_state.car) ||
        state->heading != result->end_state.heading ||
        memcmp(&state->walls, &result->end_state.walls, sizeof(state->walls)) != 0)
    {
        return 0;
    }
    for (i = 0U; i < state->box_count; ++i)
    {
        int active = top_cell_valid(state->boxes[i].cell);
        if (active != ((result->end_state.box_active_mask & (uint8_t)(1U << i)) != 0U))
        {
            return 0;
        }
        if (active && !top_cell_equal(state->boxes[i].cell,
                                      result->end_state.box_cells[i]))
        {
            return 0;
        }
    }
    for (i = 0U; i < state->target_count; ++i)
    {
        int active = top_cell_valid(state->targets[i].cell);
        if (active != ((result->end_state.target_active_mask & (uint8_t)(1U << i)) != 0U))
        {
            return 0;
        }
    }
    for (i = 0U; i < state->bomb_count; ++i)
    {
        int active = top_cell_valid(state->bombs[i]);
        if (active != ((result->end_state.bomb_active_mask &
                        (uint16_t)(UINT16_C(1) << i)) != 0U))
        {
            return 0;
        }
        if (active && !top_cell_equal(state->bombs[i], result->end_state.bomb_cells[i]))
        {
            return 0;
        }
    }
    return 1;
}

top_verify_code_t top_verify_result(const top_problem_t *problem,
                                    const top_config_t *config,
                                    const top_result_t *result,
                                    top_verify_report_t *report)
{
    top_problem_t state;
    uint32_t recomputed = 0U;
    uint8_t pending_bomb_wait = 0U;
    uint16_t s;
    top_verify_code_t code = TOP_VERIFY_OK;

    if (report != NULL)
    {
        memset(report, 0, sizeof(*report));
    }
    if (problem == NULL || config == NULL || result == NULL ||
        top_problem_validate(problem) != TOP_STATUS_OK ||
        (result->status != TOP_STATUS_OK && result->status != TOP_STATUS_PARTIAL_REPLAN))
    {
        code = TOP_VERIFY_BAD_INPUT;
        goto done;
    }
    state = *problem;
    for (s = 0U; s < result->segment_count; ++s)
    {
        const top_segment_t *segment = &result->segments[s];
        const top_path_point_t *points;
        if (report != NULL)
        {
            report->segment_index = s;
        }
        if (segment->point_count == 0U ||
            (uint32_t)segment->first_point + segment->point_count > result->exec_point_count)
        {
            code = TOP_VERIFY_BAD_POINT_RANGE;
            goto done;
        }
        points = &result->exec_points[segment->first_point];
        if (!top_cell_equal(points[0].cell, state.car))
        {
            code = TOP_VERIFY_DISCONTINUOUS;
            goto done;
        }
        if (pending_bomb_wait && segment->action != TOP_ACTION_WAIT)
        {
            code = TOP_VERIFY_BAD_WAIT;
            goto done;
        }
        if (segment->action == TOP_ACTION_MOVE)
        {
            top_cell_t boxes[TOP_MAX_BOXES];
            top_grid_blockers_t blockers;
            uint32_t duration = 0U;
            uint16_t p;
            verify_blockers(&state, &blockers, boxes);
            for (p = 1U; p < segment->point_count; ++p)
            {
                if (!top_grid_line_clear(points[p - 1U].cell,
                                         points[p].cell,
                                         &blockers))
                {
                    code = TOP_VERIFY_COLLISION;
                    goto done;
                }
                duration += top_distance_ms(points[p - 1U].cell,
                                            points[p].cell,
                                            config);
            }
            if (duration != segment->duration_ms)
            {
                code = TOP_VERIFY_BAD_DURATION;
                goto done;
            }
            state.car = points[segment->point_count - 1U].cell;
        }
        else if (segment->action == TOP_ACTION_PUSH_BOX)
        {
            uint8_t b = segment->object_index;
            top_cell_t object;
            top_cell_t delta;
            int push_count;
            int push_step;
            uint8_t t;
            int delivered = 0;
            if (segment->point_count != 2U || b >= state.box_count ||
                !top_cell_valid(state.boxes[b].cell))
            {
                code = TOP_VERIFY_BAD_PUSH;
                goto done;
            }
            delta.row = (int8_t)(((int)points[1].cell.row - (int)points[0].cell.row > 0) -
                                 ((int)points[1].cell.row - (int)points[0].cell.row < 0));
            delta.col = (int8_t)(((int)points[1].cell.col - (int)points[0].cell.col > 0) -
                                 ((int)points[1].cell.col - (int)points[0].cell.col < 0));
            push_count = abs((int)points[1].cell.row - (int)points[0].cell.row) +
                         abs((int)points[1].cell.col - (int)points[0].cell.col);
            object.row = (int8_t)(points[0].cell.row + delta.row);
            object.col = (int8_t)(points[0].cell.col + delta.col);
            if (push_count < 1 || abs(delta.row) + abs(delta.col) != 1 ||
                !top_cell_equal(state.boxes[b].cell, object) ||
                segment->effect_cell.row != points[1].cell.row + delta.row ||
                segment->effect_cell.col != points[1].cell.col + delta.col ||
                segment->duration_ms !=
                    (uint32_t)push_count * config->cell_size_mm * 1000U /
                        config->translation_speed_mmps)
            {
                code = TOP_VERIFY_BAD_PUSH;
                goto done;
            }
            for (push_step = 0; push_step < push_count; ++push_step)
            {
                top_cell_t destination = {(int8_t)(object.row + delta.row),
                                          (int8_t)(object.col + delta.col)};
                if (!top_cell_valid(destination) ||
                    top_problem_has_wall(&state, destination.row, destination.col) ||
                    verify_cell_occupied(&state, destination, TOP_OBJECT_BOX, b))
                {
                    code = TOP_VERIFY_BAD_PUSH;
                    goto done;
                }
                state.car = object;
                state.boxes[b].cell = destination;
                for (t = 0U; t < state.target_count; ++t)
                {
                    if (top_cell_valid(state.targets[t].cell) &&
                        top_cell_equal(state.targets[t].cell, destination) &&
                        verify_compatible(&state, b, t))
                    {
                        if (push_step + 1 != push_count)
                        {
                            code = TOP_VERIFY_BAD_PUSH;
                            goto done;
                        }
                        state.boxes[b].cell = (top_cell_t){-1, -1};
                        state.targets[t].cell = (top_cell_t){-1, -1};
                        delivered = 1;
                        break;
                    }
                }
                object = destination;
            }
            if (delivered && verify_all_boxes_done(&state))
            {
                verify_clear_internal_walls(&state);
            }
        }
        else if (segment->action == TOP_ACTION_PUSH_BOMB)
        {
            uint8_t b = segment->object_index;
            top_cell_t object;
            top_cell_t delta;
            int push_count;
            int push_step;
            if (segment->point_count != 2U || b >= state.bomb_count ||
                !top_cell_valid(state.bombs[b]))
            {
                code = TOP_VERIFY_BAD_PUSH;
                goto done;
            }
            delta.row = (int8_t)(((int)points[1].cell.row - (int)points[0].cell.row > 0) -
                                 ((int)points[1].cell.row - (int)points[0].cell.row < 0));
            delta.col = (int8_t)(((int)points[1].cell.col - (int)points[0].cell.col > 0) -
                                 ((int)points[1].cell.col - (int)points[0].cell.col < 0));
            push_count = abs((int)points[1].cell.row - (int)points[0].cell.row) +
                         abs((int)points[1].cell.col - (int)points[0].cell.col);
            object.row = (int8_t)(points[0].cell.row + delta.row);
            object.col = (int8_t)(points[0].cell.col + delta.col);
            if (push_count < 1 || abs(delta.row) + abs(delta.col) != 1 ||
                !top_cell_equal(state.bombs[b], object) ||
                segment->effect_cell.row != points[1].cell.row + delta.row ||
                segment->effect_cell.col != points[1].cell.col + delta.col ||
                segment->duration_ms !=
                    (uint32_t)push_count * config->cell_size_mm * 1000U /
                        config->translation_speed_mmps)
            {
                code = TOP_VERIFY_BAD_PUSH;
                goto done;
            }
            for (push_step = 0; push_step < push_count; ++push_step)
            {
                top_cell_t destination = {(int8_t)(object.row + delta.row),
                                          (int8_t)(object.col + delta.col)};
                int wall = top_problem_has_wall(&state, destination.row, destination.col);
                if (!top_cell_valid(destination) ||
                    verify_cell_occupied(&state, destination, TOP_OBJECT_BOMB, b) ||
                    (wall && push_step + 1 != push_count))
                {
                    code = TOP_VERIFY_BAD_PUSH;
                    goto done;
                }
                state.car = object;
                if (wall)
                {
                    if ((segment->end_events & TOP_EVENT_BOMB_EXPLODED) == 0U)
                    {
                        code = TOP_VERIFY_BAD_EXPLOSION;
                        goto done;
                    }
                    state.bombs[b] = (top_cell_t){-1, -1};
                    verify_explode(&state, destination);
                    pending_bomb_wait = 1U;
                }
                else
                {
                    if (push_step + 1 == push_count &&
                        (segment->end_events & TOP_EVENT_BOMB_EXPLODED) != 0U)
                    {
                        code = TOP_VERIFY_BAD_EXPLOSION;
                        goto done;
                    }
                    state.bombs[b] = destination;
                }
                object = destination;
            }
        }
        else if (segment->action == TOP_ACTION_WAIT)
        {
            if (!pending_bomb_wait || segment->duration_ms != config->bomb_wait_ms)
            {
                code = TOP_VERIFY_BAD_WAIT;
                goto done;
            }
            pending_bomb_wait = 0U;
        }
        else if (segment->action == TOP_ACTION_ROTATE)
        {
            uint32_t duration = top_rotation_ms(state.heading,
                                                segment->target_heading,
                                                config);
            if (duration != segment->duration_ms)
            {
                code = TOP_VERIFY_BAD_ROTATION;
                goto done;
            }
            state.heading = segment->target_heading;
        }
        else if (segment->action == TOP_ACTION_IDENTIFY)
        {
            uint32_t expected;
            int distance;
            top_cell_t object;
            if (segment->object_kind == TOP_OBJECT_BOX &&
                segment->object_index < state.box_count)
            {
                object = state.boxes[segment->object_index].cell;
            }
            else if (segment->object_kind == TOP_OBJECT_TARGET &&
                     segment->object_index < state.target_count)
            {
                object = state.targets[segment->object_index].cell;
            }
            else
            {
                code = TOP_VERIFY_BAD_IDENTIFICATION;
                goto done;
            }
            distance = abs((int)object.row - (int)state.car.row) +
                       abs((int)object.col - (int)state.car.col);
            expected = distance == 1 ? config->identify_near_ms :
                                       config->identify_far_ms;
            if (!verify_identification(&state, segment) ||
                segment->duration_ms != expected)
            {
                code = TOP_VERIFY_BAD_IDENTIFICATION;
                goto done;
            }
        }
        else
        {
            code = TOP_VERIFY_BAD_INPUT;
            goto done;
        }
        recomputed += segment->duration_ms;
    }
    if (pending_bomb_wait)
    {
        code = TOP_VERIFY_BAD_WAIT;
        goto done;
    }
    if (recomputed != result->motion_ms)
    {
        code = TOP_VERIFY_BAD_DURATION;
        goto done;
    }
    if (result->complete)
    {
        if (!verify_all_boxes_done(&state) || state.heading != TOP_HEADING_RIGHT ||
            state.car.col != 0 || (state.car.row != 4 && state.car.row != 5))
        {
            code = TOP_VERIFY_BAD_COMPLETION;
            goto done;
        }
    }
    if (!verify_end_state(&state, result))
    {
        code = TOP_VERIFY_END_STATE_MISMATCH;
        goto done;
    }

done:
    if (report != NULL)
    {
        report->code = code;
        report->recomputed_motion_ms = recomputed;
    }
    return code;
}

const char *top_verify_string(top_verify_code_t code)
{
    switch (code)
    {
    case TOP_VERIFY_OK: return "ok";
    case TOP_VERIFY_BAD_INPUT: return "bad_input";
    case TOP_VERIFY_BAD_POINT_RANGE: return "bad_point_range";
    case TOP_VERIFY_DISCONTINUOUS: return "discontinuous";
    case TOP_VERIFY_COLLISION: return "collision";
    case TOP_VERIFY_BAD_DURATION: return "bad_duration";
    case TOP_VERIFY_BAD_PUSH: return "bad_push";
    case TOP_VERIFY_BAD_EXPLOSION: return "bad_explosion";
    case TOP_VERIFY_BAD_WAIT: return "bad_wait";
    case TOP_VERIFY_BAD_ROTATION: return "bad_rotation";
    case TOP_VERIFY_BAD_IDENTIFICATION: return "bad_identification";
    case TOP_VERIFY_BAD_COMPLETION: return "bad_completion";
    case TOP_VERIFY_END_STATE_MISMATCH: return "end_state_mismatch";
    default: return "unknown";
    }
}
