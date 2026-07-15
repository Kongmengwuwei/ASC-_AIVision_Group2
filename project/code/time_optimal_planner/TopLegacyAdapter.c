#include "TopLegacyAdapter.h"

#include "TopGrid.h"
#include "path.h"

#include <string.h>

static uint8 top_legacy_event_bits(top_event_mask_t events)
{
    uint8 legacy = PATH_EVENT_NONE;
    if ((events & TOP_EVENT_BOMB_EXPLODED) != 0U)
    {
        legacy |= BOMB_EXPLOSION;
    }
    if ((events & TOP_EVENT_IDENTIFY) != 0U)
    {
        legacy |= IDENTIFICATION;
    }
    if ((events & (TOP_EVENT_PUSH_BOX_START | TOP_EVENT_PUSH_BOMB_START)) != 0U)
    {
        legacy |= PUSH_START_POINT;
    }
    if ((events & (TOP_EVENT_PUSH_BOX_END | TOP_EVENT_PUSH_BOMB_END)) != 0U)
    {
        legacy |= PUSH_END_POINT;
    }
    return legacy;
}

top_status_t top_legacy_import_globals(top_match_mode_t mode,
                                       const uint8 *box_id_known,
                                       const uint8 *target_id_known,
                                       top_problem_t *problem)
{
    size_t i;
    if (problem == NULL || Boxes_count > TOP_MAX_BOXES ||
        Targets_count > TOP_MAX_TARGETS || Bombs_count > TOP_MAX_BOMBS)
    {
        return TOP_STATUS_INVALID_INPUT;
    }
    top_problem_clear(problem);
    problem->match_mode = mode;
    problem->car.row = (int8_t)car.row;
    problem->car.col = (int8_t)car.col;
    problem->heading = TOP_HEADING_RIGHT;
    problem->box_count = (uint8_t)Boxes_count;
    problem->target_count = (uint8_t)Targets_count;
    problem->bomb_count = (uint8_t)Bombs_count;
    for (i = 0U; i < Obstacles_count; ++i)
    {
        if (!top_problem_set_wall(problem, obstacles[i].row, obstacles[i].col, 1))
        {
            return TOP_STATUS_INVALID_INPUT;
        }
    }
    for (i = 0U; i < Boxes_count; ++i)
    {
        problem->boxes[i].cell.row = (int8_t)boxes[i].row;
        problem->boxes[i].cell.col = (int8_t)boxes[i].col;
        problem->boxes[i].id = boxes[i].id;
        problem->boxes[i].id_known = mode == TOP_MODE_FREE_MATCH ? 1U :
                                      (box_id_known != NULL ? box_id_known[i] : 0U);
    }
    for (i = 0U; i < Targets_count; ++i)
    {
        problem->targets[i].cell.row = (int8_t)targets[i].row;
        problem->targets[i].cell.col = (int8_t)targets[i].col;
        problem->targets[i].id = targets[i].id;
        problem->targets[i].id_known = mode == TOP_MODE_FREE_MATCH ? 1U :
                                        (target_id_known != NULL ? target_id_known[i] : 0U);
    }
    for (i = 0U; i < Bombs_count; ++i)
    {
        problem->bombs[i].row = (int8_t)bombs[i].row;
        problem->bombs[i].col = (int8_t)bombs[i].col;
    }
    return top_problem_validate(problem);
}

top_status_t top_legacy_export_exec_path(const top_result_t *result,
                                         Position *legacy_path,
                                         size_t capacity,
                                         size_t *count,
                                         uint8 remap_for_path_follow)
{
    size_t i;
    size_t out_count = 0U;
    if (count != NULL)
    {
        *count = 0U;
    }
    if (result == NULL || legacy_path == NULL || count == NULL || capacity == 0U ||
        (result->status != TOP_STATUS_OK && result->status != TOP_STATUS_PARTIAL_REPLAN))
    {
        return TOP_STATUS_INVALID_INPUT;
    }
    memset(legacy_path, 0, capacity * sizeof(*legacy_path));
    for (i = 0U; i < result->exec_point_count; ++i)
    {
        Position point;
        uint8 events;
        if (!top_cell_valid(result->exec_points[i].cell))
        {
            return TOP_STATUS_INTERNAL_ERROR;
        }
        point.row = (uint8)result->exec_points[i].cell.row;
        point.col = (uint8)result->exec_points[i].cell.col;
        events = top_legacy_event_bits(result->exec_points[i].events);
        point.id = events;
        if (out_count > 0U &&
            legacy_path[out_count - 1U].row == point.row &&
            legacy_path[out_count - 1U].col == point.col)
        {
            legacy_path[out_count - 1U].id |= point.id;
            continue;
        }
        if (out_count >= capacity)
        {
            return TOP_STATUS_OUTPUT_OVERFLOW;
        }
        legacy_path[out_count++] = point;
    }
    if (remap_for_path_follow)
    {
        for (i = 0U; i < out_count; ++i)
        {
            path_remap_exec_point(&legacy_path[i]);
        }
    }
    *count = out_count;
    return out_count >= 1U ? TOP_STATUS_OK : TOP_STATUS_OUTPUT_OVERFLOW;
}
