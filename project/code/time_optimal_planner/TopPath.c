#include "TopPath.h"

#include <string.h>

static top_path_point_t top_make_point(top_cell_t cell,
                                       top_heading_t heading,
                                       top_action_t action,
                                       top_event_mask_t events,
                                       top_object_kind_t kind,
                                       uint8_t object_index)
{
    top_path_point_t point;
    point.cell = cell;
    point.heading = heading;
    point.action_from_previous = action;
    point.events = events;
    point.object_kind = kind;
    point.object_index = object_index;
    return point;
}

static int top_add_segment(top_path_builder_t *builder,
                           top_action_t action,
                           uint16_t first_point,
                           uint16_t point_count,
                           uint32_t duration_ms,
                           top_event_mask_t end_events,
                           top_object_kind_t kind,
                           uint8_t object_index,
                           top_cell_t effect_cell,
                           top_heading_t target_heading)
{
    top_segment_t *segment;
    if (builder == NULL || builder->failed ||
        builder->result->segment_count >= TOP_MAX_SEGMENTS)
    {
        if (builder != NULL)
        {
            builder->failed = 1U;
        }
        return 0;
    }
    segment = &builder->result->segments[builder->result->segment_count++];
    segment->action = action;
    segment->first_point = first_point;
    segment->point_count = point_count;
    segment->duration_ms = duration_ms;
    segment->end_events = end_events;
    segment->object_kind = kind;
    segment->object_index = object_index;
    segment->effect_cell = effect_cell;
    segment->target_heading = target_heading;
    builder->result->motion_ms += duration_ms;
    return 1;
}

void top_path_builder_init(top_path_builder_t *builder,
                           top_result_t *result,
                           const top_config_t *config,
                           top_cell_t start,
                           top_heading_t heading)
{
    if (builder == NULL)
    {
        return;
    }
    memset(builder, 0, sizeof(*builder));
    builder->result = result;
    builder->config = config;
    if (result == NULL || config == NULL)
    {
        builder->failed = 1U;
        return;
    }
    result->raw_points[0] = top_make_point(start,
                                           heading,
                                           TOP_ACTION_NONE,
                                           TOP_EVENT_ROUTE_START,
                                           TOP_OBJECT_NONE,
                                           0U);
    result->exec_points[0] = result->raw_points[0];
    result->raw_point_count = 1U;
    result->exec_point_count = 1U;
}

int top_path_append_walk(top_path_builder_t *builder,
                         const top_cell_t *raw,
                         uint16_t raw_count,
                         const top_grid_blockers_t *blockers,
                         top_event_mask_t end_events)
{
    top_cell_t optimized[TOP_CELL_COUNT];
    uint16_t optimized_count = 0U;
    uint32_t duration_ms = 0U;
    uint16_t raw_start;
    uint16_t exec_start;
    uint16_t i;
    top_heading_t heading;

    if (builder == NULL || builder->failed || raw == NULL || raw_count == 0U)
    {
        return 0;
    }
    heading = builder->result->exec_points[builder->result->exec_point_count - 1U].heading;
    if (!top_cell_equal(builder->result->exec_points[builder->result->exec_point_count - 1U].cell,
                        raw[0]))
    {
        builder->failed = 1U;
        return 0;
    }
    if (raw_count == 1U)
    {
        /* Reaching the current cell is not a movement event.  In particular,
         * consecutive pushes call this with a one-cell reconstructed walk and
         * must remain mergeable into one continuous execution segment. */
        return 1;
    }
    if (!top_grid_optimize_walk(raw,
                                raw_count,
                                blockers,
                                builder->config,
                                optimized,
                                TOP_CELL_COUNT,
                                &optimized_count,
                                &duration_ms))
    {
        builder->failed = 1U;
        return 0;
    }
    if ((uint32_t)builder->result->raw_point_count + raw_count - 1U > TOP_MAX_RAW_POINTS ||
        (uint32_t)builder->result->exec_point_count + optimized_count - 1U > TOP_MAX_EXEC_POINTS)
    {
        builder->failed = 1U;
        return 0;
    }
    raw_start = (uint16_t)(builder->result->raw_point_count - 1U);
    exec_start = (uint16_t)(builder->result->exec_point_count - 1U);
    for (i = 1U; i < raw_count; ++i)
    {
        top_event_mask_t events = (i + 1U == raw_count) ? end_events : TOP_EVENT_NONE;
        builder->result->raw_points[builder->result->raw_point_count++] =
            top_make_point(raw[i], heading, TOP_ACTION_MOVE, events,
                           TOP_OBJECT_NONE, 0U);
    }
    for (i = 1U; i < optimized_count; ++i)
    {
        top_event_mask_t events = (i + 1U == optimized_count) ? end_events : TOP_EVENT_NONE;
        builder->result->exec_points[builder->result->exec_point_count++] =
            top_make_point(optimized[i], heading, TOP_ACTION_MOVE, events,
                           TOP_OBJECT_NONE, 0U);
    }
    return top_add_segment(builder,
                           TOP_ACTION_MOVE,
                           exec_start,
                           optimized_count,
                           duration_ms,
                           end_events,
                           TOP_OBJECT_NONE,
                           0U,
                           raw[raw_count - 1U],
                           heading) && raw_start < TOP_MAX_RAW_POINTS;
}

int top_path_append_push(top_path_builder_t *builder,
                         top_action_t action,
                         top_object_kind_t kind,
                         uint8_t object_index,
                         top_cell_t car_destination,
                         top_cell_t effect_cell,
                         top_event_mask_t events,
                         uint32_t duration_ms)
{
    top_result_t *result;
    top_path_point_t *raw_previous;
    top_path_point_t *exec_previous;
    top_heading_t heading;
    uint16_t exec_start;
    top_event_mask_t start_event;
    top_segment_t *previous_segment = NULL;

    if (builder == NULL || builder->failed ||
        (action != TOP_ACTION_PUSH_BOX && action != TOP_ACTION_PUSH_BOMB))
    {
        return 0;
    }
    result = builder->result;
    if (result->raw_point_count >= TOP_MAX_RAW_POINTS)
    {
        builder->failed = 1U;
        return 0;
    }
    heading = result->exec_points[result->exec_point_count - 1U].heading;
    start_event = action == TOP_ACTION_PUSH_BOX ? TOP_EVENT_PUSH_BOX_START :
                                                    TOP_EVENT_PUSH_BOMB_START;
    raw_previous = &result->raw_points[result->raw_point_count - 1U];
    exec_previous = &result->exec_points[result->exec_point_count - 1U];
    raw_previous->events |= start_event;
    result->raw_points[result->raw_point_count++] =
        top_make_point(car_destination, heading, action, events, kind, object_index);

    if (result->segment_count > 0U)
    {
        previous_segment = &result->segments[result->segment_count - 1U];
    }
    if (previous_segment != NULL && previous_segment->action == action &&
        previous_segment->object_kind == kind &&
        previous_segment->object_index == object_index &&
        previous_segment->point_count == 2U &&
        (previous_segment->end_events &
         (TOP_EVENT_BOX_DELIVERED | TOP_EVENT_BOMB_EXPLODED | TOP_EVENT_MAP_CHANGED)) == 0U)
    {
        top_path_point_t *segment_start =
            &result->exec_points[previous_segment->first_point];
        int old_dr = (int)exec_previous->cell.row - (int)segment_start->cell.row;
        int old_dc = (int)exec_previous->cell.col - (int)segment_start->cell.col;
        int new_dr = (int)car_destination.row - (int)exec_previous->cell.row;
        int new_dc = (int)car_destination.col - (int)exec_previous->cell.col;
        int old_row_sign = (old_dr > 0) - (old_dr < 0);
        int old_col_sign = (old_dc > 0) - (old_dc < 0);
        if (new_dr == old_row_sign && new_dc == old_col_sign)
        {
            exec_previous->cell = car_destination;
            exec_previous->events = events;
            exec_previous->action_from_previous = action;
            exec_previous->object_kind = kind;
            exec_previous->object_index = object_index;
            previous_segment->duration_ms += duration_ms;
            previous_segment->end_events = events;
            previous_segment->effect_cell = effect_cell;
            result->motion_ms += duration_ms;
            return 1;
        }
    }
    if (result->exec_point_count >= TOP_MAX_EXEC_POINTS)
    {
        builder->failed = 1U;
        return 0;
    }
    exec_previous->events |= start_event;
    exec_start = (uint16_t)(result->exec_point_count - 1U);
    result->exec_points[result->exec_point_count++] =
        top_make_point(car_destination, heading, action, events, kind, object_index);
    return top_add_segment(builder,
                           action,
                           exec_start,
                           2U,
                           duration_ms,
                           events,
                           kind,
                           object_index,
                           effect_cell,
                           heading);
}

int top_path_append_rotate(top_path_builder_t *builder,
                           top_heading_t target)
{
    top_result_t *result;
    top_path_point_t previous;
    uint32_t duration;
    uint16_t start;

    if (builder == NULL || builder->failed)
    {
        return 0;
    }
    result = builder->result;
    previous = result->exec_points[result->exec_point_count - 1U];
    duration = top_rotation_ms(previous.heading, target, builder->config);
    if (duration == 0U)
    {
        return 1;
    }
    if (result->raw_point_count >= TOP_MAX_RAW_POINTS ||
        result->exec_point_count >= TOP_MAX_EXEC_POINTS)
    {
        builder->failed = 1U;
        return 0;
    }
    start = (uint16_t)(result->exec_point_count - 1U);
    result->raw_points[result->raw_point_count++] =
        top_make_point(previous.cell, target, TOP_ACTION_ROTATE,
                       TOP_EVENT_ROTATE, TOP_OBJECT_NONE, 0U);
    result->exec_points[result->exec_point_count++] =
        top_make_point(previous.cell, target, TOP_ACTION_ROTATE,
                       TOP_EVENT_ROTATE, TOP_OBJECT_NONE, 0U);
    return top_add_segment(builder,
                           TOP_ACTION_ROTATE,
                           start,
                           2U,
                           duration,
                           TOP_EVENT_ROTATE,
                           TOP_OBJECT_NONE,
                           0U,
                           previous.cell,
                           target);
}

int top_path_append_identify(top_path_builder_t *builder,
                             top_object_kind_t kind,
                             uint8_t object_index,
                             uint8_t far_mode)
{
    top_result_t *result;
    top_path_point_t previous;
    uint32_t duration;
    uint16_t start;

    if (builder == NULL || builder->failed ||
        (kind != TOP_OBJECT_BOX && kind != TOP_OBJECT_TARGET))
    {
        return 0;
    }
    result = builder->result;
    previous = result->exec_points[result->exec_point_count - 1U];
    if (result->raw_point_count >= TOP_MAX_RAW_POINTS ||
        result->exec_point_count >= TOP_MAX_EXEC_POINTS)
    {
        builder->failed = 1U;
        return 0;
    }
    duration = far_mode ? builder->config->identify_far_ms :
                          builder->config->identify_near_ms;
    start = (uint16_t)(result->exec_point_count - 1U);
    result->raw_points[result->raw_point_count++] =
        top_make_point(previous.cell, previous.heading, TOP_ACTION_IDENTIFY,
                       TOP_EVENT_IDENTIFY, kind, object_index);
    result->exec_points[result->exec_point_count++] =
        top_make_point(previous.cell, previous.heading, TOP_ACTION_IDENTIFY,
                       TOP_EVENT_IDENTIFY, kind, object_index);
    return top_add_segment(builder,
                           TOP_ACTION_IDENTIFY,
                           start,
                           2U,
                           duration,
                           TOP_EVENT_IDENTIFY,
                           kind,
                           object_index,
                           previous.cell,
                           previous.heading);
}

int top_path_append_wait(top_path_builder_t *builder,
                         uint32_t duration_ms,
                         top_event_mask_t events,
                         top_cell_t effect_cell)
{
    top_result_t *result;
    top_path_point_t previous;
    uint16_t start;

    if (builder == NULL || builder->failed)
    {
        return 0;
    }
    result = builder->result;
    previous = result->exec_points[result->exec_point_count - 1U];
    if (result->raw_point_count >= TOP_MAX_RAW_POINTS ||
        result->exec_point_count >= TOP_MAX_EXEC_POINTS)
    {
        builder->failed = 1U;
        return 0;
    }
    start = (uint16_t)(result->exec_point_count - 1U);
    result->raw_points[result->raw_point_count++] =
        top_make_point(previous.cell, previous.heading, TOP_ACTION_WAIT,
                       events | TOP_EVENT_WAIT, TOP_OBJECT_NONE, 0U);
    result->exec_points[result->exec_point_count++] =
        top_make_point(previous.cell, previous.heading, TOP_ACTION_WAIT,
                       events | TOP_EVENT_WAIT, TOP_OBJECT_NONE, 0U);
    return top_add_segment(builder,
                           TOP_ACTION_WAIT,
                           start,
                           2U,
                           duration_ms,
                           events | TOP_EVENT_WAIT,
                           TOP_OBJECT_NONE,
                           0U,
                           effect_cell,
                           previous.heading);
}

int top_path_finish(top_path_builder_t *builder,
                    top_event_mask_t events)
{
    top_result_t *result;
    if (builder == NULL || builder->failed)
    {
        return 0;
    }
    result = builder->result;
    result->raw_points[result->raw_point_count - 1U].events |= events | TOP_EVENT_ROUTE_END;
    result->exec_points[result->exec_point_count - 1U].events |= events | TOP_EVENT_ROUTE_END;
    return 1;
}
