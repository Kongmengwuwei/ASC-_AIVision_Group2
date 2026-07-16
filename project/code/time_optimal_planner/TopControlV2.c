#include "TopControlV2.h"

#include <string.h>

void top_control_v2_init(top_control_v2_t *control,
                         const top_config_t *config)
{
    if (control == NULL)
    {
        return;
    }
    memset(control, 0, sizeof(*control));
    if (config != NULL)
    {
        control->config = *config;
    }
    else
    {
        top_config_default(&control->config);
    }
    control->stage = TOP_CONTROL_IDLE;
    control->last_status = TOP_STATUS_OK;
}

top_status_t top_control_v2_load_problem(top_control_v2_t *control,
                                         const top_problem_t *problem)
{
    top_status_t status;
    if (control == NULL || problem == NULL)
    {
        return TOP_STATUS_INVALID_INPUT;
    }
    status = top_problem_validate(problem);
    if (status != TOP_STATUS_OK)
    {
        control->stage = TOP_CONTROL_ERROR;
        control->last_status = status;
        return status;
    }
    control->problem = *problem;
    memset(&control->plan, 0, sizeof(control->plan));
    control->segment_cursor = 0U;
    control->stage = TOP_CONTROL_NEED_PLAN;
    control->last_status = TOP_STATUS_OK;
    return TOP_STATUS_OK;
}

top_status_t top_control_v2_plan(top_control_v2_t *control)
{
    top_status_t status;
    if (control == NULL || control->stage != TOP_CONTROL_NEED_PLAN)
    {
        return TOP_STATUS_INVALID_INPUT;
    }
    status = top_plan(&control->problem, &control->config, &control->plan);
    control->last_status = status;
    control->segment_cursor = 0U;
    if (status == TOP_STATUS_OK || status == TOP_STATUS_PARTIAL_REPLAN)
    {
        control->stage = TOP_CONTROL_PLAN_READY;
    }
    else
    {
        control->stage = TOP_CONTROL_ERROR;
    }
    return status;
}

const top_segment_t *top_control_v2_current_segment(const top_control_v2_t *control)
{
    if (control == NULL ||
        (control->stage != TOP_CONTROL_PLAN_READY &&
         control->stage != TOP_CONTROL_EXECUTING) ||
        control->segment_cursor >= control->plan.segment_count)
    {
        return NULL;
    }
    return &control->plan.segments[control->segment_cursor];
}

const top_path_point_t *top_control_v2_segment_points(const top_control_v2_t *control,
                                                      const top_segment_t *segment)
{
    if (control == NULL || segment == NULL || segment->point_count == 0U ||
        (uint32_t)segment->first_point + segment->point_count >
            control->plan.exec_point_count)
    {
        return NULL;
    }
    return &control->plan.exec_points[segment->first_point];
}

top_status_t top_control_v2_start_execution(top_control_v2_t *control)
{
    if (control == NULL || control->stage != TOP_CONTROL_PLAN_READY ||
        control->plan.segment_count == 0U)
    {
        return TOP_STATUS_INVALID_INPUT;
    }
    control->stage = TOP_CONTROL_EXECUTING;
    return TOP_STATUS_OK;
}

top_status_t top_control_v2_segment_completed(top_control_v2_t *control)
{
    top_status_t status;
    if (control == NULL || control->stage != TOP_CONTROL_EXECUTING ||
        control->segment_cursor >= control->plan.segment_count)
    {
        return TOP_STATUS_INVALID_INPUT;
    }
    ++control->segment_cursor;
    if (control->segment_cursor < control->plan.segment_count)
    {
        return TOP_STATUS_OK;
    }
    status = top_problem_apply_result(&control->problem, &control->plan);
    if (status != TOP_STATUS_OK)
    {
        control->stage = TOP_CONTROL_ERROR;
        control->last_status = status;
        return status;
    }
    if (control->plan.requested_identify_kind == TOP_OBJECT_BOX ||
        control->plan.requested_identify_kind == TOP_OBJECT_TARGET)
    {
        control->stage = TOP_CONTROL_WAIT_IDENTIFICATION;
        return TOP_STATUS_PARTIAL_REPLAN;
    }
    if (control->plan.complete)
    {
        control->stage = TOP_CONTROL_FINISHED;
        return TOP_STATUS_OK;
    }
    /* Strict staged planning only emits partial plans for identification. */
    control->stage = TOP_CONTROL_ERROR;
    control->last_status = TOP_STATUS_INTERNAL_ERROR;
    return TOP_STATUS_INTERNAL_ERROR;
}

top_status_t top_control_v2_submit_identification(top_control_v2_t *control,
                                                  uint8_t id)
{
    top_status_t status;
    if (control == NULL || control->stage != TOP_CONTROL_WAIT_IDENTIFICATION)
    {
        return TOP_STATUS_INVALID_INPUT;
    }
    status = top_problem_set_object_id(&control->problem,
                                       control->plan.requested_identify_kind,
                                       control->plan.requested_identify_index,
                                       id);
    if (status != TOP_STATUS_OK)
    {
        control->stage = TOP_CONTROL_ERROR;
        control->last_status = status;
        return status;
    }
    control->stage = TOP_CONTROL_NEED_PLAN;
    control->last_status = TOP_STATUS_OK;
    return TOP_STATUS_OK;
}

void top_control_v2_fail(top_control_v2_t *control, top_status_t reason)
{
    if (control == NULL)
    {
        return;
    }
    control->stage = TOP_CONTROL_ERROR;
    control->last_status = reason;
}
