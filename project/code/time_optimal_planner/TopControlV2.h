#ifndef TOP_CONTROL_V2_H
#define TOP_CONTROL_V2_H

#include "TopPlanner.h"

typedef enum
{
    TOP_CONTROL_IDLE = 0,
    TOP_CONTROL_NEED_PLAN = 1,
    TOP_CONTROL_PLAN_READY = 2,
    TOP_CONTROL_EXECUTING = 3,
    TOP_CONTROL_WAIT_IDENTIFICATION = 4,
    TOP_CONTROL_FINISHED = 5,
    TOP_CONTROL_ERROR = 6
} top_control_stage_t;

typedef struct
{
    top_control_stage_t stage;
    top_status_t last_status;
    uint16_t segment_cursor;
    top_problem_t problem;
    top_config_t config;
    top_result_t plan;
} top_control_v2_t;

void top_control_v2_init(top_control_v2_t *control,
                         const top_config_t *config);
top_status_t top_control_v2_load_problem(top_control_v2_t *control,
                                         const top_problem_t *problem);
top_status_t top_control_v2_plan(top_control_v2_t *control);
const top_segment_t *top_control_v2_current_segment(const top_control_v2_t *control);
const top_path_point_t *top_control_v2_segment_points(const top_control_v2_t *control,
                                                      const top_segment_t *segment);
top_status_t top_control_v2_start_execution(top_control_v2_t *control);
top_status_t top_control_v2_segment_completed(top_control_v2_t *control);
top_status_t top_control_v2_submit_identification(top_control_v2_t *control,
                                                  uint8_t id);
void top_control_v2_fail(top_control_v2_t *control, top_status_t reason);

#endif
