#ifndef TOP_VERIFY_H
#define TOP_VERIFY_H

#include "TopPlanner.h"

typedef enum
{
    TOP_VERIFY_OK = 0,
    TOP_VERIFY_BAD_INPUT,
    TOP_VERIFY_BAD_POINT_RANGE,
    TOP_VERIFY_DISCONTINUOUS,
    TOP_VERIFY_COLLISION,
    TOP_VERIFY_BAD_DURATION,
    TOP_VERIFY_BAD_PUSH,
    TOP_VERIFY_BAD_EXPLOSION,
    TOP_VERIFY_BAD_WAIT,
    TOP_VERIFY_BAD_ROTATION,
    TOP_VERIFY_BAD_IDENTIFICATION,
    TOP_VERIFY_BAD_COMPLETION,
    TOP_VERIFY_END_STATE_MISMATCH
} top_verify_code_t;

typedef struct
{
    top_verify_code_t code;
    uint16_t segment_index;
    uint32_t recomputed_motion_ms;
} top_verify_report_t;

top_verify_code_t top_verify_result(const top_problem_t *problem,
                                    const top_config_t *config,
                                    const top_result_t *result,
                                    top_verify_report_t *report);
const char *top_verify_string(top_verify_code_t code);

#endif
