#ifndef TOP_PATH_H
#define TOP_PATH_H

#include "TopPlanner.h"
#include "TopGrid.h"

typedef struct
{
    top_result_t *result;
    const top_config_t *config;
    uint8_t failed;
} top_path_builder_t;

void top_path_builder_init(top_path_builder_t *builder,
                           top_result_t *result,
                           const top_config_t *config,
                           top_cell_t start,
                           top_heading_t heading);
int top_path_append_walk(top_path_builder_t *builder,
                         const top_cell_t *raw,
                         uint16_t raw_count,
                         const top_grid_blockers_t *blockers,
                         top_event_mask_t end_events);
int top_path_append_push(top_path_builder_t *builder,
                         top_action_t action,
                         top_object_kind_t kind,
                         uint8_t object_index,
                         top_cell_t car_destination,
                         top_cell_t effect_cell,
                         top_event_mask_t events,
                         uint32_t duration_ms);
int top_path_append_rotate(top_path_builder_t *builder,
                           top_heading_t target);
int top_path_append_identify(top_path_builder_t *builder,
                             top_object_kind_t kind,
                             uint8_t object_index,
                             uint8_t far_mode);
int top_path_append_wait(top_path_builder_t *builder,
                         uint32_t duration_ms,
                         top_event_mask_t events,
                         top_cell_t effect_cell);
int top_path_finish(top_path_builder_t *builder,
                    top_event_mask_t events);

#endif
