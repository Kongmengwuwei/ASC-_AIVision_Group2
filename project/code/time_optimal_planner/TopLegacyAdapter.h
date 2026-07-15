#ifndef TOP_LEGACY_ADAPTER_H
#define TOP_LEGACY_ADAPTER_H

#include "TopPlanner.h"
#include "Map_Path_Data.h"

/* Builds a new planner problem from the unchanged legacy globals.  Explicit
 * known arrays are necessary because ID 0 is valid in the competition. */
top_status_t top_legacy_import_globals(top_match_mode_t mode,
                                       const uint8 *box_id_known,
                                       const uint8 *target_id_known,
                                       top_problem_t *problem);

/* Lossless for geometry and legacy events; richer new events remain available
 * in top_result_t.  Duplicate stationary points are merged for path_follow. */
top_status_t top_legacy_export_exec_path(const top_result_t *result,
                                         Position *legacy_path,
                                         size_t capacity,
                                         size_t *count,
                                         uint8 remap_for_path_follow);

#endif
