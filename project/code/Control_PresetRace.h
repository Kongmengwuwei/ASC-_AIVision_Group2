#ifndef _CONTROL_PRESET_RACE_H_
#define _CONTROL_PRESET_RACE_H_

#include "Map_Path_Data.h"
#include "Preset_Route_Data.h"
#include "zf_common_typedef.h"

#define CONTROL_PRESET_ROUTE_COUNT PRESET_ROUTE_COUNT
#define CONTROL_PRESET_START_ROUTE_MIN 1U
#define CONTROL_PRESET_START_ROUTE_MAX PRESET_ROUTE_COUNT

typedef enum
{
    CONTROL_STAGE_IDLE = 0,
    CONTROL_STAGE_PRESTART_MOVE = 1,
    CONTROL_STAGE_SNAP_TO_LANDING = 20,
    CONTROL_STAGE_PRE_IDENTIFY_PAUSE = 21,
    CONTROL_STAGE_LOAD_SEGMENT = 22,
    CONTROL_STAGE_RUN_SEGMENT = 23,
    CONTROL_STAGE_ROTATE_AT_POINT = 24,
    CONTROL_STAGE_PAUSE_AT_POINT = 25,
    CONTROL_STAGE_RETURN_YAW_AT_END = 26,
    CONTROL_STAGE_MAP_END_WAIT = 27,
    CONTROL_STAGE_NEXT_ROUTE = 28,
    CONTROL_STAGE_FINISHED = 29,
    CONTROL_STAGE_RETURN_TO_START_ZONE = 30,
    CONTROL_STAGE_ERROR = 99
} control_stage_t;

extern control_stage_t g_control_stage;

void control_init(void);
void control_process(void);
void control_restart(void);
void control_set_start_enabled(uint8 enabled);
uint8 control_get_start_enabled(void);
control_stage_t control_get_stage(void);
const Position *control_get_exec_path(size_t *steps);
uint8 control_is_path_plan_paused(void);

void control_set_start_route_index(uint8 route_index);
uint8 control_get_start_route_index(void);
uint8 control_get_current_route_index(void);

/* Kept for legacy call sites. The preset race flow fixes these in code. */
void control_set_prestart_depart_dir(uint8 dir);
uint8 control_get_prestart_depart_dir(void);
void control_set_diagonal_path_enabled(uint8 enabled);
uint8 control_get_diagonal_path_enabled(void);

#endif
