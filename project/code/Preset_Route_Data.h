#ifndef _PRESET_ROUTE_DATA_H_
#define _PRESET_ROUTE_DATA_H_

#include "Map_Path_Data.h"
#include "zf_common_typedef.h"
#include <stddef.h>

#define PRESET_ROUTE_COUNT 3U
#define PRESET_DEFAULT_PRE_IDENTIFY_PAUSE_MS 2000U
#define PRESET_DEFAULT_POINT_PAUSE_MS 3200U
#define PRESET_DEFAULT_MAP_END_WAIT_MS 5000U

typedef enum
{
    PRESET_ACTION_NONE = 0U,
    PRESET_ACTION_FAKE_IDENTIFY = 1U,
    PRESET_ACTION_STOP_ONLY = 2U,
    PRESET_ACTION_ROUTE_END = 3U
} preset_action_t;

typedef enum
{
    PRESET_FACE_KEEP = 0U,
    PRESET_FACE_MAP_RIGHT = 1U,
    PRESET_FACE_MAP_UP = 2U,
    PRESET_FACE_MAP_LEFT = 3U,
    PRESET_FACE_MAP_DOWN = 4U
} preset_face_t;

typedef struct
{
    uint8 row;
    uint8 col;
    uint8 action;
    uint8 face;
    uint16 pause_ms;
} preset_point_t;

typedef struct
{
    const preset_point_t *points;
    size_t count;
    Position landing_grid;
    uint16 pre_identify_pause_ms;
    uint16 map_end_wait_ms;
    uint8 reset_yaw_at_end;
} preset_route_t;

extern const preset_route_t g_preset_routes[PRESET_ROUTE_COUNT];

#endif
