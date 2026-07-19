#include "Algorithm_Test.h"
#include "data_handle.h"
#include "path.h"
#include "path_follow.h"
#include <string.h>

static uint8 s_preset_input_enabled = 0U;
static MapPresetConfig s_active_preset;

static uint8 round_grid_value(float value, uint8 max_value)
{
    int32 rounded = (value >= 0.0f) ? (int32)(value + 0.5f) : (int32)(value - 0.5f);

    if (rounded < 0)
    {
        rounded = 0;
    }
    if (rounded > (int32)max_value)
    {
        rounded = (int32)max_value;
    }
    return (uint8)rounded;
}

static const MapPresetConfig *get_active_preset(void)
{
    if (!s_preset_input_enabled || Map_preset_count == 0U)
    {
        return 0;
    }
    return &s_active_preset;
}

static void copy_preset_objects_to_globals(const MapPresetConfig *preset, uint8 reset_car_to_start)
{
    if (preset == 0)
    {
        return;
    }

    memset(obstacles, 0, sizeof(obstacles));
    memset(boxes, 0, sizeof(boxes));
    memset(targets, 0, sizeof(targets));
    memset(bombs, 0, sizeof(bombs));

    Obstacles_count = (preset->obstacles_count <= MAX_OBSTACLES) ? preset->obstacles_count : MAX_OBSTACLES;
    Boxes_count = (preset->boxes_count <= MAX_BOXES) ? preset->boxes_count : MAX_BOXES;
    Targets_count = (preset->targets_count <= MAX_TARGETS) ? preset->targets_count : MAX_TARGETS;
    Bombs_count = (preset->bombs_count <= MAX_BOMBS) ? preset->bombs_count : MAX_BOMBS;

    if (Obstacles_count > 0U)
    {
        memcpy(obstacles, preset->obstacles, Obstacles_count * sizeof(Position));
    }
    if (Boxes_count > 0U)
    {
        memcpy(boxes, preset->boxes, Boxes_count * sizeof(Position));
    }
    if (Targets_count > 0U)
    {
        memcpy(targets, preset->targets, Targets_count * sizeof(Position));
    }
    if (Bombs_count > 0U)
    {
        memcpy(bombs, preset->bombs, Bombs_count * sizeof(Position));
    }
    if (reset_car_to_start)
    {
        car = preset->car_start;
    }
}

static void fill_map_data_from_preset(const MapPresetConfig *preset)
{
    size_t row;
    size_t col;
    size_t i;

    for (row = 0U; row < MAP_ROWS; row++)
    {
        for (col = 0U; col < MAP_COLS; col++)
        {
            map_data[row][col] = MAP_SYMBOL_EMPTY;
        }
    }

    for (i = 0U; i < preset->obstacles_count && i < MAX_OBSTACLES; i++)
    {
        if (preset->obstacles[i].row < MAP_ROWS && preset->obstacles[i].col < MAP_COLS)
        {
            map_data[preset->obstacles[i].row][preset->obstacles[i].col] = MAP_SYMBOL_OBSTACLE;
        }
    }
    for (i = 0U; i < preset->targets_count && i < MAX_TARGETS; i++)
    {
        if (preset->targets[i].row < MAP_ROWS && preset->targets[i].col < MAP_COLS)
        {
            map_data[preset->targets[i].row][preset->targets[i].col] = MAP_SYMBOL_TARGET;
        }
    }
    for (i = 0U; i < preset->boxes_count && i < MAX_BOXES; i++)
    {
        if (preset->boxes[i].row < MAP_ROWS && preset->boxes[i].col < MAP_COLS)
        {
            map_data[preset->boxes[i].row][preset->boxes[i].col] = MAP_SYMBOL_BOX;
        }
    }
    for (i = 0U; i < preset->bombs_count && i < MAX_BOMBS; i++)
    {
        if (preset->bombs[i].row < MAP_ROWS && preset->bombs[i].col < MAP_COLS)
        {
            map_data[preset->bombs[i].row][preset->bombs[i].col] = MAP_SYMBOL_BOMB;
        }
    }
    if (car.row < MAP_ROWS && car.col < MAP_COLS)
    {
        map_data[car.row][car.col] = MAP_SYMBOL_CAR;
    }
}

void Algorithm_Test_PresetInput_Init(size_t preset_index)
{
    const MapPresetConfig *preset;
    Position exec_start;

    if (Map_preset_count == 0U)
    {
        s_preset_input_enabled = 0U;
        uart_data_processing_enabled = true;
        vision_data_processing_enabled = true;
        return;
    }

    if (preset_index >= Map_preset_count)
    {
        preset_index = 0U;
    }
    if (!Map_Preset_BuildConfig(preset_index, &s_active_preset))
    {
        s_preset_input_enabled = 0U;
        uart_data_processing_enabled = true;
        vision_data_processing_enabled = true;
        return;
    }
    s_preset_input_enabled = 1U;
    uart_data_processing_enabled = false;
    vision_data_processing_enabled = false;

    preset = get_active_preset();
    copy_preset_objects_to_globals(preset, 1U);

    exec_start = preset->car_start;
    path_remap_exec_point(&exec_start);
    path_follow_reset_pose((float)exec_start.row * GRID_SIZE_M,
                           (float)exec_start.col * GRID_SIZE_M,
                           preset->car_yaw_deg);
    path_follow_set_target_yaw(0.0f);
    (void)Algorithm_Test_PresetInput_ProvideCarPoseFrame();
}

void Algorithm_Test_PresetInput_SetEnabled(uint8 enabled, size_t preset_index)
{
    if (enabled)
    {
        Algorithm_Test_PresetInput_Init(preset_index);
        return;
    }

    s_preset_input_enabled = 0U;
    uart_data_processing_enabled = true;
    vision_data_processing_enabled = true;
}

uint8 Algorithm_Test_PresetInput_IsEnabled(void)
{
    return s_preset_input_enabled;
}

const MapPresetConfig *Algorithm_Test_PresetInput_GetActive(void)
{
    return get_active_preset();
}

map_preset_plan_mode_t Algorithm_Test_PresetInput_GetPlanMode(void)
{
    const MapPresetConfig *preset = get_active_preset();
    if (preset == 0)
    {
        return MAP_PRESET_PLAN_MODE1;
    }
    return preset->plan_mode;
}

uint8 Algorithm_Test_PresetInput_ProvideMapFrame(void)
{
    const MapPresetConfig *preset = get_active_preset();
    if (preset == 0)
    {
        return 0U;
    }

    copy_preset_objects_to_globals(preset, 0U);
    fill_map_data_from_preset(preset);
    map_data_ready = true;
    map_data_updated = true;
    map_frame_count++;
    return 1U;
}

uint8 Algorithm_Test_PresetInput_ProvideCarPoseFrame(void)
{
    path_follow_status_t st;
    Position map_point;
    float row_f;
    float col_f;
    float src_col_f;

    if (!s_preset_input_enabled)
    {
        return 0U;
    }

    memset(&st, 0, sizeof(st));
    path_follow_get_status(&st);
    row_f = st.x_m / GRID_SIZE_M;
    col_f = st.y_m / GRID_SIZE_M;
    src_col_f = col_f;

#if PATH_COORD_FLIP_VERTICAL
    src_col_f = (float)(MAP_ROWS - 1U) - col_f;
#endif

#if PATH_COORD_TRANSPOSE_COMPENSATE
    car_pose.x = row_f;
    car_pose.y = src_col_f;
#else
    car_pose.x = src_col_f;
    car_pose.y = row_f;
#endif
    car_pose.yaw = st.yaw_deg;
    car_pose.x_raw = (int32)(car_pose.x * 100.0f);
    car_pose.y_raw = (int32)(car_pose.y * 100.0f);
    car_pose.yaw_raw = (int32)(car_pose.yaw * 100.0f);

    map_point.row = round_grid_value(row_f, (uint8)(MAP_ROWS - 1U));
    map_point.col = round_grid_value(col_f, (uint8)(MAP_COLS - 1U));
    map_point.id = 0U;
    path_inverse_remap_exec_point(&map_point);
    car = map_point;

    car_pose_ready = true;
    car_pose_updated = true;
    car_frame_count++;
    return 1U;
}

uint8 Algorithm_Test_PresetInput_GetObjectId(Position object_pos, uint8 is_target, uint8 *id_out)
{
    const MapPresetConfig *preset = get_active_preset();
    const Position *list = 0;
    size_t count = 0U;
    size_t i;

    if (preset == 0 || id_out == 0)
    {
        return 0U;
    }

    if (is_target)
    {
        list = preset->targets;
        count = preset->targets_count;
    }
    else
    {
        list = preset->boxes;
        count = preset->boxes_count;
    }

    for (i = 0U; i < count; i++)
    {
        if (list[i].row == object_pos.row && list[i].col == object_pos.col)
        {
            *id_out = list[i].id;
            return 1U;
        }
    }
    return 0U;
}
