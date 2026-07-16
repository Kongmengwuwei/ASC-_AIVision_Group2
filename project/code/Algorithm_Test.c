#include "Algorithm_Test.h"
#include "data_handle.h"
#include "path.h"
#include "path_follow.h"
#include <string.h>

#if ALGORITHM_TEST_BOARD_BENCHMARK
#include "Game_logic.h"
#include "fsl_gpt.h"
#include <stdio.h>
#endif

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
    path_follow_hold_current_yaw();
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

#if ALGORITHM_TEST_BOARD_BENCHMARK

#define ALGORITHM_TEST_BENCHMARK_TIMER_HZ 1000000UL

volatile algorithm_test_benchmark_report_t g_algorithm_test_benchmark;

/* Large work buffers are static so planner tests do not consume the small C stack. */
static MapPresetConfig s_benchmark_config;
static path_map_snapshot_t s_benchmark_before;
static path_map_snapshot_t s_benchmark_after;
static Position s_benchmark_exec_path[MAX_CAR_PATH];
static Position s_benchmark_initial_boxes[MAX_BOXES];
static Position s_benchmark_initial_targets[MAX_TARGETS];

static void benchmark_timer_init(void)
{
    gpt_config_t config;

    GPT_GetDefaultConfig(&config);
    config.clockSource = kGPT_ClockSource_Osc;
    config.divider = 24U;
    config.enableFreeRun = true;
    config.enableRunInDbg = true;
    GPT_Init(GPT1, &config);
    GPT_StartTimer(GPT1);
}

static uint32 benchmark_timer_now_us(void)
{
    return GPT_GetCurrentTimerCount(GPT1);
}

static void benchmark_load_config(const MapPresetConfig *config, uint8 known_ids)
{
    size_t i;

    memset(obstacles, 0, sizeof(obstacles));
    memset(boxes, 0, sizeof(boxes));
    memset(targets, 0, sizeof(targets));
    memset(bombs, 0, sizeof(bombs));
    memset(car_path, 0, sizeof(car_path));

    Obstacles_count = config->obstacles_count;
    Boxes_count = config->boxes_count;
    Targets_count = config->targets_count;
    Bombs_count = config->bombs_count;
    Car_path_count = 0U;
    car = config->car_start;

    memcpy(obstacles, config->obstacles, Obstacles_count * sizeof(Position));
    memcpy(boxes, config->boxes, Boxes_count * sizeof(Position));
    memcpy(targets, config->targets, Targets_count * sizeof(Position));
    memcpy(bombs, config->bombs, Bombs_count * sizeof(Position));

    if (!known_ids)
    {
        for (i = 0U; i < Boxes_count; i++)
        {
            boxes[i].id = MAP_PRESET_UNKNOWN_ID;
        }
        for (i = 0U; i < Targets_count; i++)
        {
            targets[i].id = MAP_PRESET_UNKNOWN_ID;
        }
    }
}

/*
 * Continue push planning from the state produced by identification.
 * Recognition discovers IDs at runtime, while the benchmark obtains the same
 * IDs from the preset by cell. Walls/bombs/car remain exactly as identification
 * left them, including any optional bomb shortcut.
 */
static uint8 benchmark_restore_preset_ids_after_identify(const MapPresetConfig *config)
{
    size_t i;
    size_t j;

    if (config == NULL ||
        Boxes_count != config->boxes_count ||
        Targets_count != config->targets_count)
    {
        return 0U;
    }

    for (i = 0U; i < Boxes_count; i++)
    {
        uint8 found = 0U;
        for (j = 0U; j < config->boxes_count; j++)
        {
            if (boxes[i].row == config->boxes[j].row &&
                boxes[i].col == config->boxes[j].col)
            {
                boxes[i].id = config->boxes[j].id;
                found = 1U;
                break;
            }
        }
        if (!found)
        {
            return 0U;
        }
    }

    for (i = 0U; i < Targets_count; i++)
    {
        uint8 found = 0U;
        for (j = 0U; j < config->targets_count; j++)
        {
            if (targets[i].row == config->targets[j].row &&
                targets[i].col == config->targets[j].col)
            {
                targets[i].id = config->targets[j].id;
                found = 1U;
                break;
            }
        }
        if (!found)
        {
            return 0U;
        }
    }

    memset(car_path, 0, sizeof(car_path));
    Car_path_count = 0U;
    return 1U;
}

static void benchmark_take_snapshot(path_map_snapshot_t *snapshot)
{
    memset(snapshot, 0, sizeof(*snapshot));
    snapshot->obstacles_count = Obstacles_count;
    snapshot->boxes_count = Boxes_count;
    snapshot->targets_count = Targets_count;
    snapshot->bombs_count = Bombs_count;
    snapshot->car_pose_grid = car;
    memcpy(snapshot->obstacles_buf, obstacles, Obstacles_count * sizeof(Position));
    memcpy(snapshot->boxes_buf, boxes, Boxes_count * sizeof(Position));
    memcpy(snapshot->targets_buf, targets, Targets_count * sizeof(Position));
    memcpy(snapshot->bombs_buf, bombs, Bombs_count * sizeof(Position));
}

static uint8 benchmark_same_cells(const Position *left,
                                  const Position *right,
                                  size_t count)
{
    size_t i;

    for (i = 0U; i < count; i++)
    {
        if (left[i].row != right[i].row || left[i].col != right[i].col)
        {
            return 0U;
        }
    }
    return 1U;
}

static uint8 benchmark_validate_raw_path(void)
{
    size_t i;

    if (Car_path_count < 2U || Car_path_count > MAX_CAR_PATH)
    {
        return 0U;
    }
    for (i = 0U; i < Car_path_count; i++)
    {
        if (car_path[i].row >= MAP_ROWS || car_path[i].col >= MAP_COLS)
        {
            return 0U;
        }
        if (i > 0U)
        {
            int row_delta = (int)car_path[i].row - (int)car_path[i - 1U].row;
            int col_delta = (int)car_path[i].col - (int)car_path[i - 1U].col;
            if (row_delta < 0)
            {
                row_delta = -row_delta;
            }
            if (col_delta < 0)
            {
                col_delta = -col_delta;
            }
            if ((row_delta + col_delta) > 1)
            {
                return 0U;
            }
        }
    }
    return 1U;
}

static uint16 benchmark_count_events(const Position *path,
                                     size_t count,
                                     uint8 event_mask)
{
    size_t i;
    uint16 found = 0U;

    for (i = 0U; i < count; i++)
    {
        if ((path[i].id & event_mask) != 0U)
        {
            found++;
        }
    }
    return found;
}

static uint32 benchmark_hash_path(const Position *path, size_t count)
{
    size_t i;
    uint32 hash = 2166136261UL;

    for (i = 0U; i < count; i++)
    {
        hash = (hash ^ path[i].row) * 16777619UL;
        hash = (hash ^ path[i].col) * 16777619UL;
        hash = (hash ^ path[i].id) * 16777619UL;
    }
    hash = (hash ^ (uint32)count) * 16777619UL;
    return hash;
}

static uint8 benchmark_exec_preserves_events(size_t exec_steps)
{
    static const uint8 events[] = {
        IDENTIFICATION, BOMB_EXPLOSION, PUSH_START_POINT, PUSH_END_POINT
    };
    size_t i;

    if (exec_steps < 2U)
    {
        return 0U;
    }
    for (i = 0U; i < (sizeof(events) / sizeof(events[0])); i++)
    {
        if (benchmark_count_events(car_path, Car_path_count, events[i]) !=
            benchmark_count_events(s_benchmark_exec_path, exec_steps, events[i]))
        {
            return 0U;
        }
    }
    return 1U;
}

static void benchmark_add_timing(volatile algorithm_test_timing_t *timing,
                                 uint32 elapsed_us,
                                 uint8 repeat)
{
    if (repeat == 0U)
    {
        timing->min_us = elapsed_us;
        timing->max_us = elapsed_us;
        timing->avg_us = elapsed_us;
        return;
    }
    if (elapsed_us < timing->min_us)
    {
        timing->min_us = elapsed_us;
    }
    if (elapsed_us > timing->max_us)
    {
        timing->max_us = elapsed_us;
    }
    timing->avg_us += elapsed_us;
}

static void benchmark_finish_timing(volatile algorithm_test_timing_t *timing)
{
    timing->avg_us /= ALGORITHM_TEST_BENCHMARK_REPEATS;
}

void Algorithm_Test_RunBoardBenchmark(void)
{
    size_t map_index;
    uint32 benchmark_start;
    uint16 map_count;

    memset((void *)&g_algorithm_test_benchmark, 0, sizeof(g_algorithm_test_benchmark));
    g_algorithm_test_benchmark.magic = ALGORITHM_TEST_BENCHMARK_MAGIC;
    g_algorithm_test_benchmark.version = ALGORITHM_TEST_BENCHMARK_VERSION;
    g_algorithm_test_benchmark.timer_hz = ALGORITHM_TEST_BENCHMARK_TIMER_HZ;
    g_algorithm_test_benchmark.repeats = ALGORITHM_TEST_BENCHMARK_REPEATS;
    map_count = (Map_preset_count <= ALGORITHM_TEST_BENCHMARK_MAX_MAPS) ?
                    (uint16)Map_preset_count : ALGORITHM_TEST_BENCHMARK_MAX_MAPS;
    g_algorithm_test_benchmark.requested_maps = map_count;

    benchmark_timer_init();
    benchmark_start = benchmark_timer_now_us();

    for (map_index = 0U; map_index < map_count; map_index++)
    {
        volatile algorithm_test_map_result_t *result =
            &g_algorithm_test_benchmark.map[map_index];
        uint8 identify_valid = 1U;
        uint8 identify_exec_valid = 1U;
        uint8 push_valid = 1U;
        uint8 push_exec_valid = 1U;
        uint8 repeat_stable = 1U;
        uint8 repeat;

        result->map_index = (uint8)map_index;
        if (!Map_Preset_BuildConfig(map_index, &s_benchmark_config))
        {
            result->result_flags = ALGORITHM_TEST_RESULT_COMPLETE;
            g_algorithm_test_benchmark.failed_maps++;
            g_algorithm_test_benchmark.completed_maps++;
            continue;
        }

        result->result_flags = ALGORITHM_TEST_RESULT_MAP_BUILT;
        result->plan_mode = (uint8)s_benchmark_config.plan_mode;

        for (repeat = 0U; repeat < ALGORITHM_TEST_BENCHMARK_REPEATS; repeat++)
        {
            size_t initial_box_count;
            size_t initial_target_count;
            size_t exec_steps = 0U;
            uint16 raw_steps;
            uint16 event_count;
            uint16 bomb_event_count;
            uint16 push_start_count;
            uint16 push_end_count;
            uint32 start_us;
            uint32 elapsed_us;
            uint32 path_hash;
            uint8 current_valid;
            uint8 current_exec_valid;
            uint8 id_restore_ok;
            Position identify_end;
            Position push_start;

            benchmark_load_config(&s_benchmark_config, 0U);
            initial_box_count = Boxes_count;
            initial_target_count = Targets_count;
            memcpy(s_benchmark_initial_boxes, boxes, sizeof(s_benchmark_initial_boxes));
            memcpy(s_benchmark_initial_targets, targets, sizeof(s_benchmark_initial_targets));
            benchmark_take_snapshot(&s_benchmark_before);

            start_us = benchmark_timer_now_us();
            Plan_path_Identify();
            elapsed_us = benchmark_timer_now_us() - start_us;
            benchmark_add_timing(&result->identify_plan, elapsed_us, repeat);
            benchmark_take_snapshot(&s_benchmark_after);
            identify_end = car;

            raw_steps = (uint16)Car_path_count;
            event_count = benchmark_count_events(car_path, Car_path_count, IDENTIFICATION);
            bomb_event_count = benchmark_count_events(car_path, Car_path_count, BOMB_EXPLOSION);
            path_hash = benchmark_hash_path(car_path, Car_path_count);
            current_valid = (benchmark_validate_raw_path() &&
                             car_path[Car_path_count - 1U].row == identify_end.row &&
                             car_path[Car_path_count - 1U].col == identify_end.col &&
                             Boxes_count == initial_box_count &&
                             Targets_count == initial_target_count &&
                             benchmark_same_cells(boxes, s_benchmark_initial_boxes,
                                                  initial_box_count) &&
                             benchmark_same_cells(targets, s_benchmark_initial_targets,
                                                  initial_target_count));
            identify_valid = (uint8)(identify_valid && current_valid);

            start_us = benchmark_timer_now_us();
            current_exec_valid = path_build_exec_from_planner(car_path,
                                                               Car_path_count,
                                                               &s_benchmark_after,
                                                               &s_benchmark_before,
                                                               s_benchmark_exec_path,
                                                               MAX_CAR_PATH,
                                                               &exec_steps);
            elapsed_us = benchmark_timer_now_us() - start_us;
            benchmark_add_timing(&result->identify_exec_build, elapsed_us, repeat);
            current_exec_valid = (uint8)(current_exec_valid &&
                                         benchmark_exec_preserves_events(exec_steps));
            identify_exec_valid = (uint8)(identify_exec_valid && current_exec_valid);

            if (repeat == 0U)
            {
                result->identify_raw_steps = raw_steps;
                result->identify_exec_steps = (uint16)exec_steps;
                result->identify_events = event_count;
                result->identify_bomb_events = bomb_event_count;
                result->identify_path_hash = path_hash;
                result->identify_end_row = identify_end.row;
                result->identify_end_col = identify_end.col;
            }
            else if (result->identify_raw_steps != raw_steps ||
                     result->identify_exec_steps != (uint16)exec_steps ||
                     result->identify_events != event_count ||
                     result->identify_bomb_events != bomb_event_count ||
                     result->identify_end_row != identify_end.row ||
                     result->identify_end_col != identify_end.col ||
                     result->identify_path_hash != path_hash)
            {
                repeat_stable = 0U;
            }

            id_restore_ok = benchmark_restore_preset_ids_after_identify(&s_benchmark_config);
            push_start = car;
            benchmark_take_snapshot(&s_benchmark_before);
            start_us = benchmark_timer_now_us();
            if (s_benchmark_config.plan_mode == MAP_PRESET_PLAN_MODE1)
            {
                Plan_path_Mode1();
            }
            else
            {
                Plan_path_Mode2();
            }
            elapsed_us = benchmark_timer_now_us() - start_us;
            benchmark_add_timing(&result->push_plan, elapsed_us, repeat);
            benchmark_take_snapshot(&s_benchmark_after);

            raw_steps = (uint16)Car_path_count;
            push_start_count = benchmark_count_events(car_path, Car_path_count,
                                                       PUSH_START_POINT);
            push_end_count = benchmark_count_events(car_path, Car_path_count,
                                                     PUSH_END_POINT);
            bomb_event_count = benchmark_count_events(car_path, Car_path_count,
                                                       BOMB_EXPLOSION);
            path_hash = benchmark_hash_path(car_path, Car_path_count);
            current_valid = (id_restore_ok &&
                             benchmark_validate_raw_path() &&
                             car_path[0].row == push_start.row &&
                             car_path[0].col == push_start.col &&
                             Boxes_count == 0U && Targets_count == 0U &&
                             car_path[Car_path_count - 1U].col == 0U &&
                             (car_path[Car_path_count - 1U].row == 4U ||
                              car_path[Car_path_count - 1U].row == 5U));
            push_valid = (uint8)(push_valid && current_valid);

            exec_steps = 0U;
            start_us = benchmark_timer_now_us();
            current_exec_valid = path_build_exec_from_planner(car_path,
                                                               Car_path_count,
                                                               &s_benchmark_after,
                                                               &s_benchmark_before,
                                                               s_benchmark_exec_path,
                                                               MAX_CAR_PATH,
                                                               &exec_steps);
            elapsed_us = benchmark_timer_now_us() - start_us;
            benchmark_add_timing(&result->push_exec_build, elapsed_us, repeat);
            current_exec_valid = (uint8)(current_exec_valid &&
                                         benchmark_exec_preserves_events(exec_steps));
            push_exec_valid = (uint8)(push_exec_valid && current_exec_valid);

            if (repeat == 0U)
            {
                result->push_raw_steps = raw_steps;
                result->push_exec_steps = (uint16)exec_steps;
                result->push_start_events = push_start_count;
                result->push_end_events = push_end_count;
                result->push_bomb_events = bomb_event_count;
                result->push_path_hash = path_hash;
            }
            else if (result->push_raw_steps != raw_steps ||
                     result->push_exec_steps != (uint16)exec_steps ||
                     result->push_start_events != push_start_count ||
                     result->push_end_events != push_end_count ||
                     result->push_bomb_events != bomb_event_count ||
                     result->push_path_hash != path_hash)
            {
                repeat_stable = 0U;
            }
        }

        benchmark_finish_timing(&result->identify_plan);
        benchmark_finish_timing(&result->identify_exec_build);
        benchmark_finish_timing(&result->push_plan);
        benchmark_finish_timing(&result->push_exec_build);

        if (identify_valid)
        {
            result->result_flags |= ALGORITHM_TEST_RESULT_IDENTIFY_VALID;
        }
        if (identify_exec_valid)
        {
            result->result_flags |= ALGORITHM_TEST_RESULT_IDENTIFY_EXEC_VALID;
        }
        if (push_valid)
        {
            result->result_flags |= ALGORITHM_TEST_RESULT_PUSH_VALID;
        }
        if (push_exec_valid)
        {
            result->result_flags |= ALGORITHM_TEST_RESULT_PUSH_EXEC_VALID;
        }
        if (repeat_stable)
        {
            result->result_flags |= ALGORITHM_TEST_RESULT_REPEAT_STABLE;
        }
        result->result_flags |= ALGORITHM_TEST_RESULT_COMPLETE;

        if (result->result_flags != ALGORITHM_TEST_RESULT_ALL_OK)
        {
            g_algorithm_test_benchmark.failed_maps++;
        }
        g_algorithm_test_benchmark.completed_maps++;
    }

    g_algorithm_test_benchmark.total_us = benchmark_timer_now_us() - benchmark_start;

    printf("PLANBENCH magic=%08lX maps=%u failed=%u repeats=%u total_us=%lu\r\n",
           (unsigned long)g_algorithm_test_benchmark.magic,
           (unsigned)g_algorithm_test_benchmark.completed_maps,
           (unsigned)g_algorithm_test_benchmark.failed_maps,
           (unsigned)g_algorithm_test_benchmark.repeats,
           (unsigned long)g_algorithm_test_benchmark.total_us);
    for (map_index = 0U; map_index < map_count; map_index++)
    {
        volatile algorithm_test_map_result_t *result =
            &g_algorithm_test_benchmark.map[map_index];
        printf("map%u flags=%04X id=%u/%u %luus end=(%u,%u) push=%u/%u %luus\r\n",
               (unsigned)result->map_index,
               (unsigned)result->result_flags,
               (unsigned)result->identify_raw_steps,
               (unsigned)result->identify_exec_steps,
               (unsigned long)result->identify_plan.avg_us,
               (unsigned)result->identify_end_row,
               (unsigned)result->identify_end_col,
               (unsigned)result->push_raw_steps,
               (unsigned)result->push_exec_steps,
               (unsigned long)result->push_plan.avg_us);
    }
}

#if defined(__GNUC__) || defined(__ARMCC_VERSION)
__attribute__((noinline))
#endif
void Algorithm_Test_BenchmarkCompleteTrap(void)
{
    while (1)
    {
    }
}

#else

void Algorithm_Test_RunBoardBenchmark(void)
{
}

void Algorithm_Test_BenchmarkCompleteTrap(void)
{
}

#endif
