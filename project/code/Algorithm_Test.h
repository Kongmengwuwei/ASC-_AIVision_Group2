#ifndef _ALGORITHM_TEST_H
#define _ALGORITHM_TEST_H

#include "Map_Path_Data.h"

#ifndef ALGORITHM_TEST_PRESET_INDEX 
#define ALGORITHM_TEST_PRESET_INDEX 0U
#endif

/*
 * RT1064 board-side planner benchmark. Keep disabled in normal vehicle
 * firmware; the benchmark build enables it from the compiler command line.
 */
#ifndef ALGORITHM_TEST_BOARD_BENCHMARK
#define ALGORITHM_TEST_BOARD_BENCHMARK 0U
#endif

#define ALGORITHM_TEST_BENCHMARK_VERSION  0x00010001UL
#define ALGORITHM_TEST_BENCHMARK_MAGIC    0x504C414EUL /* "PLAN" */
#define ALGORITHM_TEST_BENCHMARK_REPEATS  3U
#define ALGORITHM_TEST_BENCHMARK_MAX_MAPS 10U

#define ALGORITHM_TEST_RESULT_MAP_BUILT           0x0001U
#define ALGORITHM_TEST_RESULT_IDENTIFY_VALID      0x0002U
#define ALGORITHM_TEST_RESULT_IDENTIFY_EXEC_VALID 0x0004U
#define ALGORITHM_TEST_RESULT_PUSH_VALID          0x0008U
#define ALGORITHM_TEST_RESULT_PUSH_EXEC_VALID     0x0010U
#define ALGORITHM_TEST_RESULT_REPEAT_STABLE       0x0020U
#define ALGORITHM_TEST_RESULT_COMPLETE            0x8000U
#define ALGORITHM_TEST_RESULT_ALL_OK              0x803FU

typedef struct
{
    uint32 min_us;
    uint32 avg_us;
    uint32 max_us;
} algorithm_test_timing_t;

typedef struct
{
    uint16 result_flags;
    uint8 map_index;
    uint8 plan_mode;
    uint16 identify_raw_steps;
    uint16 identify_exec_steps;
    uint16 identify_events;
    uint16 identify_bomb_events;
    uint16 push_raw_steps;
    uint16 push_exec_steps;
    uint16 push_start_events;
    uint16 push_end_events;
    uint16 push_bomb_events;
    uint8 identify_end_row;
    uint8 identify_end_col;
    uint32 identify_path_hash;
    uint32 push_path_hash;
    algorithm_test_timing_t identify_plan;
    algorithm_test_timing_t identify_exec_build;
    algorithm_test_timing_t push_plan;
    algorithm_test_timing_t push_exec_build;
} algorithm_test_map_result_t;

typedef struct
{
    uint32 magic;
    uint32 version;
    uint32 timer_hz;
    uint16 requested_maps;
    uint16 completed_maps;
    uint16 failed_maps;
    uint16 repeats;
    uint32 total_us;
    algorithm_test_map_result_t map[ALGORITHM_TEST_BENCHMARK_MAX_MAPS];
} algorithm_test_benchmark_report_t;

extern volatile algorithm_test_benchmark_report_t g_algorithm_test_benchmark;

void Algorithm_Test_PresetInput_Init(size_t preset_index);
void Algorithm_Test_PresetInput_SetEnabled(uint8 enabled, size_t preset_index);
uint8 Algorithm_Test_PresetInput_IsEnabled(void);
const MapPresetConfig *Algorithm_Test_PresetInput_GetActive(void);
map_preset_plan_mode_t Algorithm_Test_PresetInput_GetPlanMode(void);
uint8 Algorithm_Test_PresetInput_ProvideMapFrame(void);
uint8 Algorithm_Test_PresetInput_ProvideCarPoseFrame(void);
uint8 Algorithm_Test_PresetInput_GetObjectId(Position object_pos, uint8 is_target, uint8 *id_out);
void Algorithm_Test_RunBoardBenchmark(void);
void Algorithm_Test_BenchmarkCompleteTrap(void);

#endif
