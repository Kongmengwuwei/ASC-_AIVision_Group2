#include "TopPlanner.h"
#include "TopGrid.h"
#include "TopVerify.h"
#include "TopControlV2.h"

#include <assert.h>
#include <stdio.h>
#include <string.h>
#include <time.h>

static uint32_t host_now_ms(void *user)
{
    clock_t *start = (clock_t *)user;
    clock_t now = clock();
    return (uint32_t)(((now - *start) * 1000) / CLOCKS_PER_SEC);
}

static void configure_host(top_config_t *config, clock_t *start)
{
    top_config_default(config);
    *start = clock();
    config->now_ms = host_now_ms;
    config->now_user = start;
    config->planning_budget_ms = 2000U;
    config->max_expansions = 2000U;
}

static top_labeled_object_t object_at(int row, int col, uint8_t id, uint8_t known)
{
    top_labeled_object_t object;
    object.cell.row = (int8_t)row;
    object.cell.col = (int8_t)col;
    object.id = id;
    object.id_known = known;
    return object;
}

static void test_grid_optimizer(void)
{
    top_config_t config;
    top_wall_bits_t walls;
    top_grid_blockers_t blockers;
    top_cell_t raw[3] = {{0, 0}, {0, 1}, {1, 1}};
    top_cell_t output[3];
    uint16_t count = 0U;
    uint32_t duration = 0U;

    top_config_default(&config);
    memset(&walls, 0, sizeof(walls));
    memset(&blockers, 0, sizeof(blockers));
    blockers.walls = &walls;
    blockers.ignored_box = -1;
    blockers.ignored_bomb = -1;
    assert(top_grid_optimize_walk(raw, 3U, &blockers, &config,
                                  output, 3U, &count, &duration));
    assert(count == 2U);
    assert(duration == 566U);

    walls.word[top_cell_index((top_cell_t){0, 1}) >> 5] |=
        UINT32_C(1) << (top_cell_index((top_cell_t){0, 1}) & 31U);
    assert(!top_grid_line_clear((top_cell_t){0, 0},
                                (top_cell_t){1, 1},
                                &blockers));
}

static void test_simple_complete(void)
{
    top_problem_t problem;
    top_config_t config;
    top_result_t result;
    clock_t start;

    top_problem_clear(&problem);
    problem.match_mode = TOP_MODE_ID_MATCH;
    problem.car = (top_cell_t){5, 0};
    problem.heading = TOP_HEADING_RIGHT;
    problem.box_count = 1U;
    problem.target_count = 1U;
    problem.boxes[0] = object_at(5, 3, 2U, 1U);
    problem.targets[0] = object_at(5, 5, 2U, 1U);
    configure_host(&config, &start);

    {
        top_status_t status = top_plan(&problem, &config, &result);
        if (status != TOP_STATUS_OK)
        {
            printf("simple status=%s expanded=%u generated=%u timed=%u limit=%u\n",
                   top_status_string(status), result.expanded_nodes,
                   result.generated_nodes, result.timed_out, result.node_limit_hit);
            fflush(stdout);
        }
        assert(status == TOP_STATUS_OK);
    }
    assert(result.complete);
    assert(!result.needs_replan);
    assert(result.exec_point_count >= 2U);
    assert(result.end_state.car.col == 0);
    assert(result.end_state.car.row == 4 || result.end_state.car.row == 5);
    assert(result.end_state.heading == TOP_HEADING_RIGHT);
    assert(result.end_state.box_active_mask == 0U);
    assert(result.end_state.target_active_mask == 0U);
    assert(result.motion_ms == 3200U);
    {
        top_verify_report_t report;
        assert(top_verify_result(&problem, &config, &result, &report) == TOP_VERIFY_OK);
    }
}

static void test_unknown_identification(void)
{
    top_problem_t problem;
    top_config_t config;
    top_result_t result;
    clock_t start;

    top_problem_clear(&problem);
    problem.match_mode = TOP_MODE_ID_MATCH;
    problem.car = (top_cell_t){5, 0};
    problem.heading = TOP_HEADING_RIGHT;
    problem.box_count = 2U;
    problem.target_count = 2U;
    problem.boxes[0] = object_at(4, 4, TOP_ID_UNKNOWN, 0U);
    problem.boxes[1] = object_at(7, 7, TOP_ID_UNKNOWN, 0U);
    problem.targets[0] = object_at(4, 9, TOP_ID_UNKNOWN, 0U);
    problem.targets[1] = object_at(7, 10, TOP_ID_UNKNOWN, 0U);
    configure_host(&config, &start);

    assert(top_plan(&problem, &config, &result) == TOP_STATUS_PARTIAL_REPLAN);
    assert(result.needs_replan);
    assert(result.requested_identify_kind == TOP_OBJECT_BOX ||
           result.requested_identify_kind == TOP_OBJECT_TARGET);
    assert(result.motion_ms >= 200U);
    {
        top_verify_report_t report;
        assert(top_verify_result(&problem, &config, &result, &report) == TOP_VERIFY_OK);
    }
}

static void test_bomb_required(void)
{
    top_problem_t problem;
    top_config_t config;
    top_result_t result;
    top_verify_report_t report;
    clock_t start;
    int row;

    top_problem_clear(&problem);
    problem.match_mode = TOP_MODE_FREE_MATCH;
    problem.car = (top_cell_t){7, 4};
    problem.heading = TOP_HEADING_RIGHT;
    problem.box_count = 1U;
    problem.target_count = 1U;
    problem.bomb_count = 1U;
    problem.boxes[0] = object_at(5, 2, 0U, 1U);
    problem.targets[0] = object_at(5, 7, 0U, 1U);
    problem.bombs[0] = (top_cell_t){6, 4};
    for (row = 0; row < (int)TOP_ROWS; ++row)
    {
        if (row != 6 && row != 7)
        {
            assert(top_problem_set_wall(&problem, row, 4, 1));
        }
    }
    assert(top_problem_set_wall(&problem, 7, 3, 1));
    assert(top_problem_set_wall(&problem, 7, 5, 1));
    configure_host(&config, &start);
    config.max_expansions = 50U;
    config.max_nodes = 2048U;

    {
        top_status_t status = top_plan(&problem, &config, &result);
        if (status != TOP_STATUS_OK)
        {
            printf("bomb status=%s expanded=%u generated=%u timed=%u limit=%u\n",
                   top_status_string(status), result.expanded_nodes,
                   result.generated_nodes, result.timed_out, result.node_limit_hit);
            fflush(stdout);
        }
        assert(status == TOP_STATUS_OK);
    }
    assert(result.complete);
    assert((result.end_state.bomb_active_mask & 1U) == 0U);
    assert(top_verify_result(&problem, &config, &result, &report) == TOP_VERIFY_OK);
}

static void test_last_pair_inference(void)
{
    top_problem_t problem;
    top_config_t config;
    top_result_t result;
    clock_t start;

    top_problem_clear(&problem);
    problem.match_mode = TOP_MODE_ID_MATCH;
    problem.car = (top_cell_t){5, 0};
    problem.box_count = 1U;
    problem.target_count = 1U;
    problem.boxes[0] = object_at(5, 2, TOP_ID_UNKNOWN, 0U);
    problem.targets[0] = object_at(5, 4, TOP_ID_UNKNOWN, 0U);
    configure_host(&config, &start);
    assert(top_plan(&problem, &config, &result) == TOP_STATUS_OK);
    assert(result.complete);
    assert(result.requested_identify_kind == TOP_OBJECT_NONE);
}

static void test_repeated_ids(void)
{
    top_problem_t problem;
    top_config_t config;
    top_result_t result;
    top_verify_report_t report;
    clock_t start;

    top_problem_clear(&problem);
    problem.match_mode = TOP_MODE_ID_MATCH;
    problem.car = (top_cell_t){5, 0};
    problem.box_count = 2U;
    problem.target_count = 2U;
    problem.boxes[0] = object_at(4, 3, 7U, 1U);
    problem.boxes[1] = object_at(6, 3, 7U, 1U);
    problem.targets[0] = object_at(4, 6, 7U, 1U);
    problem.targets[1] = object_at(6, 6, 7U, 1U);
    configure_host(&config, &start);
    assert(top_plan(&problem, &config, &result) == TOP_STATUS_OK);
    assert(result.complete);
    assert(top_verify_result(&problem, &config, &result, &report) == TOP_VERIFY_OK);
}

static void test_interleaved_delivery(void)
{
    top_problem_t problem;
    top_config_t config;
    top_result_t result;
    clock_t start;

    top_problem_clear(&problem);
    problem.match_mode = TOP_MODE_ID_MATCH;
    problem.car = (top_cell_t){5, 0};
    problem.box_count = 3U;
    problem.target_count = 3U;
    problem.boxes[0] = object_at(5, 2, 1U, 1U);
    problem.targets[0] = object_at(5, 3, 1U, 1U);
    problem.boxes[1] = object_at(8, 9, TOP_ID_UNKNOWN, 0U);
    problem.targets[1] = object_at(8, 12, TOP_ID_UNKNOWN, 0U);
    problem.boxes[2] = object_at(2, 9, TOP_ID_UNKNOWN, 0U);
    problem.targets[2] = object_at(2, 12, TOP_ID_UNKNOWN, 0U);
    configure_host(&config, &start);
    {
        top_status_t status = top_plan(&problem, &config, &result);
        if (status != TOP_STATUS_PARTIAL_REPLAN ||
            result.requested_identify_kind != TOP_OBJECT_NONE)
        {
            printf("interleave status=%s request=%d mask=%u motion=%u\n",
                   top_status_string(status),
                   (int)result.requested_identify_kind,
                   result.end_state.box_active_mask,
                   (unsigned)result.motion_ms);
            fflush(stdout);
        }
        assert(status == TOP_STATUS_PARTIAL_REPLAN);
    }
    assert(result.needs_replan);
    assert(result.requested_identify_kind == TOP_OBJECT_NONE);
    assert((result.end_state.box_active_mask & 1U) == 0U);
}

static void test_control_flow(void)
{
    top_problem_t problem;
    top_config_t config;
    top_control_v2_t control;
    clock_t start;

    top_problem_clear(&problem);
    problem.match_mode = TOP_MODE_FREE_MATCH;
    problem.car = (top_cell_t){5, 0};
    problem.box_count = 1U;
    problem.target_count = 1U;
    problem.boxes[0] = object_at(5, 2, 0U, 1U);
    problem.targets[0] = object_at(5, 4, 0U, 1U);
    configure_host(&config, &start);
    top_control_v2_init(&control, &config);
    assert(top_control_v2_load_problem(&control, &problem) == TOP_STATUS_OK);
    assert(top_control_v2_plan(&control) == TOP_STATUS_OK);
    assert(top_control_v2_start_execution(&control) == TOP_STATUS_OK);
    while (control.stage == TOP_CONTROL_EXECUTING)
    {
        const top_segment_t *segment = top_control_v2_current_segment(&control);
        assert(segment != NULL);
        assert(top_control_v2_segment_points(&control, segment) != NULL);
        (void)top_control_v2_segment_completed(&control);
    }
    assert(control.stage == TOP_CONTROL_FINISHED);
}

int main(void)
{
    puts("grid"); fflush(stdout);
    test_grid_optimizer();
    puts("simple"); fflush(stdout);
    test_simple_complete();
    puts("identify"); fflush(stdout);
    test_unknown_identification();
    puts("bomb"); fflush(stdout);
    test_bomb_required();
    puts("inference"); fflush(stdout);
    test_last_pair_inference();
    puts("repeat"); fflush(stdout);
    test_repeated_ids();
    puts("interleave"); fflush(stdout);
    test_interleaved_delivery();
    puts("control"); fflush(stdout);
    test_control_flow();
    puts("top planner smoke tests passed");
    return 0;
}
