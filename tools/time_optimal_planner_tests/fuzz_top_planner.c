#include "TopPlanner.h"
#include "TopVerify.h"

#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

static uint32_t fuzz_now(void *user)
{
    clock_t origin = *(clock_t *)user;
    return (uint32_t)(((clock() - origin) * 1000) / CLOCKS_PER_SEC);
}

static top_cell_t random_cell(void)
{
    top_cell_t cell;
    cell.row = (int8_t)(1 + rand() % ((int)TOP_ROWS - 2));
    cell.col = (int8_t)(1 + rand() % ((int)TOP_COLS - 2));
    return cell;
}

static int same(top_cell_t a, top_cell_t b)
{
    return a.row == b.row && a.col == b.col;
}

static void make_random_problem(top_problem_t *problem)
{
    uint8_t box_count = (uint8_t)(1 + rand() % 2);
    uint8_t i;
    uint8_t walls;
    top_problem_clear(problem);
    problem->match_mode = (rand() & 1) ? TOP_MODE_FREE_MATCH : TOP_MODE_ID_MATCH;
    problem->car = (top_cell_t){(int8_t)(4 + rand() % 2), 0};
    problem->heading = (top_heading_t)(rand() % 4);
    problem->box_count = box_count;
    problem->target_count = box_count;
    for (i = 0U; i < box_count; ++i)
    {
        do
        {
            problem->boxes[i].cell = random_cell();
        } while ((i > 0U && same(problem->boxes[i].cell, problem->boxes[0].cell)) ||
                 same(problem->boxes[i].cell, problem->car));
        problem->boxes[i].id = (uint8_t)(i % 2U);
        problem->boxes[i].id_known = 1U;
    }
    for (i = 0U; i < box_count; ++i)
    {
        int retry;
        for (retry = 0; retry < 100; ++retry)
        {
            uint8_t j;
            int occupied = 0;
            problem->targets[i].cell = random_cell();
            for (j = 0U; j < box_count; ++j)
            {
                occupied |= same(problem->targets[i].cell, problem->boxes[j].cell);
            }
            for (j = 0U; j < i; ++j)
            {
                occupied |= same(problem->targets[i].cell, problem->targets[j].cell);
            }
            if (!occupied)
            {
                break;
            }
        }
        problem->targets[i].id = (uint8_t)(i % 2U);
        problem->targets[i].id_known = 1U;
    }
    walls = (uint8_t)(rand() % 9);
    for (i = 0U; i < walls; ++i)
    {
        top_cell_t wall = random_cell();
        uint8_t j;
        int occupied = same(wall, problem->car);
        for (j = 0U; j < box_count; ++j)
        {
            occupied |= same(wall, problem->boxes[j].cell);
            occupied |= same(wall, problem->targets[j].cell);
        }
        if (!occupied)
        {
            (void)top_problem_set_wall(problem, wall.row, wall.col, 1);
        }
    }
}

int main(void)
{
    unsigned solved = 0U;
    unsigned proven_no_solution = 0U;
    unsigned timed_out = 0U;
    unsigned node_limited = 0U;
    unsigned iteration;
    srand(0x1064);
    for (iteration = 0U; iteration < 200U; ++iteration)
    {
        top_problem_t problem;
        top_config_t config;
        top_result_t result;
        top_verify_report_t report;
        top_status_t status;
        clock_t origin = clock();
        make_random_problem(&problem);
        assert(top_problem_validate(&problem) == TOP_STATUS_OK);
        top_config_default(&config);
        config.now_ms = fuzz_now;
        config.now_user = &origin;
        config.planning_budget_ms = 100U;
        config.max_expansions = 800U;
        status = top_plan(&problem, &config, &result);
        if (status == TOP_STATUS_OK)
        {
            ++solved;
            assert(result.complete);
            assert(result.exec_point_count <= result.raw_point_count);
            assert(top_verify_result(&problem, &config, &result, &report) == TOP_VERIFY_OK);
            if (result.exec_point_count > 1U)
            {
                top_result_t corrupted = result;
                corrupted.exec_points[1].cell.row = -1;
                assert(top_verify_result(&problem, &config, &corrupted, &report) !=
                       TOP_VERIFY_OK);
            }
        }
        else
        {
            assert(status == TOP_STATUS_NO_SOLUTION ||
                   status == TOP_STATUS_TIMEOUT_NO_SOLUTION ||
                   status == TOP_STATUS_NODE_LIMIT_NO_SOLUTION);
            if (status == TOP_STATUS_NO_SOLUTION)
            {
                ++proven_no_solution;
            }
            else if (status == TOP_STATUS_TIMEOUT_NO_SOLUTION)
            {
                ++timed_out;
            }
            else
            {
                ++node_limited;
            }
        }
    }
    printf("fuzz passed: solved=%u no_solution=%u timeout=%u node_limit=%u\n",
           solved,
           proven_no_solution,
           timed_out,
           node_limited);
    assert(solved > 40U);
    return 0;
}
