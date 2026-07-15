#include "TopPlanner.h"
#include "TopVerify.h"

#include <stdio.h>
#include <string.h>
#include <time.h>

typedef struct
{
    const char *name;
    const char *text;
} bench_map_t;

static const bench_map_t maps[] = {
    {"legacy_map0",
     "..............\n"
     "..............\n"
     "..............\n"
     "......B.TT....\n"
     "..............\n"
     "C.....B.......\n"
     "..............\n"
     "..............\n"
     "..............\n"
     "..............\n"},
    {"legacy_map2",
     ".#............\n"
     ".T......#####.\n"
     "#B###...#...#.\n"
     "....#...#T#.#.\n"
     "....#####T#.#.\n"
     "C......B..B.#.\n"
     "...........##.\n"
     "..............\n"
     ".....####.....\n"
     "..............\n"},
    {"legacy_map3",
     ".#...#..#....T\n"
     "...#..#.####..\n"
     "#.###.#.#.....\n"
     ".T#.#.#.#..#..\n"
     ".##.#.........\n"
     "C.#.......##..\n"
     ".D.......D.#..\n"
     "..B.#.....BD..\n"
     ".B..########..\n"
     "....#T........\n"},
    {"legacy_map4",
     "......#....#T#\n"
     "...........#.#\n"
     ".....#.#...#.#\n"
     ".#.#..B....#..\n"
     ".#.#.#.#......\n"
     "C#.#..B..#.#..\n"
     ".....#.#.#.#..\n"
     ".#.##.B..#.##.\n"
     ".....#.#......\n"
     "....T.......T.\n"},
    {"legacy_map5",
     ".#.T....#.....\n"
     "........#..T..\n"
     "......D.#.....\n"
     "........#..B..\n"
     "C...#####.....\n"
     "..B.#.........\n"
     "....#....B....\n"
     "....#.........\n"
     "....#.......T.\n"
     "....#.........\n"},
    {"legacy_map6",
     "..#.#........T\n"
     "....#......#..\n"
     ".B..#B........\n"
     ".........#...#\n"
     "..#....#......\n"
     "C..#..#T..#...\n"
     "............B.\n"
     "####.##..##...\n"
     "..............\n"
     "T.......#.....\n"}
};

static uint32_t bench_now_ms(void *user)
{
    clock_t origin = *(clock_t *)user;
    return (uint32_t)(((clock() - origin) * 1000) / CLOCKS_PER_SEC);
}

static int parse_map(const char *text, top_problem_t *problem)
{
    int row = 0;
    int col = 0;
    top_problem_clear(problem);
    problem->match_mode = TOP_MODE_FREE_MATCH;
    problem->heading = TOP_HEADING_RIGHT;
    while (*text != '\0' && row < (int)TOP_ROWS)
    {
        char ch = *text++;
        if (ch == '\n')
        {
            ++row;
            col = 0;
            continue;
        }
        if (col >= (int)TOP_COLS)
        {
            continue;
        }
        if (ch == '#')
        {
            (void)top_problem_set_wall(problem, row, col, 1);
        }
        else if (ch == 'C')
        {
            problem->car = (top_cell_t){(int8_t)row, (int8_t)col};
        }
        else if (ch == 'B' && problem->box_count < TOP_MAX_BOXES)
        {
            uint8_t at = problem->box_count++;
            problem->boxes[at].cell = (top_cell_t){(int8_t)row, (int8_t)col};
            problem->boxes[at].id_known = 1U;
            problem->boxes[at].id = at;
        }
        else if (ch == 'T' && problem->target_count < TOP_MAX_TARGETS)
        {
            uint8_t at = problem->target_count++;
            problem->targets[at].cell = (top_cell_t){(int8_t)row, (int8_t)col};
            problem->targets[at].id_known = 1U;
            problem->targets[at].id = at;
        }
        else if (ch == 'D' && problem->bomb_count < TOP_MAX_BOMBS)
        {
            problem->bombs[problem->bomb_count++] =
                (top_cell_t){(int8_t)row, (int8_t)col};
        }
        ++col;
    }
    return top_problem_validate(problem) == TOP_STATUS_OK;
}

int main(void)
{
    size_t i;
    printf("workspace=%u bytes result=%u bytes\n",
           (unsigned)top_planner_workspace_bytes(),
           (unsigned)sizeof(top_result_t));
    for (i = 0U; i < sizeof(maps) / sizeof(maps[0]); ++i)
    {
        top_problem_t problem;
        top_config_t config;
        top_result_t result;
        top_verify_report_t report;
        top_status_t status;
        clock_t origin = clock();
        if (!parse_map(maps[i].text, &problem))
        {
            printf("%-12s invalid map\n", maps[i].name);
            continue;
        }
        top_config_default(&config);
        config.now_ms = bench_now_ms;
        config.now_user = &origin;
        config.planning_budget_ms = 1000U;
        config.max_expansions = 1600U;
        config.heuristic_weight_permille = 8000U;
        status = top_plan(&problem, &config, &result);
        printf("%-12s %-23s plan=%4ums motion=%6ums nodes=%4u/%4u points=%3u",
               maps[i].name,
               top_status_string(status),
               (unsigned)result.planning_ms,
               (unsigned)result.motion_ms,
               result.expanded_nodes,
               result.generated_nodes,
               result.exec_point_count);
        if (status == TOP_STATUS_OK || status == TOP_STATUS_PARTIAL_REPLAN)
        {
            top_verify_code_t verify = top_verify_result(&problem, &config, &result, &report);
            printf(" verify=%s", top_verify_string(verify));
        }
        putchar('\n');
    }
    return 0;
}
