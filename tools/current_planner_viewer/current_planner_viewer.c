#include "Game_logic.h"
#include "path.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

typedef struct
{
    int valid;
    size_t raw_count;
    size_t exec_count;
    double plan_ms;
    double build_ms;
    double distance_cells;
    path_map_snapshot_t start_map;
    Position raw[MAX_CAR_PATH];
    Position exec[MAX_CAR_PATH];
} phase_result_t;

static void load_config(const MapPresetConfig *config, int known_ids)
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
        for (i = 0U; i < Boxes_count; ++i)
        {
            boxes[i].id = MAP_PRESET_UNKNOWN_ID;
        }
        for (i = 0U; i < Targets_count; ++i)
        {
            targets[i].id = MAP_PRESET_UNKNOWN_ID;
        }
    }
}

static void take_snapshot(path_map_snapshot_t *snapshot)
{
    memset(snapshot, 0, sizeof(*snapshot));
    snapshot->obstacles_count = Obstacles_count;
    snapshot->boxes_count = Boxes_count;
    snapshot->targets_count = Targets_count;
    snapshot->bombs_count = Bombs_count;
    snapshot->car_pose_grid = car;
    memcpy(snapshot->obstacles_buf, obstacles, sizeof(obstacles));
    memcpy(snapshot->boxes_buf, boxes, sizeof(boxes));
    memcpy(snapshot->targets_buf, targets, sizeof(targets));
    memcpy(snapshot->bombs_buf, bombs, sizeof(bombs));
}

static uint8 config_object_id_at(const Position *objects,
                                 size_t count,
                                 Position object,
                                 uint8 fallback)
{
    size_t i;
    for (i = 0U; i < count; ++i)
    {
        if (objects[i].row == object.row && objects[i].col == object.col)
        {
            return objects[i].id;
        }
    }
    return fallback;
}

/* Continue from the simulated state produced by Plan_path_Identify().
 * Recognition changes IDs at runtime, so the preset IDs are restored by cell
 * while the post-identification car, walls and bombs remain untouched. */
static void load_push_state_after_identify(const MapPresetConfig *config,
                                           const path_map_snapshot_t *identify_end)
{
    size_t i;

    memset(obstacles, 0, sizeof(obstacles));
    memset(boxes, 0, sizeof(boxes));
    memset(targets, 0, sizeof(targets));
    memset(bombs, 0, sizeof(bombs));
    memset(car_path, 0, sizeof(car_path));

    Obstacles_count = identify_end->obstacles_count;
    Boxes_count = identify_end->boxes_count;
    Targets_count = identify_end->targets_count;
    Bombs_count = identify_end->bombs_count;
    Car_path_count = 0U;
    car = identify_end->car_pose_grid;

    memcpy(obstacles, identify_end->obstacles_buf, sizeof(obstacles));
    memcpy(boxes, identify_end->boxes_buf, sizeof(boxes));
    memcpy(targets, identify_end->targets_buf, sizeof(targets));
    memcpy(bombs, identify_end->bombs_buf, sizeof(bombs));

    for (i = 0U; i < Boxes_count; ++i)
    {
        boxes[i].id = config_object_id_at(config->boxes,
                                          config->boxes_count,
                                          boxes[i],
                                          (uint8)(i + 1U));
    }
    for (i = 0U; i < Targets_count; ++i)
    {
        targets[i].id = config_object_id_at(config->targets,
                                            config->targets_count,
                                            targets[i],
                                            (uint8)(i + 1U));
    }
}

static double path_distance(const Position *path_points, size_t count)
{
    size_t i;
    double distance = 0.0;

    for (i = 1U; i < count; ++i)
    {
        double dr = (double)path_points[i].row - (double)path_points[i - 1U].row;
        double dc = (double)path_points[i].col - (double)path_points[i - 1U].col;
        distance += sqrt(dr * dr + dc * dc);
    }
    return distance;
}

static void run_loaded_phase(const MapPresetConfig *config,
                             int identify_phase,
                             phase_result_t *result,
                             path_map_snapshot_t *end_map)
{
    path_map_snapshot_t before;
    path_map_snapshot_t after;
    clock_t started;
    size_t i;

    memset(result, 0, sizeof(*result));
    take_snapshot(&before);
    result->start_map = before;

    started = clock();
    if (identify_phase)
    {
        Plan_path_Identify();
    }
    else if (config->plan_mode == MAP_PRESET_PLAN_MODE1)
    {
        Plan_path_Mode1();
    }
    else
    {
        Plan_path_Mode2();
    }
    result->plan_ms = 1000.0 * (double)(clock() - started) / (double)CLOCKS_PER_SEC;

    if (Car_path_count < 2U || Car_path_count > MAX_CAR_PATH)
    {
        return;
    }

    result->raw_count = Car_path_count;
    memcpy(result->raw, car_path, Car_path_count * sizeof(Position));
    take_snapshot(&after);
    if (end_map != NULL)
    {
        *end_map = after;
    }

    started = clock();
    if (!path_build_exec_from_planner(result->raw,
                                      result->raw_count,
                                      &after,
                                      &before,
                                      result->exec,
                                      MAX_CAR_PATH,
                                      &result->exec_count))
    {
        return;
    }
    result->build_ms = 1000.0 * (double)(clock() - started) / (double)CLOCKS_PER_SEC;

    for (i = 0U; i < result->exec_count; ++i)
    {
        path_inverse_remap_exec_point(&result->exec[i]);
    }
    result->distance_cells = path_distance(result->exec, result->exec_count);
    result->valid = 1;
}

static int same_cell(Position a, Position b)
{
    return a.row == b.row && a.col == b.col;
}

static int cell_in_list(const Position *list, size_t count, int row, int col)
{
    size_t i;
    for (i = 0U; i < count; ++i)
    {
        if ((int)list[i].row == row && (int)list[i].col == col)
        {
            return 1;
        }
    }
    return 0;
}

static const char *event_class(uint8 id)
{
    if ((id & BOMB_EXPLOSION) != 0U)
    {
        return "event explosion";
    }
    if ((id & IDENTIFICATION) != 0U)
    {
        return "event identify";
    }
    if ((id & PUSH_END_POINT) != 0U)
    {
        return "event push-end";
    }
    if ((id & PUSH_START_POINT) != 0U)
    {
        return "event push-start";
    }
    return "waypoint";
}

static void print_event_text(FILE *out, uint8 id)
{
    int wrote = 0;
#define WRITE_EVENT(bit, name)                    \
    do                                             \
    {                                              \
        if ((id & (bit)) != 0U)                   \
        {                                          \
            fprintf(out, "%s%s", wrote ? "|" : "", (name)); \
            wrote = 1;                             \
        }                                          \
    } while (0)

    WRITE_EVENT(BOMB_EXPLOSION, "爆炸");
    WRITE_EVENT(TURNING_POINT, "转折");
    WRITE_EVENT(IDENTIFICATION, "识别");
    WRITE_EVENT(PUSH_START_POINT, "推物开始");
    WRITE_EVENT(PUSH_END_POINT, "推物结束");
    if (!wrote)
    {
        fputs("移动", out);
    }
#undef WRITE_EVENT
}

static void render_base_map(FILE *out, const path_map_snapshot_t *map)
{
    int row;
    int col;
    const int cell = 28;

    for (row = 0; row < MAP_ROWS; ++row)
    {
        for (col = 0; col < MAP_COLS; ++col)
        {
            int x = col * cell;
            int y = row * cell;
            fprintf(out, "<rect class=\"grid-cell\" x=\"%d\" y=\"%d\" width=\"%d\" height=\"%d\"/>",
                    x, y, cell, cell);
            if (cell_in_list(map->obstacles_buf, map->obstacles_count, row, col))
            {
                fprintf(out, "<rect class=\"wall\" x=\"%d\" y=\"%d\" width=\"%d\" height=\"%d\"/>",
                        x + 1, y + 1, cell - 2, cell - 2);
            }
            if (cell_in_list(map->targets_buf, map->targets_count, row, col))
            {
                fprintf(out, "<circle class=\"target\" cx=\"%d\" cy=\"%d\" r=\"7\"/>",
                        x + cell / 2, y + cell / 2);
            }
            if (cell_in_list(map->boxes_buf, map->boxes_count, row, col))
            {
                fprintf(out, "<rect class=\"box\" x=\"%d\" y=\"%d\" width=\"14\" height=\"14\" rx=\"2\"/>",
                        x + 7, y + 7);
            }
            if (cell_in_list(map->bombs_buf, map->bombs_count, row, col))
            {
                fprintf(out, "<circle class=\"bomb\" cx=\"%d\" cy=\"%d\" r=\"7\"/>",
                        x + cell / 2, y + cell / 2);
            }
        }
    }
    fprintf(out,
            "<polygon class=\"car\" points=\"%d,%d %d,%d %d,%d\"/>",
            (int)map->car_pose_grid.col * cell + 7,
            (int)map->car_pose_grid.row * cell + 6,
            (int)map->car_pose_grid.col * cell + 22,
            (int)map->car_pose_grid.row * cell + 14,
            (int)map->car_pose_grid.col * cell + 7,
            (int)map->car_pose_grid.row * cell + 22);
}

static void render_phase(FILE *out,
                         const char *title,
                         const phase_result_t *phase)
{
    size_t i;
    const int cell = 28;

    fprintf(out, "<article class=\"phase\"><h3>%s</h3>", title);
    if (!phase->valid)
    {
        fputs("<p class=\"failed\">规划或执行路径提取失败</p></article>", out);
        return;
    }

    fprintf(out,
            "<p class=\"metrics\">起点 (%u,%u) → 终点 (%u,%u) · 原始点 %u · 执行点 %u · 距离 %.2f 格 · 规划 %.3f ms · 路径提取 %.3f ms</p>",
            (unsigned)phase->exec[0].row,
            (unsigned)phase->exec[0].col,
            (unsigned)phase->exec[phase->exec_count - 1U].row,
            (unsigned)phase->exec[phase->exec_count - 1U].col,
            (unsigned)phase->raw_count,
            (unsigned)phase->exec_count,
            phase->distance_cells,
            phase->plan_ms,
            phase->build_ms);
    fprintf(out, "<svg viewBox=\"0 0 %d %d\" role=\"img\">", MAP_COLS * cell, MAP_ROWS * cell);
    render_base_map(out, &phase->start_map);

    fputs("<polyline class=\"route\" points=\"", out);
    for (i = 0U; i < phase->exec_count; ++i)
    {
        fprintf(out, "%d,%d ",
                (int)phase->exec[i].col * cell + cell / 2,
                (int)phase->exec[i].row * cell + cell / 2);
    }
    fputs("\"/>", out);

    for (i = 0U; i < phase->exec_count; ++i)
    {
        int x = (int)phase->exec[i].col * cell + cell / 2;
        int y = (int)phase->exec[i].row * cell + cell / 2;
        fprintf(out, "<circle class=\"%s\" cx=\"%d\" cy=\"%d\" r=\"%d\"/>",
                event_class(phase->exec[i].id),
                x,
                y,
                phase->exec[i].id == 0U ? 3 : 6);
        if (phase->exec[i].id != 0U)
        {
            fprintf(out, "<text class=\"event-label\" x=\"%d\" y=\"%d\">%u</text>",
                    x + 6, y - 6, (unsigned)i);
        }
    }
    fputs("</svg>", out);

    fputs("<details><summary>按执行顺序查看路径点</summary><ol class=\"points\">", out);
    for (i = 0U; i < phase->exec_count; ++i)
    {
        fprintf(out, "<li><code>(%u,%u)</code> — ",
                (unsigned)phase->exec[i].row,
                (unsigned)phase->exec[i].col);
        print_event_text(out, phase->exec[i].id);
        fputs("</li>", out);
    }
    fputs("</ol></details></article>", out);
}

static void write_header(FILE *out)
{
    fputs("<!doctype html><html lang=\"zh-CN\"><head><meta charset=\"utf-8\">"
          "<meta name=\"viewport\" content=\"width=device-width,initial-scale=1\">"
          "<title>当前路径规划结果</title><style>"
          ":root{font-family:Inter,Segoe UI,Microsoft YaHei,sans-serif;color:#172033;background:#f3f6fa}"
          "body{max-width:1280px;margin:auto;padding:24px}h1{margin-bottom:8px}.intro{color:#536075}"
          ".legend{display:flex;flex-wrap:wrap;gap:14px;margin:18px 0;padding:12px;background:white;border-radius:10px}"
          ".map{margin:24px 0;padding:18px;background:white;border:1px solid #dce3ec;border-radius:14px;box-shadow:0 3px 12px #16233b12}"
          ".panels{display:grid;grid-template-columns:repeat(auto-fit,minmax(430px,1fr));gap:18px}"
          ".phase{min-width:0}.metrics{color:#536075;font-size:14px}.failed{color:#b42318;font-weight:700}"
          "svg{width:100%;height:auto;background:#fcfdff;border:1px solid #ced7e3;border-radius:8px}"
          ".grid-cell{fill:#fff;stroke:#e2e8f0;stroke-width:1}.wall{fill:#303846}.target{fill:#fff;stroke:#8b5cf6;stroke-width:3}"
          ".box{fill:#c88743;stroke:#75451e;stroke-width:2}.bomb{fill:#20242b;stroke:#ef4444;stroke-width:3}.car{fill:#16a34a;stroke:#14532d;stroke-width:2}"
          ".route{fill:none;stroke:#2563eb;stroke-width:4;stroke-linecap:round;stroke-linejoin:round;opacity:.72}"
          ".waypoint{fill:#1d4ed8}.event{stroke:#fff;stroke-width:2}.identify{fill:#f59e0b}.push-start{fill:#06b6d4}.push-end{fill:#22c55e}.explosion{fill:#ef4444}"
          ".event-label{font-size:9px;font-weight:700;fill:#111827;paint-order:stroke;stroke:white;stroke-width:3px}"
          "details{margin-top:10px}.points{columns:2;column-gap:30px;font-size:13px}.points li{break-inside:avoid;margin:3px 0}code{color:#1d4ed8}"
          "</style></head><body><h1>当前算法的预设地图路径结果</h1>"
          "<p class=\"intro\">直接调用项目当前 Game_logic、Algorithm 与 path 模块生成。蓝线为提取后的实际执行路径。</p>"
          "<div class=\"legend\"><span>■ 墙</span><span>□ 箱子</span><span>○ 目标点</span><span>● 炸弹</span>"
          "<span>橙色：识别</span><span>青色：推物开始</span><span>绿色：推物结束</span><span>红色：爆炸</span></div>", out);
}

static int parse_map_argument(int argc, char **argv, int *selected_map, const char **output_path)
{
    int i;
    *selected_map = -1;
    *output_path = "current_planner_paths.html";

    for (i = 1; i < argc; ++i)
    {
        if (strcmp(argv[i], "--map") == 0 && i + 1 < argc)
        {
            *selected_map = atoi(argv[++i]);
        }
        else if (strcmp(argv[i], "--output") == 0 && i + 1 < argc)
        {
            *output_path = argv[++i];
        }
        else
        {
            fprintf(stderr, "用法: %s [--map 0-%u] [--output 文件.html]\n",
                    argv[0], (unsigned)(Map_preset_count - 1U));
            return 0;
        }
    }
    if (*selected_map >= (int)Map_preset_count)
    {
        fprintf(stderr, "地图编号越界: %d\n", *selected_map);
        return 0;
    }
    return 1;
}

int main(int argc, char **argv)
{
    FILE *out;
    const char *output_path;
    int selected_map;
    size_t map_index;
    unsigned failures = 0U;

    if (!parse_map_argument(argc, argv, &selected_map, &output_path))
    {
        return 2;
    }
    out = fopen(output_path, "wb");
    if (out == NULL)
    {
        perror(output_path);
        return 3;
    }

    write_header(out);
    for (map_index = 0U; map_index < Map_preset_count; ++map_index)
    {
        MapPresetConfig config;
        phase_result_t identify;
        phase_result_t push;
        path_map_snapshot_t identify_end;
        Position transition = {0U, 0U, 0U};

        if (selected_map >= 0 && map_index != (size_t)selected_map)
        {
            continue;
        }
        if (!Map_Preset_BuildConfig(map_index, &config))
        {
            ++failures;
            continue;
        }

        load_config(&config, 0);
        memset(&identify_end, 0, sizeof(identify_end));
        run_loaded_phase(&config, 1, &identify, &identify_end);
        memset(&push, 0, sizeof(push));
        if (identify.valid)
        {
            transition = identify.exec[identify.exec_count - 1U];
            identify_end.car_pose_grid = transition;
            load_push_state_after_identify(&config, &identify_end);
            run_loaded_phase(&config, 0, &push, NULL);
            if (push.valid && !same_cell(transition, push.exec[0]))
            {
                push.valid = 0;
            }
        }
        if (!identify.valid || !push.valid)
        {
            ++failures;
        }

        fprintf(out,
                "<section class=\"map\"><h2>%s（map%u，%s）</h2><div class=\"panels\">",
                config.name,
                (unsigned)map_index,
                config.plan_mode == MAP_PRESET_PLAN_MODE1 ? "Mode1" : "Mode2");
        render_phase(out, "识别阶段", &identify);
        render_phase(out, "推箱阶段（从识别终点继续）", &push);
        fputs("</div></section>", out);

        printf("map%u: identify raw=%u exec=%u, transition=(%u,%u), push raw=%u exec=%u%s\n",
               (unsigned)map_index,
               (unsigned)identify.raw_count,
               (unsigned)identify.exec_count,
               (unsigned)transition.row,
               (unsigned)transition.col,
               (unsigned)push.raw_count,
               (unsigned)push.exec_count,
               (identify.valid && push.valid) ? "" : " [FAILED]");
    }
    fputs("</body></html>", out);
    fclose(out);
    printf("HTML 路径报告已生成: %s\n", output_path);
    return failures == 0U ? 0 : 1;
}
