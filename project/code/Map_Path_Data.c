#include "Map_Path_Data.h"
#include <string.h>

// 当前生效地图对象数量
size_t Obstacles_count = 0; // 当前障碍物数量
size_t Boxes_count = 0;     // 当前箱子数量
size_t Targets_count = 0;   // 当前目标点数量
size_t Bombs_count = 0;     // 当前炸弹数量
size_t Car_path_count = 0;  // 当前路径点数量

// 当前生效地图对象位置
Position obstacles[MAX_OBSTACLES] = {{0}}; // 当前障碍物坐标列表
Position boxes[MAX_BOXES] = {{0}};         // 当前箱子坐标列表
Position targets[MAX_TARGETS] = {{0}};     // 当前目标点坐标列表
Position bombs[MAX_BOMBS] = {{0}};         // 当前炸弹坐标列表
Position car = {1, 1};                     // 车辆整数栅格位置
Position car_path[MAX_CAR_PATH] = {{0}};   // 预留路径坐标列表
CarPose car_position = {1, 1, 0, 1.00, 1.00, 0.00};   // 车辆浮点栅格位置

// 在此直接输入地图
const char map_text1[] =".#............\n"
                       ".T......#####.\n"
                       "#B###...#...#.\n"
                       "....#...#T#.#.\n"
                       "....#####T#.#.\n"
                       "C......B..B.#.\n"
                       "..........###.\n"
                       "..............\n"
                       ".....####.....\n"
                       "..............\n";
const char map_text2[] ="..#.#........T\n"
                       "....#......#..\n"
                       ".B..#B........\n"
                       ".........#...#\n"
                       "..#....#......\n"
                       "C..#..#T..#...\n"
                       "............B.\n"
                       "####.##..##...\n"
                       "..............\n"
                       "T.......#.....\n";
const char map_text3[] =".#...#..#....T\n"
                       "...#..#.####..\n"
                       "#.###.#.#.....\n"
                       ".T#.#.#.#..#..\n"
                       ".##.#.........\n"
                       "..#.......##..\n"
                       "CD.......D.#..\n"
                       "..B.#.....BD..\n"
                       ".B..########..\n"
                       "....#T........\n";
const char map_text4[] ="......#....#T#\n"
                       "...........#.#\n"
                       ".....#.#...#.#\n"
                       ".#.#..B....#..\n"
                       ".#.#.#.#......\n"
                       "C#.#..B..#.#..\n"
                       ".....#.#.#.#..\n"
                       ".#.##.B..#.##.\n"
                       ".....#.#......\n"
                       "....T.......T.\n";
const char map_text5[] =".#.T....#.....\n"
                       "........#..T..\n"
                       "......D.#.....\n"
                       "........#..B..\n"
                       "C...#####.....\n"
                       "..B.#.........\n"
                       "....#....B....\n"
                       "....#.........\n"
                       "....#.......T.\n"
                       "....#.........\n";

const MapPresetTextConfig map_preset_texts[] = {
    {"map1", map_text1, MAP_PRESET_PLAN_MODE2, 0.0f, 0, 0U, 0, 0U},
    {"map2", map_text2, MAP_PRESET_PLAN_MODE2, 0.0f, 0, 0U, 0, 0U},
    {"map3", map_text3, MAP_PRESET_PLAN_MODE2, 0.0f, 0, 0U, 0, 0U},
    {"map4", map_text4, MAP_PRESET_PLAN_MODE2, 0.0f, 0, 0U, 0, 0U},
    {"map5", map_text5, MAP_PRESET_PLAN_MODE2, 0.0f, 0, 0U, 0, 0U},
};

const size_t Map_preset_count = sizeof(map_preset_texts) / sizeof(map_preset_texts[0]);

static uint8 map_preset_next_id(const uint8 *ids, size_t id_count, size_t index)
{
    if (ids != 0 && index < id_count)
    {
        return ids[index];
    }
    return (uint8)(index + 1U);
}

uint8 Map_Preset_BuildConfig(size_t preset_index, MapPresetConfig *out_config)
{
    const MapPresetTextConfig *src;
    const char *p;
    size_t row = 0U;
    size_t col = 0U;
    size_t box_index = 0U;
    size_t target_index = 0U;
    uint8 car_found = 0U;

    if (out_config == 0 || Map_preset_count == 0U)
    {
        return 0U;
    }

    if (preset_index >= Map_preset_count)
    {
        preset_index = 0U;
    }

    src = &map_preset_texts[preset_index];
    memset(out_config, 0, sizeof(*out_config));
    out_config->name = src->name;
    out_config->plan_mode = src->plan_mode;
    out_config->car_start.row = 0U;
    out_config->car_start.col = 0U;
    out_config->car_start.id = 0U;
    out_config->car_yaw_deg = src->car_yaw_deg;

    p = src->map_text;
    while (p != 0 && *p != '\0' && row < MAP_ROWS)
    {
        char ch = *p++;
        Position pos;

        if (ch == '\r')
        {
            continue;
        }
        if (ch == '\n')
        {
            row++;
            col = 0U;
            continue;
        }
        if (col >= MAP_COLS)
        {
            continue;
        }

        pos.row = (uint8)row;
        pos.col = (uint8)col;
        pos.id = 0U;

        switch (ch)
        {
        case '#':
            if (out_config->obstacles_count < MAX_OBSTACLES)
            {
                out_config->obstacles[out_config->obstacles_count++] = pos;
            }
            break;
        case 'B':
        case 'b':
            if (out_config->boxes_count < MAX_BOXES)
            {
                pos.id = map_preset_next_id(src->box_ids, src->box_id_count, box_index);
                out_config->boxes[out_config->boxes_count++] = pos;
            }
            box_index++;
            break;
        case 'T':
        case 't':
            if (out_config->targets_count < MAX_TARGETS)
            {
                pos.id = map_preset_next_id(src->target_ids, src->target_id_count, target_index);
                out_config->targets[out_config->targets_count++] = pos;
            }
            target_index++;
            break;
        case 'D':
        case 'd':
            if (out_config->bombs_count < MAX_BOMBS)
            {
                out_config->bombs[out_config->bombs_count++] = pos;
            }
            break;
        case 'C':
        case 'c':
            if (!car_found)
            {
                out_config->car_start = pos;
                car_found = 1U;
            }
            break;
        default:
            break;
        }

        col++;
    }

    return 1U;
}
