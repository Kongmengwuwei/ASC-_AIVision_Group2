#include "Algorithm_Test.h"
#include "data_handle.h"
#include "path.h"
#include "path_follow.h"
#include <string.h>

/*
 * 内部状态：
 * s_grid  : 地图位图（按位存储障碍/箱子/目标/炸弹）
 * s_rows  : 当前地图行数
 * s_cols  : 当前地图列数
 * s_car   : 小车当前位置
 * s_ready : 地图是否已初始化
 */
static uint8 s_grid[MAP_ROWS * MAP_COLS] = {0};
static int s_rows = MAP_ROWS;
static int s_cols = MAP_COLS;
static Position s_car = {0, 0};
static uint8 s_ready = 0u;
static uint8 s_path_ready = 0u;
size_t s_path_index = 0u;
static int s_box_id_map[MAP_ROWS * MAP_COLS];
static int s_target_id_map[MAP_ROWS * MAP_COLS];
static uint8 s_preset_input_enabled = 0U;
static MapPresetConfig s_active_preset;

// 判断坐标是否在地图范围内。
static int in_range(int row, int col)
{
    return (row >= 0 && row < s_rows && col >= 0 && col < s_cols);
}

// 二维坐标映射到一维数组下标。
static int grid_index(int row, int col)
{
    return row * s_cols + col;
}

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
    size_t i;

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
        for (i = 0U; i < Boxes_count; i++)
        {
            boxes[i] = preset->boxes[i];
            boxes[i].id = MAP_PRESET_UNKNOWN_ID;
        }
    }
    if (Targets_count > 0U)
    {
        for (i = 0U; i < Targets_count; i++)
        {
            targets[i] = preset->targets[i];
            targets[i].id = MAP_PRESET_UNKNOWN_ID;
        }
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
        return;
    }

    if (preset_index >= Map_preset_count)
    {
        preset_index = 0U;
    }
    if (!Map_Preset_BuildConfig(preset_index, &s_active_preset))
    {
        s_preset_input_enabled = 0U;
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

static void reset_id_maps(void)
{
    int i;
    for (i = 0; i < (MAP_ROWS * MAP_COLS); i++)
    {
        s_box_id_map[i] = -1;
        s_target_id_map[i] = -1;
    }
}

static int get_id_by_flag(uint8 element_flag, int idx)
{
    // 仅箱子和目标点需要维护ID，其它元素ID统一按0处理。
    if (element_flag == CELL_BOX)
    {
        return s_box_id_map[idx];
    }
    if (element_flag == CELL_TARGET)
    {
        return s_target_id_map[idx];
    }
    return 0;
}

// 若箱子和目标点都具备有效ID，则必须ID相同才允许消除；否则按旧规则直接消除。
static int can_box_clear_target(int box_id, int target_id)
{
    int box_has_id = (box_id > 0);
    int target_has_id = (target_id > 0);

    if (box_has_id && target_has_id)
    {
        return (box_id == target_id);
    }
    return 1;
}

// 将点集写入地图位图（越界点自动忽略）。
static void fill_points(const Position *points, int count, uint8 flag, int *id_map)
{
    int i;
    if (points == 0 || count <= 0)
    {
        return;
    }

    for (i = 0; i < count; i++)
    {
        int row = points[i].row;
        int col = points[i].col;
        if (in_range(row, col))
        {
            int idx = grid_index(row, col);
            s_grid[idx] |= flag;
            if (id_map != 0)
            {
                id_map[idx] = points[i].id;
            }
        }
    }
}

// 解析移动命令，输出行列方向增量。
static int decode_move(char move_cmd, int *d_row, int *d_col)
{
    if (d_row == 0 || d_col == 0)
    {
        return 0;
    }

    *d_row = 0;
    *d_col = 0;

    switch (move_cmd)
    {
    case 'W':
    case 'w':
        *d_row = -1;
        return 1;
    case 'S':
    case 's':
        *d_row = 1;
        return 1;
    case 'A':
    case 'a':
        *d_col = -1;
        return 1;
    case 'D':
    case 'd':
        *d_col = 1;
        return 1;
    case 'Q':
    case 'q':
        *d_row = -1;
        *d_col = -1;
        return 1;
    case 'E':
    case 'e':
        *d_row = -1;
        *d_col = 1;
        return 1;
    case 'Z':
    case 'z':
        *d_row = 1;
        *d_col = -1;
        return 1;
    case 'C':
    case 'c':
        *d_row = 1;
        *d_col = 1;
        return 1;
    default:
        return 0;
    }
}

// 将路径相邻点位移转换为 Move_car 可识别的方向命令。
static int delta_to_move_cmd(int d_row, int d_col, char *cmd)
{
    if (cmd == 0)
    {
        return 0;
    }

    if (d_row == -1 && d_col == 0)
    {
        *cmd = 'W';
        return 1;
    }
    if (d_row == 1 && d_col == 0)
    {
        *cmd = 'S';
        return 1;
    }
    if (d_row == 0 && d_col == -1)
    {
        *cmd = 'A';
        return 1;
    }
    if (d_row == 0 && d_col == 1)
    {
        *cmd = 'D';
        return 1;
    }
    if (d_row == -1 && d_col == -1)
    {
        *cmd = 'Q';
        return 1;
    }
    if (d_row == -1 && d_col == 1)
    {
        *cmd = 'E';
        return 1;
    }
    if (d_row == 1 && d_col == -1)
    {
        *cmd = 'Z';
        return 1;
    }
    if (d_row == 1 && d_col == 1)
    {
        *cmd = 'C';
        return 1;
    }
    return 0;
}

// 清除(center_row, center_col)周围3x3范围内的墙。
static void clear_obstacles_3x3(int center_row, int center_col)
{
    int row;
    int col;

    for (row = center_row - 1; row <= center_row + 1; row++)
    {
        for (col = center_col - 1; col <= center_col + 1; col++)
        {
            if (!in_range(row, col))
            {
                continue;
            }
            {
                int idx = grid_index(row, col);
                s_grid[idx] &= (uint8)(~CELL_OBSTACLE);
            }
        }
    }
}

// 用外部输入对象初始化内部地图状态。
void Test_init_map(int rows, int cols,
                   const Position *obstacles, int obstacles_cnt,
                   const Position *boxes, int boxes_cnt,
                   const Position *targets, int targets_cnt,
                   const Position *bombs, int bombs_cnt,
                   Position car_start)
{
    if (rows > 0 && rows <= MAP_ROWS)
    {
        s_rows = rows;
    }
    else
    {
        s_rows = MAP_ROWS;
    }

    if (cols > 0 && cols <= MAP_COLS)
    {
        s_cols = cols;
    }
    else
    {
        s_cols = MAP_COLS;
    }

    memset(s_grid, 0, sizeof(s_grid));
    reset_id_maps();

    fill_points(obstacles, obstacles_cnt, CELL_OBSTACLE, 0);
    fill_points(boxes, boxes_cnt, CELL_BOX, s_box_id_map);
    fill_points(targets, targets_cnt, CELL_TARGET, s_target_id_map);
    fill_points(bombs, bombs_cnt, CELL_BOMB, 0);

    if (in_range(car_start.row, car_start.col))
    {
        s_car = car_start;
    }
    else
    {
        s_car.row = 0;
        s_car.col = 0;
    }

    s_ready = 1u;
}

// 从全局识别结果加载地图。
void Test_Data_Load(void)
{
    Test_init_map(MAP_ROWS, MAP_COLS,
                  obstacles, (int)Obstacles_count,
                  boxes, (int)Boxes_count,
                  targets, (int)Targets_count,
                  bombs, (int)Bombs_count,
                  car);
}

// 导出指定元素坐标。
int Test_get_positions(uint8 element_flag, Position *out_points, int max_points)
{
    int count = 0;
    int row;
    int col;

    if (!s_ready || element_flag == 0u || max_points <= 0)
    {
        return 0;
    }

    for (row = 0; row < s_rows; row++)
    {
        for (col = 0; col < s_cols; col++)
        {
            uint8 cell = s_grid[grid_index(row, col)];
            if ((cell & element_flag) != 0u)
            {
                if (out_points != 0 && count < max_points)
                {
                    int idx = grid_index(row, col);
                    out_points[count].row = row;
                    out_points[count].col = col;
                    out_points[count].id = get_id_by_flag(element_flag, idx);
                }
                count++;
            }
        }
    }

    if (count > max_points)
    {
        return max_points;
    }
    return count;
}

// 将内部状态回写到全局数组。
void Test_Data_Save(void)
{
    if (!s_ready)
    {
        return;
    }

    memset(obstacles, 0, sizeof(obstacles));
    memset(boxes, 0, sizeof(boxes));
    memset(targets, 0, sizeof(targets));
    memset(bombs, 0, sizeof(bombs));

    Obstacles_count = (size_t)Test_get_positions(CELL_OBSTACLE, obstacles, MAX_OBSTACLES);
    Boxes_count = (size_t)Test_get_positions(CELL_BOX, boxes, MAX_BOXES);
    Targets_count = (size_t)Test_get_positions(CELL_TARGET, targets, MAX_TARGETS);
    Bombs_count = (size_t)Test_get_positions(CELL_BOMB, bombs, MAX_BOMBS);

    car = s_car;
}

/*
 * 执行一次小车移动：
 * 1) 撞墙不能走；
 * 2) 可推动箱子一格；
 * 3) 箱子进目标点后，箱子和目标点同时消失；
 * 4) 可推动炸弹；炸弹撞墙时爆炸并清除3x3墙体。
 */
Move_Result Move_car(char move_cmd)
{
    int d_row = 0;
    int d_col = 0;
    int next_row;
    int next_col;
    int push_row;
    int push_col;
    int next_idx;
    int push_idx;
    uint8 next_cell;
    uint8 push_cell;

    if (!s_ready)
    {
        return MOVE_BLOCKED;
    }

    if (!decode_move(move_cmd, &d_row, &d_col))
    {
        return MOVE_BLOCKED;
    }

    next_row = s_car.row + d_row;
    next_col = s_car.col + d_col;
    if (!in_range(next_row, next_col))
    {
        return MOVE_BLOCKED;
    }

    next_cell = s_grid[grid_index(next_row, next_col)];
    next_idx = grid_index(next_row, next_col);
    if ((next_cell & CELL_OBSTACLE) != 0u)
    {
        return MOVE_BLOCKED;
    }

    // 处理推箱子。
    if ((next_cell & CELL_BOX) != 0u)
    {
        push_row = next_row + d_row;
        push_col = next_col + d_col;
        if (!in_range(push_row, push_col))
        {
            return MOVE_BLOCKED;
        }

        push_idx = grid_index(push_row, push_col);
        push_cell = s_grid[push_idx];
        if ((push_cell & (CELL_OBSTACLE | CELL_BOX | CELL_BOMB)) != 0u)
        {
            return MOVE_BLOCKED;
        }

        s_grid[next_idx] &= (uint8)(~CELL_BOX);
        if ((push_cell & CELL_TARGET) != 0u)
        {
            int box_id = s_box_id_map[next_idx];
            int target_id = s_target_id_map[push_idx];
            if (can_box_clear_target(box_id, target_id))
            {
                // 同ID(或任一方无ID)时，箱子和目标点消失
                s_box_id_map[next_idx] = -1;
                s_grid[push_idx] &= (uint8)(~CELL_TARGET);
                s_target_id_map[push_idx] = -1;
                s_car.row = next_row;
                s_car.col = next_col;
                return MOVE_BOX_TARGET_CLEARED;
            }
            // ID不同：目标点保留，箱子可穿过并停在该格
        }

        // 普通推箱子：箱子ID随箱子移动
        s_grid[push_idx] |= CELL_BOX;
        s_box_id_map[push_idx] = s_box_id_map[next_idx];
        s_box_id_map[next_idx] = -1;
        s_car.row = next_row;
        s_car.col = next_col;
        return MOVE_OK;
    }

    // 处理推炸弹。
    if ((next_cell & CELL_BOMB) != 0u)
    {
        push_row = next_row + d_row;
        push_col = next_col + d_col;
        if (!in_range(push_row, push_col))
        {
            return MOVE_BLOCKED;
        }

        push_idx = grid_index(push_row, push_col);
        push_cell = s_grid[push_idx];
        if ((push_cell & (CELL_BOX | CELL_BOMB)) != 0u)
        {
            return MOVE_BLOCKED;
        }

        s_grid[next_idx] &= (uint8)(~CELL_BOMB);
        if ((push_cell & CELL_OBSTACLE) != 0u)
        {
            // 炸弹撞墙：炸弹消失，墙体爆炸清除
            clear_obstacles_3x3(push_row, push_col);
            s_car.row = next_row;
            s_car.col = next_col;
            return MOVE_BOMB_EXPLODED;
        }

        // 普通推炸弹
        s_grid[push_idx] |= CELL_BOMB;
        s_car.row = next_row;
        s_car.col = next_col;
        return MOVE_OK;
    }

    // 普通前进。
    s_car.row = next_row;
    s_car.col = next_col;
    return MOVE_OK;
}

void Test_Path_Init(void)
{
    if (!s_ready)
    {
        Test_Data_Load();
    }
    // 小车对齐到路径起点，从第2个点开始逐步执行。
    s_car = car_path[0];
    car = s_car;
    s_path_index = 1u;
    s_path_ready = 1u;

    Test_Data_Save();
}

int Test_Path_Step(char *out_cmd)
{
    char cmd;
    Move_Result ret;

    if (!s_path_ready)
    {
        return -1;
    }

    while (s_path_index < Car_path_count)
    {
        int d_row = car_path[s_path_index].row - car_path[s_path_index - 1u].row;
        int d_col = car_path[s_path_index].col - car_path[s_path_index - 1u].col;
        s_path_index++;

        // 相邻重复点不产生移动命令，直接跳过。
        if (d_row == 0 && d_col == 0)
        {
            continue;
        }

        if (!delta_to_move_cmd(d_row, d_col, &cmd))
        {
            s_path_ready = 0u;
            return -3;
        }

        ret = Move_car(cmd);

        if (out_cmd != 0)
        {
            *out_cmd = cmd;
        }

        if (ret == MOVE_BLOCKED)
        {
            s_path_ready = 0u;
            return -4;
        }

        Test_Data_Save();
        return 1;
    }

    // 路径结束。
    s_path_ready = 0u;
    Test_Data_Save();
    return 0;
}

int Test_Path_ALL(void)
{
    int ret;
    int step_count = 0;

    if (!s_ready)
    {
        Test_Data_Load();
    }
    if (!s_ready || Car_path_count < 2U)
    {
        return -1;
    }
    if (!in_range(car_path[0].row, car_path[0].col))
    {
        return -2;
    }

    Test_Path_Init();

    while (1)
    {
        ret = Test_Path_Step(0);
        if (ret == 1)
        {
            step_count++;
            continue;
        }
        if (ret == 0)
        {
            return step_count;
        }
        return ret;
    }
}
