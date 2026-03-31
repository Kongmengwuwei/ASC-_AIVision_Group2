#include "Algorithm_Test.h"
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

// 将点集写入地图位图（越界点自动忽略）。
static void fill_points(const Position *points, int count, uint8 flag)
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
            s_grid[grid_index(row, col)] |= flag;
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

            s_grid[grid_index(row, col)] &= (uint8)(~CELL_OBSTACLE);
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

    fill_points(obstacles, obstacles_cnt, CELL_OBSTACLE);
    fill_points(boxes, boxes_cnt, CELL_BOX);
    fill_points(targets, targets_cnt, CELL_TARGET);
    fill_points(bombs, bombs_cnt, CELL_BOMB);

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
                    out_points[count].row = row;
                    out_points[count].col = col;
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

        push_cell = s_grid[grid_index(push_row, push_col)];
        if ((push_cell & (CELL_OBSTACLE | CELL_BOX | CELL_BOMB)) != 0u)
        {
            return MOVE_BLOCKED;
        }

        s_grid[grid_index(next_row, next_col)] &= (uint8)(~CELL_BOX);
        if ((push_cell & CELL_TARGET) != 0u)
        {
            s_grid[grid_index(push_row, push_col)] &= (uint8)(~CELL_TARGET);
            s_car.row = next_row;
            s_car.col = next_col;
            return MOVE_BOX_TARGET_CLEARED;
        }

        s_grid[grid_index(push_row, push_col)] |= CELL_BOX;
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

        push_cell = s_grid[grid_index(push_row, push_col)];
        if ((push_cell & (CELL_BOX | CELL_BOMB)) != 0u)
        {
            return MOVE_BLOCKED;
        }

        s_grid[grid_index(next_row, next_col)] &= (uint8)(~CELL_BOMB);
        if ((push_cell & CELL_OBSTACLE) != 0u)
        {
            clear_obstacles_3x3(push_row, push_col);
            s_car.row = next_row;
            s_car.col = next_col;
            return MOVE_BOMB_EXPLODED;
        }

        s_grid[grid_index(push_row, push_col)] |= CELL_BOMB;
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

    if (ret < 0)
    {
        return ret;
    }

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
