#include "Algorithm_Test.h"
#include <string.h>

/*
 * 内部状态说明：
 * s_grid  : 地图位图，按位存储障碍/箱子/目标/炸弹。
 * s_rows  : 当前地图行数。
 * s_cols  : 当前地图列数。
 * s_car   : 小车当前位置（网格坐标）。
 * s_ready : 地图是否已完成初始化。
 */
static uint8 s_grid[MAP_ROWS * MAP_COLS] = {0};
static int s_rows = MAP_ROWS;
static int s_cols = MAP_COLS;
static Point s_car = {0, 0};
static uint8 s_ready = 0u;

// 判断坐标是否在当前地图有效范围内。
static int in_range(int row, int col)
{
    return (row >= 0 && row < s_rows && col >= 0 && col < s_cols);
}

// 将二维(row,col)映射到一维数组下标。
static int grid_index(int row, int col)
{
    return row * s_cols + col;
}

// 将 points 列表中的元素按 flag 写入位图（越界点自动忽略）。
static void fill_points(const Point *points, int count, uint8 flag)
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

/*
 * 把用户输入命令解析为方向增量。
 * d_row = 行方向，d_col = 列方向：
 * 上(-1,0) 下(1,0) 左(0,-1) 右(0,1)
 */
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
        *d_row = -1;
        return 1;
    case 'S':
        *d_row = 1;
        return 1;
    case 'A':
        *d_col = -1;
        return 1;
    case 'D':
        *d_col = 1;
        return 1;
    default:
        return 0;
    }
}

// 清除以(center_row, center_col)为中心 3x3 范围内的所有墙(OBSTACLE)。
static void clear_obstacles_3x3(int center_row, int center_col)
{
    int row;
    int col;

    for (row = center_row - 1; row <= center_row + 1; row++)
    {
        for (col = center_col - 1; col <= center_col + 1; col++)
        {
            if (in_range(row, col))
            {
                s_grid[grid_index(row, col)] &= (uint8)(~CELL_OBSTACLE);
            }
        }
    }
}

/*
 * 初始化地图：
 * 1) 校验并设置地图尺寸；
 * 2) 清空内部位图；
 * 3) 写入障碍/箱子/目标/炸弹；
 * 4) 设置小车初始位置；
 * 5) 标记地图已就绪。
 */
void Test_init_map(int rows, int cols,
                   const Point *obstacles, int obstacles_cnt,
                   const Point *boxes, int boxes_cnt,
                   const Point *targets, int targets_cnt,
                   const Point *bombs, int bombs_cnt,
                   Point car_start)
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

// 直接读取识别模块当前全局数据，刷新内部地图。
void Test_Data_Load(void)
{
    Test_init_map(MAP_ROWS, MAP_COLS,
                  obstacles, (int)actual_obstacles_count,
                  boxes, (int)actual_boxes_count,
                  targets, (int)actual_targets_count,
                       bombs, (int)actual_bombs_count,
                       car);
}

/*
 * 统计并导出指定元素的坐标。
 * 当 out_points 为空时，函数仍会进行计数；
 * 当计数超过 max_points 时，仅写入前 max_points 个。
 */
int Test_get_positions(uint8 element_flag, Point *out_points, int max_points)
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

/*
 * 将内部位图反向同步到 Camera_handler 的全局数组：
 * obstacles / boxes / targets / map_bombs / car
 */
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

    actual_obstacles_count = (size_t)Test_get_positions(CELL_OBSTACLE, obstacles, MAX_OBSTACLES);
    actual_boxes_count = (size_t)Test_get_positions(CELL_BOX, boxes, MAX_BOXES);
    actual_targets_count = (size_t)Test_get_positions(CELL_TARGET, targets, MAX_TARGETS);
    actual_bombs_count = (size_t)Test_get_positions(CELL_BOMB, bombs, MAX_BOMBS);

    car = s_car;
}

/*
 * 执行一次小车移动并处理交互：
 * A. 普通移动：目标格为空或只有目标点 -> 小车直接前进；
 * B. 推箱子：
 *    - 前方是箱子，且箱子下一格必须可进入（不能是墙/箱子/炸弹）；
 *    - 若箱子下一格是目标点，则箱子和目标点同时消失；
 * C. 推炸弹：
 *    - 前方是炸弹，且炸弹下一格不能是箱子或炸弹；
 *    - 若炸弹下一格是墙，则触发爆炸，清除该墙中心 3x3 范围内的墙。
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

    // 规则1：推箱子（单向 1 格）。
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

        // 先移除原位置箱子。
        s_grid[grid_index(next_row, next_col)] &= (uint8)(~CELL_BOX);
        if ((push_cell & CELL_TARGET) != 0u)
        {
            // 规则2：箱子进入目标点时，箱子和目标点都消失。
            s_grid[grid_index(push_row, push_col)] &= (uint8)(~CELL_TARGET);
            s_car.row = next_row;
            s_car.col = next_col;
            return MOVE_BOX_TARGET_CLEARED;
        }

        // 普通推箱子成功：箱子占据下一格。
        s_grid[grid_index(push_row, push_col)] |= CELL_BOX;
        s_car.row = next_row;
        s_car.col = next_col;
        return MOVE_OK;
    }

    // 规则3：推炸弹。
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

        // 炸弹离开原位置。
        s_grid[grid_index(next_row, next_col)] &= (uint8)(~CELL_BOMB);
        if ((push_cell & CELL_OBSTACLE) != 0u)
        {
            // 规则4：炸弹撞墙后爆炸，清除墙体 3x3。
            clear_obstacles_3x3(push_row, push_col);
            s_car.row = next_row;
            s_car.col = next_col;
            return MOVE_BOMB_EXPLODED;
        }

        // 炸弹未撞墙，正常被推动一格。
        s_grid[grid_index(push_row, push_col)] |= CELL_BOMB;
        s_car.row = next_row;
        s_car.col = next_col;
        return MOVE_OK;
    }

    // 普通前进（前方不是墙/箱子/炸弹）。
    s_car.row = next_row;
    s_car.col = next_col;
    return MOVE_OK;
}
