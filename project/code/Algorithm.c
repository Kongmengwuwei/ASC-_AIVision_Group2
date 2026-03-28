#include "Algorithm.h"

// 网格构建
static void grid_build(int rows, int cols,
                       const Point *obstacles, int obstacles_cnt,
                       const Point *bombs, int bombs_cnt,
                       const Point *boxes, int boxes_cnt,
                       uint8_t *grid)
{
    // 确定网格大小
    int n = rows * cols;
    // 清空网格
    memset(grid, 0, n);
    // 标记各块
    for (int i = 0; i < obstacles_cnt; i++)
    {
        int r = obstacles[i].row, c = obstacles[i].col;
        if (r >= 0 && r < rows && c >= 0 && c < cols)
            grid[r * cols + c] |= OBSTACLE;
    }
    for (int i = 0; i < bombs_cnt; i++)
    {
        int r = bombs[i].row, c = bombs[i].col;
        if (r >= 0 && r < rows && c >= 0 && c < cols)
            grid[r * cols + c] |= BOMB;
    }
    for (int i = 0; i < boxes_cnt; i++)
    {
        int r = boxes[i].row, c = boxes[i].col;
        if (r >= 0 && r < rows && c >= 0 && c < cols)
        {
            grid[r * cols + c] |= BOX;
            grid[r * cols + c] = (grid[r * cols + c] & 0x0F) | ((i <= 15 ? i : 15) << 4);
        }
    }
}

