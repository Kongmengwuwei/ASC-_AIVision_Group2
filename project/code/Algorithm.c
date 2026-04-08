#include "Algorithm.h"
#include <string.h>

/*
 * Algorithm.c 负责“单个箱子”的底层路径规划，核心能力包括：
 * 1) 2D A*：小车在静态障碍下寻路（可选 4/8 方向）。
 * 2) 3D A*：在状态中加入“推箱朝向 face”，同时规划“推进行为 + 绕行换向”。
 * 3) 炸弹策略：先推炸弹炸墙，再在新地形上推目标箱子。
 * 4) 结果整合：比较“直接推箱”与“炸墙后推箱”，输出最短可行总路径。
 *
 * 约定：
 * - 返回值 < 0 表示不可达/失败；>= 0 表示路径长度或步骤数。
 * - Position 统一使用栅格坐标(row, col)。
 * - 本文件多处函数为 static，仅供本模块内部组合使用。
 */

/*
 * 性能优化说明：
 * 1) 为 2D/3D A* 堆分别维护“节点在堆中的位置表”，将原本 O(N) 的 heap_update
 *    降为 O(1) 定位 + O(logN) 上浮。
 * 2) 2D A* 预计算“靠近箱子区域”标记，避免每次扩展都线性扫描全部箱子。
 */
static int heap_pos_2d[grid_size];
static int heap_pos_3d[grid_size * 4];

/*
 * 将障碍物/炸弹/箱子写入一维 grid。
 * grid 每个单元是位标记：
 * - 低 4 位记录是否为障碍、炸弹、箱子等类别。
 * - 高 4 位在箱子场景下记录“箱子索引”(0~15，超出截断到 15)。
 */
static void grid_build(int row_cnt, int col_cnt,
                       const Position *obstacles, int obstacles_cnt,
                       const Position *bombs, int bombs_cnt,
                       const Position *boxes, int boxes_cnt,
                       int use_blocked_bombs,
                       uint8_t *grid)
{
    // 确定网格大小
    int n = row_cnt * col_cnt;
    if (n > grid_size)
        n = grid_size;
    // 清空网格
    memset(grid, 0, n);
    // 标记各块
    for (int i = 0; i < obstacles_cnt; i++)
    {
        int r = obstacles[i].row, c = obstacles[i].col;
        if (r >= 0 && r < row_cnt && c >= 0 && c < col_cnt)
            grid[r * col_cnt + c] |= OBSTACLE;
    }
    for (int i = 0; i < bombs_cnt; i++)
    {
        int r = bombs[i].row, c = bombs[i].col;
        if (r >= 0 && r < row_cnt && c >= 0 && c < col_cnt)
            grid[r * col_cnt + c] |= BOMB;
    }
    for (int i = 0; i < boxes_cnt; i++)
    {
        int r = boxes[i].row, c = boxes[i].col;
        if (r >= 0 && r < row_cnt && c >= 0 && c < col_cnt)
        {
            int index = r * col_cnt + c;
            grid[index] |= BOX;
            grid[index] = (grid[index] & 0x0F) | ((i <= 15 ? i : 15) << 4);
        }
    }
}
/* 检查单元是否不可通行：障碍 / 炸弹 / 已封锁炸弹。 */
static int check_obstacle(uint8_t *grid, int col_cnt, int row, int col)
{
    if (row < 0 || col < 0)
        return 0;
    return ((grid[col_cnt * row + col] & (OBSTACLE | BOMB | BLOCKED_BOMB)) != 0);
}
/* 与 check_obstacle 类似，但把普通炸弹视为可通行（用于特殊策略场景）。 */
static int check_obstacle_except_common_bomb(uint8_t *grid, int col_cnt, int row, int col)
{
    if (row < 0 || col < 0)
        return 0;
    return ((grid[col_cnt * row + col] & (OBSTACLE | BLOCKED_BOMB)) != 0);
}
/*
 * 判断某位置是否存在“除 skip_index 外的箱子”。
 * 用于“正在推动第 skip_index 个箱子”时，忽略它自身占位，仅检查其他箱子碰撞。
 */
static inline int BOX_EXCLUDING(const uint8_t *grid, int col_cnt, int row, int col, size_t skip_index)
{
    size_t index = (size_t)(row * col_cnt + col);
    uint8_t cell = grid[index];
    if (!(cell & BOX))
        return 0;
    if (skip_index == SIZE_MAX)
        return 1;
    return (size_t)(cell >> 4) != skip_index;
}
/* BOX_EXCLUDING 的边界包装版本。 */
static int check_box_with_excluding(uint8_t *grid, int col_cnt, int row, int col, size_t skip_index)
{
    if (row < 0 || col < 0)
        return 0;
    return (BOX_EXCLUDING(grid, col_cnt, row, col, skip_index));
}
/*
 * 判断“箱子下一步落点”是否被阻塞：
 * - 越界阻塞
 * - 网格障碍阻塞
 * - 与其他箱子重叠阻塞（排除当前正在推动的箱子）
 */
static int check_push_destination_blocked(uint8_t *grid, int row_cnt, int col_cnt,
                                          const Position *boxes, int boxes_cnt,
                                          int pushing_box_index,
                                          int row, int col)
{
    if (row < 0 || row >= row_cnt || col < 0 || col >= col_cnt)
        return 1;
    if (check_obstacle(grid, col_cnt, row, col))
        return 1;
    for (int i = 0; i < boxes_cnt; i++)
    {
        if (i == pushing_box_index)
            continue;
        if (boxes[i].row == row && boxes[i].col == col)
            return 1;
    }
    return 0;
}
/*
 * 边限制位图（当前版本未在主流程启用）：
 * 预留给“禁止某些转移边”类高级约束，例如人工禁行线/方向约束。
 */
static const uint32_t *banned_edge_container = NULL;

/* 在边限制位图中标记某条边为“禁用”。 */
static inline void banned_edge_set(uint32_t *banned_edge_container, int edge_index)
{
    if (!banned_edge_container || edge_index < 0 || edge_index > banned_edge_cnt)
        return;
    int unit_index = edge_index >> 5;
    int bit_index = edge_index & 31;
    banned_edge_container[unit_index] |= (1u << bit_index);
}
/* 查询某条边是否被标记为“禁用”。 */
static inline int banned_edge_check(const uint32_t *banned_edge_container, int edge_index)
{
    if (!banned_edge_container || edge_index < 0 || edge_index > banned_edge_cnt)
        return 0;
    int unit_index = edge_index >> 5;
    int bit_index = edge_index & 31;
    return (banned_edge_container[unit_index] & (1u << bit_index)) != 0;
}

/*
 * 以下是 2D A* 使用的最小堆工具：
 * - 以 f_cost 为主排序键，h_cost 为次排序键（更接近终点者优先）。
 * - 用数组实现二叉堆，降低 open 集合取最小代价节点的复杂度。
 */

/* 交换堆数组中的两个槽位。 */
static inline void heap_swap(binary_heap *heap, int i, int j)
{
    int node_i = heap->index[i];
    int node_j = heap->index[j];
    heap->index[i] = node_j;
    heap->index[j] = node_i;
    heap_pos_2d[node_i] = j;
    heap_pos_2d[node_j] = i;
}
/* 比较堆内两个节点优先级。 */
static inline int heap_compare_less(const a_star_param *nodes, const binary_heap *heap, int i, int j)
{
    int index_i = heap->index[i];
    int index_j = heap->index[j];
    if (nodes[index_i].f_cost != nodes[index_j].f_cost)
        return nodes[index_i].f_cost < nodes[index_j].f_cost;

    return nodes[index_i].h_cost < nodes[index_j].h_cost;
}

/* 堆上浮：用于新插入或代价下降后的重排。 */
static inline void heap_sift_up(a_star_param *nodes, binary_heap *heap, int i)
{
    while (i > 0)
    {
        int parent = (i - 1) >> 1;
        if (heap_compare_less(nodes, heap, i, parent))
        {
            heap_swap(heap, i, parent);
            i = parent;
        }
        else
            break;
    }
}
/* 堆下沉：用于弹出堆顶后把尾元素放到根并恢复堆序。 */
static inline void heap_sift_down(a_star_param *nodes, binary_heap *heap, int i)
{
    int size = heap->size;
    while (1)
    {
        int left = (i << 1) + 1;
        if (left >= size)
            break;
        int right = left + 1;
        int min = i;

        if (heap_compare_less(nodes, heap, left, min))
            min = left;
        if (right < size && heap_compare_less(nodes, heap, right, min))
            min = right;

        if (min != i)
        {
            heap_swap(heap, i, min);
            i = min;
        }
        else
            break;
    }
}

/* 压入 open 集合。 */
static void heap_push(a_star_param *nodes, binary_heap *heap, int node_index)
{
    if (heap->size >= grid_size || node_index < 0 || node_index >= grid_size)
        return;
    if (heap_pos_2d[node_index] >= 0)
        return;

    int i = heap->size++;
    heap->index[i] = node_index;
    heap_pos_2d[node_index] = i;
    heap_sift_up(nodes, heap, i);
}
/* 弹出 open 集合中最优节点。 */
static int heap_pop(a_star_param *nodes, binary_heap *heap)
{
    if (heap->size == 0)
        return -1;

    int top = heap->index[0];
    heap_pos_2d[top] = -1;
    heap->size--;
    if (heap->size > 0)
    {
        int moved = heap->index[heap->size];
        heap->index[0] = moved;
        heap_pos_2d[moved] = 0;
        heap_sift_down(nodes, heap, 0);
    }
    return top;
}

/* 节点已有更优 g_cost 时，触发堆内位置更新。 */
static void heap_update(a_star_param *nodes, binary_heap *heap, int node_index)
{
    if (node_index < 0 || node_index >= grid_size)
        return;
    if (heap_pos_2d[node_index] < 0)
        return;
    heap_sift_up(nodes, heap, heap_pos_2d[node_index]);
}

/* 八方向近似启发：直移代价 10，斜移代价 14。 */
static inline int diagonal_distance(Position p1, Position p2)
{
    int dif_row = p1.row > p2.row ? p1.row - p2.row : p2.row - p1.row;
    int dif_col = p1.col > p2.col ? p1.col - p2.col : p2.col - p1.col;
    int min_dif = dif_row < dif_col ? dif_row : dif_col;
    int max_dif = dif_row > dif_col ? dif_row : dif_col;
    // 直走权重10，斜走权重14
    return 14 * min_dif + 10 * (max_dif - min_dif);
}
/* 曼哈顿距离：用于箱子推送阶段的启发估计。 */
static inline int manhattan_distance_cells(Position p1, Position p2)
{
    int dif_row = p1.row > p2.row ? p1.row - p2.row : p2.row - p1.row;
    int dif_col = p1.col > p2.col ? p1.col - p2.col : p2.col - p1.col;
    return dif_row + dif_col;
}

/*
 * 从箱子轨迹中提取“推点”序列（当前流程未直接调用，作为策略扩展保留）：
 * - 每次方向变化意味着小车必须重定位到新的推面。
 * - 输出的小车推点可用于分段执行或动作控制。
 */
static int extract_push_points_from_path(const Position *box_path, int path_len,
                                         Position *push_points, int *push_points_cnt)
{
    // 如果无法构成一条线段，则不存在推点
    if (path_len < 2 || !box_path || !push_points || !push_points_cnt)
    {
        if (push_points_cnt)
            *push_points_cnt = 0;
        return 0;
    }
    int cnt = 0;

    // 获取起步时第一段直道的推动方向
    int current_dir_row = box_path[1].row - box_path[0].row;
    int current_dir_col = box_path[1].col - box_path[0].col;
    // 根据初始方向，计算出起步时的首个推点
    push_points[cnt].row = (int8_t)(box_path[0].row - current_dir_row);
    push_points[cnt].col = (int8_t)(box_path[0].col - current_dir_col);
    cnt++;
    // 顺着箱子的后续轨迹按顺序遍历，寻找方向变化的拐角
    for (int i = 1; i < path_len - 1; i++)
    {
        int next_dir_row = box_path[i + 1].row - box_path[i].row;
        int next_dir_col = box_path[i + 1].col - box_path[i].col;
        // 如果前进的方向发生了变化，说明箱子在方格(i)遇到了必须转弯的拐点
        if (next_dir_row != current_dir_row || next_dir_col != current_dir_col)
        {
            // 小车在这里结束推进，重新绕去箱子当前(i)位置新的推点
            push_points[cnt].row = (int8_t)(box_path[i].row - next_dir_row);
            push_points[cnt].col = (int8_t)(box_path[i].col - next_dir_col);
            cnt++;

            // 切换比对基准给刚发现的新直道，为下一个拐角做准备
            current_dir_row = next_dir_row;
            current_dir_col = next_dir_col;
        }
    }
    *push_points_cnt = cnt;
    return 1;
}

/* 2D A* 节点池。 */
a_star_param a_star[grid_size];

/*
 * 2D A*：小车从 start 到 target 的行走路径规划。
 * allow_diagonal 语义：
 * - 1：空车可 8 向移动，但靠近箱子时自动降级为 4 向防碰撞。
 * - 0：强制 4 向（典型用于推箱时更稳定的栅格动作）。
 */
static int a_star_path_plan(int row_cnt, int col_cnt,
                            const Position *obstacles, int obstacles_cnt,
                            const Position *bombs, int bombs_cnt,
                            const Position *boxes, int boxes_cnt,
                            const Position start, const Position target,
                            int allow_diagonal,
                            Position *out_path)
{
    // 起点终点合法性判断
    if (start.row >= row_cnt || start.col >= col_cnt)
        return -1;
    if (target.row >= row_cnt || target.col >= col_cnt)
        return -1;

    // 构建网格状态
    uint8_t grid[grid_size];
    grid_build(row_cnt, col_cnt, obstacles, obstacles_cnt, bombs, bombs_cnt, boxes, boxes_cnt, 0, grid);

    // 初始化 A* 的节点访问组标识
    memset(a_star, 0, sizeof(a_star));

    binary_heap open_set;
    open_set.size = 0;
    for (int i = 0; i < grid_size; i++)
    {
        heap_pos_2d[i] = -1;
    }

    int start_index = start.row * col_cnt + start.col;
    int target_index = target.row * col_cnt + target.col;

    a_star[start_index].g_cost = 0;
    a_star[start_index].h_cost = diagonal_distance(start, target);
    a_star[start_index].f_cost = a_star[start_index].g_cost + a_star[start_index].h_cost;
    a_star[start_index].parent_index = -1;
    a_star[start_index].open_or_close = 1; // 1: open

    heap_push(a_star, &open_set, start_index);

    // 8向移动方向 (0-3: 直行正交向, 4-7: 对角斜向)
    const int dir_row[] = {-1, 1, 0, 0, -1, -1, 1, 1};
    const int dir_col[] = {0, 0, -1, 1, -1, 1, -1, 1};
    const int move_cost[] = {10, 10, 10, 10, 14, 14, 14, 14};
    uint8_t near_box_map[grid_size];
    memset(near_box_map, 0, sizeof(near_box_map));

    if (allow_diagonal)
    {
        for (int b = 0; b < boxes_cnt; b++)
        {
            int br = boxes[b].row;
            int bc = boxes[b].col;
            if (br >= row_cnt || bc >= col_cnt)
                continue;
            for (int dr = -1; dr <= 1; dr++)
            {
                for (int dc = -1; dc <= 1; dc++)
                {
                    int rr = br + dr;
                    int cc = bc + dc;
                    if (rr >= 0 && rr < row_cnt && cc >= 0 && cc < col_cnt)
                    {
                        near_box_map[rr * col_cnt + cc] = 1;
                    }
                }
            }
        }
    }

    while (open_set.size > 0)
    {
        int current_index = heap_pop(a_star, &open_set);
        if (current_index == -1)
            break;

        a_star[current_index].open_or_close = 2; // 2: closed

        int r = current_index / col_cnt;
        int c = current_index % col_cnt;

        if (current_index == target_index)
        {
            // 已找到路径，向外回溯
            int path_len = 0;
            int curr = target_index;
            while (curr != -1)
            {
                out_path[path_len].row = curr / col_cnt;
                out_path[path_len].col = curr % col_cnt;
                path_len++;
                curr = a_star[curr].parent_index;
            }
            // 反转路径（由于回溯的方向是从终点向起点追溯的）
            for (int i = 0; i < path_len / 2; i++)
            {
                Position temp = out_path[i];
                out_path[i] = out_path[path_len - 1 - i];
                out_path[path_len - 1 - i] = temp;
            }
            return path_len; // 正常返回路径步数
        }

        // 当前格是否靠近箱子（预计算查表，避免循环内重复扫描全部箱子）
        int is_near_box = allow_diagonal ? near_box_map[current_index] : 0;

        // 当周边有箱子时，或不允许斜穿(推箱模式)时，强制降级为 4 向移动防碰；在空旷地带则全开 8 向以加快寻路和移动速度
        int dir_count = (!allow_diagonal || is_near_box) ? 4 : 8;

        // 检查周围邻居节点
        for (int i = 0; i < dir_count; i++)
        {
            int nr = r + dir_row[i];
            int nc = c + dir_col[i];

            if (nr >= 0 && nr < row_cnt && nc >= 0 && nc < col_cnt)
            {
                // 如果是对角线移动(i >= 4)，则强制进行"禁止切角"验证
                if (i >= 4)
                {
                    // 正交两边分别为 (r + dr[i], c) 和 (r, c + dc[i])
                    // 确保上方/下方 和 左方/右方 都是空路，防止斜穿墙角 or 箱子
                    if (check_obstacle(grid, col_cnt, nr, c) || (grid[nr * col_cnt + c] & BOX) ||
                        check_obstacle(grid, col_cnt, r, nc) || (grid[r * col_cnt + nc] & BOX))
                        continue;
                }

                int neighbor_index = nr * col_cnt + nc;

                // 利用已有方法检测目标格本身是否可通过网格 (包括墙和箱子)
                if (check_obstacle(grid, col_cnt, nr, nc) || (grid[neighbor_index] & BOX))
                    continue;

                // 如果已经加入了 close 集合
                if (a_star[neighbor_index].open_or_close == 2)
                    continue;

                int tentative_g_cost = a_star[current_index].g_cost + move_cost[i]; // 累加移动代价值 10 或 14

                if (a_star[neighbor_index].open_or_close == 0)
                { // unvisited (此前未被访问过)
                    a_star[neighbor_index].parent_index = current_index;
                    a_star[neighbor_index].g_cost = tentative_g_cost;
                    a_star[neighbor_index].h_cost = diagonal_distance((Position){nr, nc}, target);
                    a_star[neighbor_index].f_cost = a_star[neighbor_index].g_cost + a_star[neighbor_index].h_cost;
                    a_star[neighbor_index].open_or_close = 1;
                    heap_push(a_star, &open_set, neighbor_index);
                }
                else if (tentative_g_cost < a_star[neighbor_index].g_cost)
                { // 已在 open 集合中，但找到了开销更优的路径
                    a_star[neighbor_index].parent_index = current_index;
                    a_star[neighbor_index].g_cost = tentative_g_cost;
                    a_star[neighbor_index].f_cost = a_star[neighbor_index].g_cost + a_star[neighbor_index].h_cost;
                    heap_update(a_star, &open_set, neighbor_index); // 更新使得在二叉堆内上浮
                }
            }
        }
    }
    return -1; // 寻找不到路径
}

/*
 * 3D A* 状态定义：
 * state = (box_row, box_col, face)
 * - face 表示小车站在箱子哪一侧并准备向哪个方向推。
 * - 同一个箱子格子有 4 个朝向状态，因此容量是 grid_size * 4。
 */
a_star_3d_param a_star_3d[grid_size * 4];

/* 3D A* 的堆交换。 */
static inline void heap_swap_3d(binary_heap_3d *heap, int i, int j)
{
    int node_i = heap->index[i];
    int node_j = heap->index[j];
    heap->index[i] = node_j;
    heap->index[j] = node_i;
    heap_pos_3d[node_i] = j;
    heap_pos_3d[node_j] = i;
}
/* 3D A* 的优先级比较：f 为主，g 和 h 为辅。 */
static inline int heap_compare_less_3d(const binary_heap_3d *heap, int i, int j)
{
    int index_i = heap->index[i];
    int index_j = heap->index[j];
    if (a_star_3d[index_i].f_cost != a_star_3d[index_j].f_cost)
        return a_star_3d[index_i].f_cost < a_star_3d[index_j].f_cost;
    if (a_star_3d[index_i].g_cost != a_star_3d[index_j].g_cost)
        return a_star_3d[index_i].g_cost < a_star_3d[index_j].g_cost;

    return a_star_3d[index_i].h_cost < a_star_3d[index_j].h_cost;
}

/* 3D A* 堆上浮。 */
static inline void heap_sift_up_3d(binary_heap_3d *heap, int i)
{
    while (i > 0)
    {
        int parent = (i - 1) >> 1;
        if (heap_compare_less_3d(heap, i, parent))
        {
            heap_swap_3d(heap, i, parent);
            i = parent;
        }
        else
            break;
    }
}
/* 3D A* 堆下沉。 */
static inline void heap_sift_down_3d(binary_heap_3d *heap, int i)
{
    int size = heap->size;
    while (1)
    {
        int left = (i << 1) + 1;
        if (left >= size)
            break;
        int right = left + 1;
        int min = i;

        if (heap_compare_less_3d(heap, left, min))
            min = left;
        if (right < size && heap_compare_less_3d(heap, right, min))
            min = right;

        if (min != i)
        {
            heap_swap_3d(heap, i, min);
            i = min;
        }
        else
            break;
    }
}

/* 3D A* 入堆。 */
static void heap_push_3d(binary_heap_3d *heap, int node_index)
{
    if (heap->size >= grid_size * 4 || node_index < 0 || node_index >= (grid_size * 4))
        return;
    if (heap_pos_3d[node_index] >= 0)
        return;

    int i = heap->size++;
    heap->index[i] = node_index;
    heap_pos_3d[node_index] = i;
    heap_sift_up_3d(heap, i);
}
/* 3D A* 出堆。 */
static int heap_pop_3d(binary_heap_3d *heap)
{
    if (heap->size == 0)
        return -1;

    int top = heap->index[0];
    heap_pos_3d[top] = -1;
    heap->size--;
    if (heap->size > 0)
    {
        int moved = heap->index[heap->size];
        heap->index[0] = moved;
        heap_pos_3d[moved] = 0;
        heap_sift_down_3d(heap, 0);
    }
    return top;
}

/* 3D A* 节点代价改进后的重排。 */
static void heap_update_3d(binary_heap_3d *heap, int node_index)
{
    if (node_index < 0 || node_index >= (grid_size * 4))
        return;
    if (heap_pos_3d[node_index] < 0)
        return;
    heap_sift_up_3d(heap, heap_pos_3d[node_index]);
}

/*
 * 3D A*（推箱主算法）：
 * - 起点：先枚举 4 个可能推面，筛出小车可到达的入推姿态。
 * - 扩展：
 *   1) 同向直推：箱子前进一步，face 不变。
 *   2) 绕行换向：箱子不动，小车绕到另一侧，face 改变。
 * - 代价设计：
 *   COST_PUSH > COST_WALK，鼓励减少无效走动；
 *   COST_REORIENT_PENALTY 约束频繁换向。
 * - 输出：返回完整“小车路径”，不是仅箱子轨迹。
 */
int a_star_path_plan_3d(int row_cnt, int col_cnt,
                        const Position *obstacles, int obstacles_cnt,
                        const Position *bombs, int bombs_cnt,
                        const Position *boxes, int boxes_cnt,
                        const Position *targets, int targets_cnt,
                        int box_index,
                        Position car_start,
                        Position *full_car_path,
                        Position *best_target_out)
{
    const int COST_WALK = 10;
    const int COST_PUSH = 50;
    const int COST_REORIENT_PENALTY = 30;

    Position local_boxes[MAX_BOXES + 4];
    for (int i = 0; i < boxes_cnt; i++)
        local_boxes[i] = boxes[i];
    Position box_start = local_boxes[box_index];

    uint8_t grid[grid_size];
    grid_build(row_cnt, col_cnt, obstacles, obstacles_cnt, bombs, bombs_cnt, local_boxes, boxes_cnt, 0, grid);

    memset(a_star_3d, 0, sizeof(a_star_3d));

    binary_heap_3d open_set;
    open_set.size = 0;
    for (int i = 0; i < (grid_size * 4); i++)
    {
        heap_pos_3d[i] = -1;
    }

    int target_3d_index = -1;

    const int dir_row_3d[] = {1, -1, 0, 0};
    const int dir_col_3d[] = {0, 0, 1, -1};
    int min_target_distance_from_start = 999999;

    for (int t = 0; t < targets_cnt; t++)
    {
        int h = manhattan_distance_cells(box_start, targets[t]);
        if (h < min_target_distance_from_start)
            min_target_distance_from_start = h;
    }

    // 从4个方向中选择推向
    for (int f = 0; f < 4; f++)
    {
        int push_point_row = box_start.row - dir_row_3d[f];
        int push_point_col = box_start.col - dir_col_3d[f];
        int first_push_row = box_start.row + dir_row_3d[f];
        int first_push_col = box_start.col + dir_col_3d[f];

        local_boxes[box_index] = box_start;
        Position temp_path[grid_size];

        if (check_push_destination_blocked(grid, row_cnt, col_cnt,
                                           local_boxes, boxes_cnt,
                                           box_index,
                                           first_push_row, first_push_col))
            continue;

        // 计算到达推点的路径
        int walk_len = a_star_path_plan(row_cnt, col_cnt,
                                        obstacles, obstacles_cnt, bombs, bombs_cnt,
                                        local_boxes, boxes_cnt,
                                        car_start, (Position){push_point_row, push_point_col},
                                        1,
                                        temp_path);

        if (walk_len >= 0)
        { // 如果小车能顺利绕到这一面
            // 设置带有推向的索引
            int state_index = (box_start.row * col_cnt + box_start.col) * 4 + f;

            // 计算到达目标点的距离
            a_star_3d[state_index].g_cost = walk_len * COST_WALK;
            a_star_3d[state_index].h_cost = min_target_distance_from_start * COST_PUSH; // 调整比例，优先考虑箱子终点
            a_star_3d[state_index].f_cost = a_star_3d[state_index].g_cost + a_star_3d[state_index].h_cost;
            a_star_3d[state_index].parent_index = -1;
            a_star_3d[state_index].open_or_close = 1;
            a_star_3d[state_index].is_push = 0; // 没推过，刚转移到推位

            heap_push_3d(&open_set, state_index);
        }
    }

    while (open_set.size > 0)
    {
        int curr_index = heap_pop_3d(&open_set);
        if (curr_index == -1)
            break;

        a_star_3d[curr_index].open_or_close = 2; // Close

        int row = (curr_index / 4) / col_cnt; // 箱子真实行
        int col = (curr_index / 4) % col_cnt; // 箱子真实列
        int face = curr_index % 4;            // 小车所在的面

        int reached = 0;
        for (int t = 0; t < targets_cnt; t++)
        {
            if (row == targets[t].row && col == targets[t].col)
            {
                reached = 1;
                if (best_target_out)
                    *best_target_out = targets[t];
                break;
            }
        }
        if (reached)
        {
            target_3d_index = curr_index;
            break;
        } // 推完跳出

        // 同向推箱子
        int next_row = row + dir_row_3d[face];
        int next_col = col + dir_col_3d[face];

        if (!check_push_destination_blocked(grid, row_cnt, col_cnt,
                                            local_boxes, boxes_cnt,
                                            box_index,
                                            next_row, next_col))
        {
            int next_index = (next_row * col_cnt + next_col) * 4 + face; // face不变，没换推面

            if (a_star_3d[next_index].open_or_close != 2)
            {
                int tentative_g = a_star_3d[curr_index].g_cost + COST_PUSH; // 离开原路径，添加偏移代价

                if (a_star_3d[next_index].open_or_close == 0 || tentative_g < a_star_3d[next_index].g_cost)
                {
                    a_star_3d[next_index].parent_index = curr_index;
                    a_star_3d[next_index].g_cost = tentative_g;

                    int min_h = 999999;
                    for (int t = 0; t < targets_cnt; t++)
                    {
                        int h = manhattan_distance_cells((Position){next_row, next_col}, targets[t]);
                        if (h < min_h)
                            min_h = h;
                    }

                    a_star_3d[next_index].h_cost = min_h * COST_PUSH;
                    a_star_3d[next_index].f_cost = tentative_g + a_star_3d[next_index].h_cost;
                    a_star_3d[next_index].is_push = 1; // 直走

                    if (a_star_3d[next_index].open_or_close == 0)
                    {
                        a_star_3d[next_index].open_or_close = 1;
                        heap_push_3d(&open_set, next_index);
                    }
                    else
                    {
                        heap_update_3d(&open_set, next_index);
                    }
                }
            }
        }

        // 绕路推箱子
        for (int next_face = 0; next_face < 4; next_face++)
        {
            if (next_face == face)
                continue; // 跳过相同方向

            int next_index = (row * col_cnt + col) * 4 + next_face; // 只改变了推向
            if (a_star_3d[next_index].open_or_close == 2)
                continue; // 跳过已关闭的路径

            // 计算小车需要绕到的位置
            int target_face_row = row - dir_row_3d[next_face];
            int target_face_col = col - dir_col_3d[next_face];

            if (target_face_row < 0 || target_face_row >= row_cnt || target_face_col < 0 || target_face_col >= col_cnt)
                continue;
            local_boxes[box_index] = (Position){row, col}; // 将当前被推动对象放在实时位置
            // 换向占位检查：只看障碍和“其他箱子”，不使用静态 grid 的 BOX 位
            if (check_push_destination_blocked(grid, row_cnt, col_cnt,
                                               local_boxes, boxes_cnt,
                                               box_index,
                                               target_face_row, target_face_col))
                continue;

            Position car_from = {row - dir_row_3d[face], col - dir_col_3d[face]};
            Position temp_path[grid_size];

            int walk_len = a_star_path_plan(row_cnt, col_cnt,
                                            obstacles, obstacles_cnt, bombs, bombs_cnt,
                                            local_boxes, boxes_cnt,
                                            car_from, (Position){target_face_row, target_face_col},
                                            1,
                                            temp_path);

            if (walk_len >= 0)
            {
                int tentative_g = a_star_3d[curr_index].g_cost + walk_len * COST_WALK + COST_REORIENT_PENALTY;
                if (a_star_3d[next_index].open_or_close == 0 || tentative_g < a_star_3d[next_index].g_cost)
                {
                    a_star_3d[next_index].parent_index = curr_index;
                    a_star_3d[next_index].g_cost = tentative_g;
                    a_star_3d[next_index].h_cost = a_star_3d[curr_index].h_cost; // 箱子没动
                    a_star_3d[next_index].f_cost = tentative_g + a_star_3d[next_index].h_cost;
                    a_star_3d[next_index].is_push = 0; // 换向推

                    if (a_star_3d[next_index].open_or_close == 0)
                    {
                        a_star_3d[next_index].open_or_close = 1;
                        heap_push_3d(&open_set, next_index);
                    }
                    else
                    {
                        heap_update_3d(&open_set, next_index);
                    }
                }
            }
        }
    }

    if (target_3d_index == -1)
        return -1;

    int sp_path[grid_size * 4];
    int sp_path_len = 0;
    int curr = target_3d_index;

    while (curr != -1)
    {
        sp_path[sp_path_len++] = curr;
        curr = a_star_3d[curr].parent_index;
    }

    for (int i = 0; i < sp_path_len / 2; i++)
    {
        int temp = sp_path[i];
        sp_path[i] = sp_path[sp_path_len - 1 - i];
        sp_path[sp_path_len - 1 - i] = temp;
    }

    int total_car_len = 0;

    // 先让小车跑到推点
    int start_index = sp_path[0];
    int start_row = (start_index / 4) / col_cnt;
    int start_col = (start_index / 4) % col_cnt;
    int start_face = start_index % 4;

    Position car_target = {start_row - dir_row_3d[start_face], start_col - dir_col_3d[start_face]};
    local_boxes[box_index] = (Position){start_row, start_col};
    Position temp[grid_size];

    int walk_len = a_star_path_plan(row_cnt, col_cnt,
                                    obstacles, obstacles_cnt, bombs, bombs_cnt,
                                    local_boxes, boxes_cnt,
                                    car_start, car_target,
                                    1,
                                    temp);

    for (int k = 0; k < walk_len; k++)
        full_car_path[total_car_len++] = temp[k];

    for (int i = 1; i < sp_path_len; i++)
    {
        int prev_index = sp_path[i - 1];
        int curr_index = sp_path[i];

        int prev_r = (prev_index / 4) / col_cnt;
        int prev_c = (prev_index / 4) % col_cnt;
        int prev_f = prev_index % 4;

        int curr_r = (curr_index / 4) / col_cnt;
        int curr_c = (curr_index / 4) % col_cnt;
        int curr_f = curr_index % 4;

        if (a_star_3d[curr_index].is_push == 1)
        {
            // 直推
            full_car_path[total_car_len++] = (Position){prev_r, prev_c};
        }
        else
        {
            // 绕路推
            Position c_from = {prev_r - dir_row_3d[prev_f], prev_c - dir_col_3d[prev_f]};
            Position c_to = {prev_r - dir_row_3d[curr_f], prev_c - dir_col_3d[curr_f]};

            local_boxes[box_index] = (Position){prev_r, prev_c};
            Position temp[grid_size];

            int walk_steps = a_star_path_plan(row_cnt, col_cnt,
                                              obstacles, obstacles_cnt, bombs, bombs_cnt,
                                              local_boxes, boxes_cnt,
                                              c_from, c_to,
                                              1,
                                              temp);

            for (int k = 0; k < walk_steps; k++)
                full_car_path[total_car_len++] = temp[k];
        }
    }
    return total_car_len;
}

/*
 * 模拟炸弹爆炸：清除 bomb_target 为中心 3x3 范围内的障碍物。
 * 用于“第一阶段推炸弹后”更新第二阶段推箱的地图。
 */
void simulate_bomb_explosion(Position *obstacles, int *obstacles_cnt, Position bomb_target)
{
    int new_cnt = 0;
    for (int i = 0; i < *obstacles_cnt; i++)
    {
        int dr = obstacles[i].row - bomb_target.row;
        int dc = obstacles[i].col - bomb_target.col;
        // 如果在 3x3 九宫格内，墙壁被炸毁
        if (dr >= -1 && dr <= 1 && dc >= -1 && dc <= 1)
            continue;
        obstacles[new_cnt++] = obstacles[i];
    }
    // 将多余位置置空
    for (int i = new_cnt; i < *obstacles_cnt; i++)
    {
        obstacles[i].row = -1;
        obstacles[i].col = -1;
    }
    *obstacles_cnt = new_cnt;
}

/*
 * 选择“值得尝试炸掉”的候选墙：
 * 仅在箱子起点与目标点包围盒外扩 1 格区域内筛选，减少组合爆炸搜索量。
 */
int get_candidate_walls(const Position *obstacles, int obstacles_cnt,
                        Position box_start, Position target,
                        Position *out_candidate_walls)
{
    int cnt = 0;
    int min_row = box_start.row < target.row ? box_start.row : target.row;
    int max_row = box_start.row > target.row ? box_start.row : target.row;
    int min_col = box_start.col < target.col ? box_start.col : target.col;
    int max_col = box_start.col > target.col ? box_start.col : target.col;

    // 向外扩张搜索圈
    min_row -= 1;
    max_row += 1;
    min_col -= 1;
    max_col += 1;

    for (int i = 0; i < obstacles_cnt; i++)
    {
        int r = obstacles[i].row;
        int c = obstacles[i].col;
        if (r >= min_row && r <= max_row && c >= min_col && c <= max_col)
        {
            out_candidate_walls[cnt++] = obstacles[i];
        }
    }
    return cnt;
}

/*
 * 路径特殊点标记策略（写入 Position.id）：
 * - IDENTIFICATION : 路径起点（建议在该点做一次姿态/视觉确认）
 * - TURNING_POINT  : 行进方向发生变化的拐点
 * - BOMB_EXPLOSION : 炸弹爆炸对应的事件点（优先级最高）
 */
static int marker_priority(uint8_t marker_id)
{
    if (marker_id == BOMB_EXPLOSION)
        return 3;
    if (marker_id == IDENTIFICATION)
        return 2;
    if (marker_id == TURNING_POINT)
        return 1;
    return 0;
}

static void mark_path_id(Position *path, int path_len, int index, uint8_t marker_id)
{
    if (!path || index < 0 || index >= path_len)
        return;
    if (marker_priority(marker_id) >= marker_priority(path[index].id))
    {
        path[index].id = marker_id;
    }
}

static void annotate_path_special_ids(Position *path, int path_len, int bomb_event_index)
{
    if (!path || path_len <= 0)
        return;

    for (int i = 0; i < path_len; i++)
    {
        path[i].id = 0;
    }

    mark_path_id(path, path_len, 0, IDENTIFICATION);

    for (int i = 1; i < path_len - 1; i++)
    {
        int dr1 = (int)path[i].row - (int)path[i - 1].row;
        int dc1 = (int)path[i].col - (int)path[i - 1].col;
        int dr2 = (int)path[i + 1].row - (int)path[i].row;
        int dc2 = (int)path[i + 1].col - (int)path[i].col;
        if (dr1 != dr2 || dc1 != dc2)
        {
            mark_path_id(path, path_len, i, TURNING_POINT);
        }
    }

    if (bomb_event_index >= 0 && bomb_event_index < path_len)
    {
        mark_path_id(path, path_len, bomb_event_index, BOMB_EXPLOSION);
    }
}

/*
 * 评估单个“炸弹 + 墙”组合：
 * Phase1: 把炸弹当作“可推动物体”推到 wall_target。
 * Phase2: 在炸后地形上推动目标箱子到目标点。
 * 成功返回两阶段总步数，并输出拼接后的完整小车路径。
 */
int evaluate_bomb_shortcut(int row_cnt, int col_cnt,
                           const Position *obstacles, int obstacles_cnt,
                           const Position *bombs, int bombs_cnt,
                           const Position *boxes, int boxes_cnt,
                           const Position *targets, int targets_cnt,
                           int box_index,
                           int bomb_index,
                           Position wall_target,
                           Position car_start,
                           Position *out_full_path,
                           Position *out_bomb_target,
                           Position *out_box_target,
                           int *out_bomb_event_index)
{
    // 把炸弹加入箱子数组进行A*计算
    Position temp_boxes[MAX_BOXES + 1];
    for (int i = 0; i < boxes_cnt; i++)
        temp_boxes[i] = boxes[i];

    int virtual_bomb_box_index = boxes_cnt;
    temp_boxes[virtual_bomb_box_index] = bombs[bomb_index];
    int temp_boxes_cnt = boxes_cnt + 1;

    // 剔除这颗炸弹
    Position temp_bombs[MAX_BOMBS + 2];
    int temp_bombs_cnt = 0;
    for (int i = 0; i < bombs_cnt; i++)
    {
        if (i != bomb_index)
            temp_bombs[temp_bombs_cnt++] = bombs[i];
    }

    // 剔除目标墙壁，以便可以将炸弹直接推入墙壁的坐标内爆破
    Position phase1_obs[grid_size];
    int phase1_obs_cnt = 0;
    for (int i = 0; i < obstacles_cnt; i++)
    {
        if (obstacles[i].row != wall_target.row || obstacles[i].col != wall_target.col)
            phase1_obs[phase1_obs_cnt++] = obstacles[i];
    }

    Position actual_targets[1];
    actual_targets[0] = wall_target;

    Position best_bomb_target;

    // 推炸弹
    int phase1_steps = a_star_path_plan_3d(row_cnt, col_cnt,
                                           phase1_obs, phase1_obs_cnt,
                                           temp_bombs, temp_bombs_cnt,
                                           temp_boxes, temp_boxes_cnt,
                                           actual_targets, 1,
                                           virtual_bomb_box_index,
                                           car_start,
                                           out_full_path,
                                           &best_bomb_target);

    if (phase1_steps < 0)
        return -1;

    // 修改地形
    Position temp_obstacles[grid_size];
    for (int i = 0; i < obstacles_cnt; i++)
        temp_obstacles[i] = obstacles[i];
    int temp_obs_cnt = obstacles_cnt;
    simulate_bomb_explosion(temp_obstacles, &temp_obs_cnt, best_bomb_target);

    // 交接小车坐标
    Position new_car_start = out_full_path[phase1_steps - 1];

    // 推箱子
    int phase2_steps = a_star_path_plan_3d(row_cnt, col_cnt,
                                           temp_obstacles, temp_obs_cnt,
                                           temp_bombs, temp_bombs_cnt,
                                           boxes, boxes_cnt,
                                           targets, targets_cnt,
                                           box_index,
                                           new_car_start,
                                           out_full_path + phase1_steps,
                                           out_box_target);

    if (phase2_steps < 0)
        return -1;

    if (out_bomb_target)
        *out_bomb_target = best_bomb_target;
    if (out_bomb_event_index)
        *out_bomb_event_index = phase1_steps - 1;

    return phase1_steps + phase2_steps;
}

/*
 * 统一对外的单箱规划入口：
 * 1) 先求“直接推箱”路径。
 * 2) 若存在炸弹，则枚举炸弹与候选墙，评估“先炸后推”。
 * 3) 在所有可行方案中取最短路径，回填 path_plan_result。
 */
int integrated_path_output(int row_cnt, int col_cnt,
                           const Position *obstacles, int obstacles_cnt,
                           const Position *bombs, int bombs_cnt,
                           const Position *boxes, int boxes_cnt,
                           const Position *targets, int targets_cnt,
                           int box_index,
                           Position car_start,
                           path_plan_result *result)
{
    if (!result)
        return -1;

    // 初始化结果
    memset(result, 0, sizeof(*result));
    result->total_steps = -1;
    result->bomb_index = -1;
    result->bomb_target = (Position){255, 255, 0};
    result->wall_target = (Position){255, 255, 0};
    result->box_target = (Position){255, 255, 0};

    int best_len = -1;
    int best_bomb_event_index = -1;

    Position simple_path[grid_size * 4];
    Position simple_target;

    int simple_len = a_star_path_plan_3d(row_cnt, col_cnt,
                                         obstacles, obstacles_cnt,
                                         bombs, bombs_cnt,
                                         boxes, boxes_cnt,
                                         targets, targets_cnt,
                                         box_index,
                                         car_start,
                                         simple_path,
                                         &simple_target);

    if (simple_len > 0)
    {
        best_len = simple_len;
        result->total_steps = simple_len;
        result->used_bomb = 0;
        result->box_target = simple_target;
        memcpy(result->car_path, simple_path, (size_t)simple_len * sizeof(Position));
        best_bomb_event_index = -1;
    }

    if (bombs_cnt > 0)
    {
        Position candidate_walls[grid_size];
        int wall_cnt;

        if (simple_len < 0)
        {
            // 直接推失败 → 搜索所有墙壁
            wall_cnt = obstacles_cnt;
            for (int i = 0; i < obstacles_cnt; i++)
            {
                candidate_walls[i] = obstacles[i];
            }
        }
        else
        {
            // 直接推成功 → 只搜索路径区间内的墙
            wall_cnt = get_candidate_walls(obstacles, obstacles_cnt,
                                           boxes[box_index], simple_target,
                                           candidate_walls);
        }

        for (int b = 0; b < bombs_cnt; b++)
        {
            for (int w = 0; w < wall_cnt; w++)
            {
                Position temp_path[grid_size * 8];
                Position actual_bomb_target;
                Position actual_box_target;
                int bomb_event_index = -1;
                int bomb_steps = evaluate_bomb_shortcut(row_cnt, col_cnt,
                                                        obstacles, obstacles_cnt,
                                                        bombs, bombs_cnt,
                                                        boxes, boxes_cnt,
                                                        targets, targets_cnt,
                                                        box_index, b,
                                                        candidate_walls[w],
                                                        car_start,
                                                        temp_path,
                                                        &actual_bomb_target,
                                                        &actual_box_target,
                                                        &bomb_event_index);

                if (bomb_steps > 0 && (best_len < 0 || bomb_steps < best_len))
                {
                    best_len = bomb_steps;
                    result->total_steps = bomb_steps;
                    result->used_bomb = 1;
                    result->bomb_index = b;
                    result->wall_target = candidate_walls[w];
                    result->bomb_target = actual_bomb_target;
                    result->box_target = actual_box_target;
                    memcpy(result->car_path, temp_path, (size_t)bomb_steps * sizeof(Position));
                    best_bomb_event_index = bomb_event_index;
                }
            }
        }
    }

    if (result->total_steps > 0)
    {
        annotate_path_special_ids(result->car_path, result->total_steps, best_bomb_event_index);
    }

    return result->total_steps;
}
