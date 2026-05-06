#include "Algorithm.h"
#include <string.h>

/*
 * Algorithm.c：底层路径搜索模块
 * 1) 2D A*：用于小车在静态地图中的最短路；
 * 2) 3D A*：把“箱子位置 + 推箱朝向”纳入状态，规划完整推箱过程；
 * 3) 炸弹捷径评估：评估“先推炸弹炸墙，再推箱”是否更优；
 * 4) 综合输出：统一比较直推与炸墙方案，输出最优完整路径。
 *
 * 约定：
 * - 返回值 <= 0 表示失败或不可达，> 0 表示路径步数；
 * - Position 使用 (row, col, id) 网格坐标；
 * - 大部分函数为 static，仅供本文件内部组合调用。
 */

/*
 * 性能优化要点：
 * 1) 2D/3D 堆结构维护“节点在堆内位置”，把 update 从 O(N) 降到 O(logN)；
 * 2) 炸弹评估采用多层候选筛选与曼哈顿下界剪枝，减少高代价搜索次数。
 */
static int heap_pos_2d[grid_size];
static int heap_pos_3d[grid_size * 4];

/* 快速炸弹评估参数：用于限制“炸弹捷径”搜索规模，降低总计算量。 */
#define FAST_BOMB_SHORTCUT_TRIGGER_STEPS      26
#define FAST_BOMB_MAX_BOMBS_WHEN_REACHABLE    3
#define FAST_BOMB_MAX_WALLS_WHEN_REACHABLE    8
#define FAST_BOMB_MAX_WALLS_WHEN_BLOCKED      18

static int abs_i(int x)
{
    return (x < 0) ? -x : x;
}

static int manhattan_dist(Position a, Position b)
{
    return abs_i((int)a.row - (int)b.row) + abs_i((int)a.col - (int)b.col);
}

static int min_target_dist(Position from, const Position *targets, int targets_cnt)
{
    int i;
    int best = 0x3fffffff;
    if (!targets || targets_cnt <= 0)
        return 0;

    for (i = 0; i < targets_cnt; i++)
    {
        int d = manhattan_dist(from, targets[i]);
        if (d < best)
            best = d;
    }
    return (best == 0x3fffffff) ? 0 : best;
}

static int wall_priority_score(Position wall,
                               Position box,
                               const Position *targets,
                               int targets_cnt,
                               Position car_start)
{
    int i;
    int best_target = 0x3fffffff;

    for (i = 0; i < targets_cnt; i++)
    {
        int d = manhattan_dist(wall, targets[i]);
        if (d < best_target)
            best_target = d;
    }
    if (targets_cnt <= 0)
        best_target = 0;

    return manhattan_dist(wall, box) + best_target + (manhattan_dist(wall, car_start) >> 1);
}

static int select_top_walls_by_score(const Position *walls,
                                     int wall_cnt,
                                     int limit,
                                     Position box,
                                     const Position *targets,
                                     int targets_cnt,
                                     Position car_start,
                                     Position *out_walls)
{
    uint8_t used[grid_size];
    int picked = 0;
    int k, i;

    if (!walls || !out_walls || wall_cnt <= 0 || limit <= 0)
        return 0;
    if (wall_cnt > grid_size)
        wall_cnt = grid_size;
    if (limit > wall_cnt)
        limit = wall_cnt;

    memset(used, 0, sizeof(used));

    for (k = 0; k < limit; k++)
    {
        int best_idx = -1;
        int best_score = 0x3fffffff;
        for (i = 0; i < wall_cnt; i++)
        {
            int score;
            if (used[i])
                continue;
            score = wall_priority_score(walls[i], box, targets, targets_cnt, car_start);
            if (score < best_score)
            {
                best_score = score;
                best_idx = i;
            }
        }
        if (best_idx < 0)
            break;
        used[best_idx] = 1;
        out_walls[picked++] = walls[best_idx];
    }

    return picked;
}

static int select_top_bomb_indices_by_score(const Position *bombs,
                                            int bombs_cnt,
                                            int limit,
                                            Position box,
                                            Position car_start,
                                            int *out_indices)
{
    uint8_t used[MAX_BOMBS];
    int picked = 0;
    int k, i;

    if (!bombs || !out_indices || bombs_cnt <= 0 || limit <= 0)
        return 0;
    if (bombs_cnt > MAX_BOMBS)
        bombs_cnt = MAX_BOMBS;
    if (limit > bombs_cnt)
        limit = bombs_cnt;

    memset(used, 0, sizeof(used));

    for (k = 0; k < limit; k++)
    {
        int best_idx = -1;
        int best_score = 0x3fffffff;
        for (i = 0; i < bombs_cnt; i++)
        {
            int score;
            if (used[i])
                continue;
            score = manhattan_dist(bombs[i], box) + (manhattan_dist(bombs[i], car_start) >> 1);
            if (score < best_score)
            {
                best_score = score;
                best_idx = i;
            }
        }
        if (best_idx < 0)
            break;
        used[best_idx] = 1;
        out_indices[picked++] = best_idx;
    }

    return picked;
}

/*
 * 灏嗛殰纰嶇墿/鐐稿脊/绠卞瓙鍐欏叆涓€缁?grid銆? * grid 姣忎釜鍗曞厓鏄綅鏍囪锛? * - 浣?4 浣嶈褰曟槸鍚︿负闅滅銆佺偢寮广€佺瀛愮瓑绫诲埆銆? * - 楂?4 浣嶅湪绠卞瓙鍦烘櫙涓嬭褰曗€滅瀛愮储寮曗€?0~15锛岃秴鍑烘埅鏂埌 15)銆? */
static void grid_build(int row_cnt, int col_cnt,
                       const Position *obstacles, int obstacles_cnt,
                       const Position *bombs, int bombs_cnt,
                       const Position *boxes, int boxes_cnt,
                       uint8_t *grid)
{
    // 纭畾缃戞牸澶у皬
    int n = row_cnt * col_cnt;
    if (n > grid_size)
        n = grid_size;
    // 娓呯┖缃戞牸
    memset(grid, 0, n);
    // 鏍囪鍚勫潡
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
/* 妫€鏌ュ崟鍏冩槸鍚︿笉鍙€氳锛氶殰纰?/ 鐐稿脊 / 宸插皝閿佺偢寮广€?*/
static int check_obstacle(uint8_t *grid, int col_cnt, int row, int col)
{
    if (row < 0 || col < 0)
        return 0;
    return ((grid[col_cnt * row + col] & (OBSTACLE | BOMB | BLOCKED_BOMB)) != 0);
}
/*
 * 鍒ゆ柇鏌愪綅缃槸鍚﹀瓨鍦ㄢ€滈櫎 skip_index 澶栫殑绠卞瓙鈥濄€? * 鐢ㄤ簬鈥滄鍦ㄦ帹鍔ㄧ skip_index 涓瀛愨€濇椂锛屽拷鐣ュ畠鑷韩鍗犱綅锛屼粎妫€鏌ュ叾浠栫瀛愮鎾炪€? */
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
/* BOX_EXCLUDING 鐨勮竟鐣屽寘瑁呯増鏈€?*/
static int check_box_with_excluding(uint8_t *grid, int col_cnt, int row, int col, size_t skip_index)
{
    if (row < 0 || col < 0)
        return 0;
    return (BOX_EXCLUDING(grid, col_cnt, row, col, skip_index));
}
/*
 * 鍒ゆ柇鈥滅瀛愪笅涓€姝ヨ惤鐐光€濇槸鍚﹁闃诲锛? * - 瓒婄晫闃诲
 * - 缃戞牸闅滅闃诲
 * - 涓庡叾浠栫瀛愰噸鍙犻樆濉烇紙鎺掗櫎褰撳墠姝ｅ湪鎺ㄥ姩鐨勭瀛愶級
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
 * 浠ヤ笅鏄?2D A* 浣跨敤鐨勬渶灏忓爢宸ュ叿锛? * - 浠?f_cost 涓轰富鎺掑簭閿紝h_cost 涓烘鎺掑簭閿紙鏇存帴杩戠粓鐐硅€呬紭鍏堬級銆? * - 鐢ㄦ暟缁勫疄鐜颁簩鍙夊爢锛岄檷浣?open 闆嗗悎鍙栨渶灏忎唬浠疯妭鐐圭殑澶嶆潅搴︺€? */

/* 浜ゆ崲鍫嗘暟缁勪腑鐨勪袱涓Ы浣嶃€?*/
static inline void heap_swap(binary_heap *heap, int i, int j)
{
    int node_i = heap->index[i];
    int node_j = heap->index[j];
    heap->index[i] = node_j;
    heap->index[j] = node_i;
    heap_pos_2d[node_i] = j;
    heap_pos_2d[node_j] = i;
}
/* 姣旇緝鍫嗗唴涓や釜鑺傜偣浼樺厛绾с€?*/
static inline int heap_compare_less(const a_star_param *nodes, const binary_heap *heap, int i, int j)
{
    int index_i = heap->index[i];
    int index_j = heap->index[j];
    if (nodes[index_i].f_cost != nodes[index_j].f_cost)
        return nodes[index_i].f_cost < nodes[index_j].f_cost;

    return nodes[index_i].h_cost < nodes[index_j].h_cost;
}

/* 鍫嗕笂娴細鐢ㄤ簬鏂版彃鍏ユ垨浠ｄ环涓嬮檷鍚庣殑閲嶆帓銆?*/
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
/* 鍫嗕笅娌夛細鐢ㄤ簬寮瑰嚭鍫嗛《鍚庢妸灏惧厓绱犳斁鍒版牴骞舵仮澶嶅爢搴忋€?*/
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

/* 鍘嬪叆 open 闆嗗悎銆?*/
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
/* 寮瑰嚭 open 闆嗗悎涓渶浼樿妭鐐广€?*/
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

/* 鑺傜偣宸叉湁鏇翠紭 g_cost 鏃讹紝瑙﹀彂鍫嗗唴浣嶇疆鏇存柊銆?*/
static void heap_update(a_star_param *nodes, binary_heap *heap, int node_index)
{
    if (node_index < 0 || node_index >= grid_size)
        return;
    if (heap_pos_2d[node_index] < 0)
        return;
    heap_sift_up(nodes, heap, heap_pos_2d[node_index]);
}

/* 鍏柟鍚戣繎浼煎惎鍙戯細鐩寸Щ浠ｄ环 10锛屾枩绉讳唬浠?14銆?*/
static inline int diagonal_distance(Position p1, Position p2)
{
    int dif_row = p1.row > p2.row ? p1.row - p2.row : p2.row - p1.row;
    int dif_col = p1.col > p2.col ? p1.col - p2.col : p2.col - p1.col;
    int min_dif = dif_row < dif_col ? dif_row : dif_col;
    int max_dif = dif_row > dif_col ? dif_row : dif_col;
    // 鐩磋蛋鏉冮噸10锛屾枩璧版潈閲?4
    return 14 * min_dif + 10 * (max_dif - min_dif);
}
/* 鏇煎搱椤胯窛绂伙細鐢ㄤ簬绠卞瓙鎺ㄩ€侀樁娈电殑鍚彂浼拌銆?*/
static inline int manhattan_distance_cells(Position p1, Position p2)
{
    int dif_row = p1.row > p2.row ? p1.row - p2.row : p2.row - p1.row;
    int dif_col = p1.col > p2.col ? p1.col - p2.col : p2.col - p1.col;
    return dif_row + dif_col;
}

/* 2D A* 鑺傜偣姹犮€?*/
a_star_param a_star[grid_size];

/*
 * 2D A*锛氬皬杞︿粠 start 鍒?target 鐨勮璧拌矾寰勮鍒掋€? * allow_diagonal 璇箟锛? * - 1锛氱┖杞﹀彲 8 鍚戠Щ鍔紝浣嗛潬杩戠瀛愭椂鑷姩闄嶇骇涓?4 鍚戦槻纰版挒銆? * - 0锛氬己鍒?4 鍚戯紙鍏稿瀷鐢ㄤ簬鎺ㄧ鏃舵洿绋冲畾鐨勬爡鏍煎姩浣滐級銆? */
static int a_star_path_plan(int row_cnt, int col_cnt,
                            const Position *obstacles, int obstacles_cnt,
                            const Position *bombs, int bombs_cnt,
                            const Position *boxes, int boxes_cnt,
                            const Position start, const Position target,
                            int allow_diagonal,
                            Position *out_path)
{
    // Validate start/target.
    if (start.row >= row_cnt || start.col >= col_cnt)
        return -1;
    if (target.row >= row_cnt || target.col >= col_cnt)
        return -1;
    // Build occupancy grid.
    uint8_t grid[grid_size];
    grid_build(row_cnt, col_cnt, obstacles, obstacles_cnt, bombs, bombs_cnt, boxes, boxes_cnt, grid);

    // 鍒濆鍖?A* 鐨勮妭鐐硅闂粍鏍囪瘑
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

    // 8鍚戠Щ鍔ㄦ柟鍚?(0-3: 鐩磋姝ｄ氦鍚? 4-7: 瀵硅鏂滃悜)
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
            // 宸叉壘鍒拌矾寰勶紝鍚戝鍥炴函
            int path_len = 0;
            int curr = target_index;
            while (curr != -1)
            {
                out_path[path_len].row = curr / col_cnt;
                out_path[path_len].col = curr % col_cnt;
                path_len++;
                curr = a_star[curr].parent_index;
            }
            // Reverse path because backtracking is from target to start.
            for (int i = 0; i < path_len / 2; i++)
            {
                Position temp = out_path[i];
                out_path[i] = out_path[path_len - 1 - i];
                out_path[path_len - 1 - i] = temp;
            }
            return path_len; // 姝ｅ父杩斿洖璺緞姝ユ暟
        }

        // 褰撳墠鏍兼槸鍚﹂潬杩戠瀛愶紙棰勮绠楁煡琛紝閬垮厤寰幆鍐呴噸澶嶆壂鎻忓叏閮ㄧ瀛愶級
        int is_near_box = allow_diagonal ? near_box_map[current_index] : 0;

        // 褰撳懆杈规湁绠卞瓙鏃讹紝鎴栦笉鍏佽鏂滅┛(鎺ㄧ妯″紡)鏃讹紝寮哄埗闄嶇骇涓?4 鍚戠Щ鍔ㄩ槻纰帮紱鍦ㄧ┖鏃峰湴甯﹀垯鍏ㄥ紑 8 鍚戜互鍔犲揩瀵昏矾鍜岀Щ鍔ㄩ€熷害
        int dir_count = (!allow_diagonal || is_near_box) ? 4 : 8;
        // Expand neighbors.
        for (int i = 0; i < dir_count; i++)
        {
            int nr = r + dir_row[i];
            int nc = c + dir_col[i];

            if (nr >= 0 && nr < row_cnt && nc >= 0 && nc < col_cnt)
            {
                // 濡傛灉鏄瑙掔嚎绉诲姩(i >= 4)锛屽垯寮哄埗杩涜"绂佹鍒囪"楠岃瘉
                if (i >= 4)
                {
                    // 姝ｄ氦涓よ竟鍒嗗埆涓?(r + dr[i], c) 鍜?(r, c + dc[i])
                    // 纭繚涓婃柟/涓嬫柟 鍜?宸︽柟/鍙虫柟 閮芥槸绌鸿矾锛岄槻姝㈡枩绌垮瑙?or 绠卞瓙
                    if (check_obstacle(grid, col_cnt, nr, c) || (grid[nr * col_cnt + c] & BOX) ||
                        check_obstacle(grid, col_cnt, r, nc) || (grid[r * col_cnt + nc] & BOX))
                        continue;
                }

                int neighbor_index = nr * col_cnt + nc;

                // 鍒╃敤宸叉湁鏂规硶妫€娴嬬洰鏍囨牸鏈韩鏄惁鍙€氳繃缃戞牸 (鍖呮嫭澧欏拰绠卞瓙)
                if (check_obstacle(grid, col_cnt, nr, nc) || (grid[neighbor_index] & BOX))
                    continue;

                // 濡傛灉宸茬粡鍔犲叆浜?close 闆嗗悎
                if (a_star[neighbor_index].open_or_close == 2)
                    continue;

                int tentative_g_cost = a_star[current_index].g_cost + move_cost[i]; // 绱姞绉诲姩浠ｄ环鍊?10 鎴?14

                if (a_star[neighbor_index].open_or_close == 0)
                { // unvisited (姝ゅ墠鏈璁块棶杩?
                    a_star[neighbor_index].parent_index = current_index;
                    a_star[neighbor_index].g_cost = tentative_g_cost;
                    a_star[neighbor_index].h_cost = diagonal_distance((Position){nr, nc}, target);
                    a_star[neighbor_index].f_cost = a_star[neighbor_index].g_cost + a_star[neighbor_index].h_cost;
                    a_star[neighbor_index].open_or_close = 1;
                    heap_push(a_star, &open_set, neighbor_index);
                }
                else if (tentative_g_cost < a_star[neighbor_index].g_cost)
                {
                    // Already in open set, but found a better route.
                    a_star[neighbor_index].parent_index = current_index;
                    a_star[neighbor_index].g_cost = tentative_g_cost;
                    a_star[neighbor_index].f_cost = a_star[neighbor_index].g_cost + a_star[neighbor_index].h_cost;
                                        heap_update(a_star, &open_set, neighbor_index);
                }
            }
        }
    }
    return -1; // 瀵绘壘涓嶅埌璺緞
}

/*
 * 3D A* 鐘舵€佸畾涔夛細
 * state = (box_row, box_col, face)
 * - face 琛ㄧず灏忚溅绔欏湪绠卞瓙鍝竴渚у苟鍑嗗鍚戝摢涓柟鍚戞帹銆? * - 鍚屼竴涓瀛愭牸瀛愭湁 4 涓湞鍚戠姸鎬侊紝鍥犳瀹归噺鏄?grid_size * 4銆? */
a_star_3d_param a_star_3d[grid_size * 4];

/* 3D A* 鐨勫爢浜ゆ崲銆?*/
static inline void heap_swap_3d(binary_heap_3d *heap, int i, int j)
{
    int node_i = heap->index[i];
    int node_j = heap->index[j];
    heap->index[i] = node_j;
    heap->index[j] = node_i;
    heap_pos_3d[node_i] = j;
    heap_pos_3d[node_j] = i;
}
/* 3D A* 鐨勪紭鍏堢骇姣旇緝锛歠 涓轰富锛実 鍜?h 涓鸿緟銆?*/
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

/* 3D A* 鍫嗕笂娴€?*/
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
/* 3D A* 鍫嗕笅娌夈€?*/
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

/* 3D A* 鍏ュ爢銆?*/
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
/* 3D A* 鍑哄爢銆?*/
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

/* 3D A* 鑺傜偣浠ｄ环鏀硅繘鍚庣殑閲嶆帓銆?*/
static void heap_update_3d(binary_heap_3d *heap, int node_index)
{
    if (node_index < 0 || node_index >= (grid_size * 4))
        return;
    if (heap_pos_3d[node_index] < 0)
        return;
    heap_sift_up_3d(heap, heap_pos_3d[node_index]);
}

/*
 * 3D A*锛堟帹绠变富绠楁硶锛夛細
 * - 璧风偣锛氬厛鏋氫妇 4 涓彲鑳芥帹闈紝绛涘嚭灏忚溅鍙埌杈剧殑鍏ユ帹濮挎€併€? * - 鎵╁睍锛? *   1) 鍚屽悜鐩存帹锛氱瀛愬墠杩涗竴姝ワ紝face 涓嶅彉銆? *   2) 缁曡鎹㈠悜锛氱瀛愪笉鍔紝灏忚溅缁曞埌鍙︿竴渚э紝face 鏀瑰彉銆? * - 浠ｄ环璁捐锛? *   COST_PUSH > COST_WALK锛岄紦鍔卞噺灏戞棤鏁堣蛋鍔紱
 *   COST_REORIENT_PENALTY 绾︽潫棰戠箒鎹㈠悜銆? * - 杈撳嚭锛氳繑鍥炲畬鏁粹€滃皬杞﹁矾寰勨€濓紝涓嶆槸浠呯瀛愯建杩广€? */
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
    grid_build(row_cnt, col_cnt, obstacles, obstacles_cnt, bombs, bombs_cnt, local_boxes, boxes_cnt, grid);

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

    // 浠?涓柟鍚戜腑閫夋嫨鎺ㄥ悜
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
        // Compute path from car to push point.
        int walk_len = a_star_path_plan(row_cnt, col_cnt,
                                        obstacles, obstacles_cnt, bombs, bombs_cnt,
                                        local_boxes, boxes_cnt,
                                        car_start, (Position){push_point_row, push_point_col},
                                        1,
                                        temp_path);

        if (walk_len >= 0)
        {
            // If car can reach this push face, initialize that state.
            int state_index = (box_start.row * col_cnt + box_start.col) * 4 + f;

            // 璁＄畻鍒拌揪鐩爣鐐圭殑璺濈
            a_star_3d[state_index].g_cost = walk_len * COST_WALK;
            a_star_3d[state_index].h_cost = min_target_distance_from_start * COST_PUSH; // 璋冩暣姣斾緥锛屼紭鍏堣€冭檻绠卞瓙缁堢偣
            a_star_3d[state_index].f_cost = a_star_3d[state_index].g_cost + a_star_3d[state_index].h_cost;
            a_star_3d[state_index].parent_index = -1;
            a_star_3d[state_index].open_or_close = 1;
            a_star_3d[state_index].is_push = 0; // 娌℃帹杩囷紝鍒氳浆绉诲埌鎺ㄤ綅

            heap_push_3d(&open_set, state_index);
        }
    }

    while (open_set.size > 0)
    {
        int curr_index = heap_pop_3d(&open_set);
        if (curr_index == -1)
            break;

        a_star_3d[curr_index].open_or_close = 2; // Close
        int row = (curr_index / 4) / col_cnt; // box row
        int col = (curr_index / 4) % col_cnt; // box col
        int face = curr_index % 4;            // push face
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
        } // 鎺ㄥ畬璺冲嚭
        // Push box forward with same face.
        int next_row = row + dir_row_3d[face];
        int next_col = col + dir_col_3d[face];

        if (!check_push_destination_blocked(grid, row_cnt, col_cnt,
                                            local_boxes, boxes_cnt,
                                            box_index,
                                            next_row, next_col))
        {
            int next_index = (next_row * col_cnt + next_col) * 4 + face; // face涓嶅彉锛屾病鎹㈡帹闈?
            if (a_star_3d[next_index].open_or_close != 2)
            {
                int tentative_g = a_star_3d[curr_index].g_cost + COST_PUSH; // 绂诲紑鍘熻矾寰勶紝娣诲姞鍋忕Щ浠ｄ环

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
                    a_star_3d[next_index].is_push = 1; // 鐩磋蛋

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
        // Reposition car to another push face without moving box.
        for (int next_face = 0; next_face < 4; next_face++)
        {
            if (next_face == face)
                continue; // 璺宠繃鐩稿悓鏂瑰悜

            int next_index = (row * col_cnt + col) * 4 + next_face; // 鍙敼鍙樹簡鎺ㄥ悜
            if (a_star_3d[next_index].open_or_close == 2)
                continue; // 璺宠繃宸插叧闂殑璺緞

            // 璁＄畻灏忚溅闇€瑕佺粫鍒扮殑浣嶇疆
            int target_face_row = row - dir_row_3d[next_face];
            int target_face_col = col - dir_col_3d[next_face];

            if (target_face_row < 0 || target_face_row >= row_cnt || target_face_col < 0 || target_face_col >= col_cnt)
                continue;
            local_boxes[box_index] = (Position){row, col}; // 灏嗗綋鍓嶈鎺ㄥ姩瀵硅薄鏀惧湪瀹炴椂浣嶇疆
            // Reorientation occupancy check.
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
                    a_star_3d[next_index].h_cost = a_star_3d[curr_index].h_cost; // 绠卞瓙娌″姩
                    a_star_3d[next_index].f_cost = tentative_g + a_star_3d[next_index].h_cost;
                    a_star_3d[next_index].is_push = 0; // 鎹㈠悜鎺?
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

    // 鍏堣灏忚溅璺戝埌鎺ㄧ偣
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

        int curr_f = curr_index % 4;

        if (a_star_3d[curr_index].is_push == 1)
        {
            // 鐩存帹
            full_car_path[total_car_len++] = (Position){prev_r, prev_c};
        }
        else
        {
            // Walk around for reorientation.
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
 * 模拟炸弹爆炸：
 * 以 bomb_target 为中心，清除 3x3 范围内障碍物。 */
void simulate_bomb_explosion(Position *obstacles, int *obstacles_cnt, Position bomb_target)
{
    int new_cnt = 0;
    for (int i = 0; i < *obstacles_cnt; i++)
    {
        int dr = obstacles[i].row - bomb_target.row;
        int dc = obstacles[i].col - bomb_target.col;
        // 濡傛灉鍦?3x3 涔濆鏍煎唴锛屽澹佽鐐告瘉
        if (dr >= -1 && dr <= 1 && dc >= -1 && dc <= 1)
            continue;
        obstacles[new_cnt++] = obstacles[i];
    }
    // Clear remaining slots.
    for (int i = new_cnt; i < *obstacles_cnt; i++)
    {
        obstacles[i].row = -1;
        obstacles[i].col = -1;
    }
    *obstacles_cnt = new_cnt;
}

/*
 * 筛选“可能值得炸”的候选墙：
 * 仅在箱子起点与目标点围成的局部窗口（外扩 1 格）中取墙，
 * 用于降低炸弹组合搜索规模。 */
int get_candidate_walls(const Position *obstacles, int obstacles_cnt,
                        Position box_start, Position target,
                        Position *out_candidate_walls)
{
    int cnt = 0;
    int min_row = box_start.row < target.row ? box_start.row : target.row;
    int max_row = box_start.row > target.row ? box_start.row : target.row;
    int min_col = box_start.col < target.col ? box_start.col : target.col;
    int max_col = box_start.col > target.col ? box_start.col : target.col;
    // Expand search window outward by 1 cell.
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
 * - TURNING_POINT：发生转向的关键点；
 * - BOMB_EXPLOSION：炸弹爆破事件点（优先级最高）。 */
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
 * 评估一个“炸弹 + 墙点”组合的总代价：
 * Phase1：把 bomb_index 对应炸弹当作可推对象，推到 wall_target；
 * Phase2：在爆破后的新地图上继续推目标箱子到目标点；
 * 成功时返回两阶段总步数，并输出拼接后的完整小车路径。 */
int evaluate_bomb_shortcut(int row_cnt, int col_cnt,
                           const Position *obstacles, int obstacles_cnt,
                           const Position *bombs, int bombs_cnt,
                           const Position *boxes, int boxes_cnt,
                           const Position *targets, int targets_cnt,
                           int box_index,
                           int bomb_index,
                           Position wall_target,
                           int max_total_steps,
                           Position car_start,
                           Position *out_full_path,
                           Position *out_bomb_target,
                           Position *out_box_target,
                           int *out_bomb_event_index)
{
    // 鎶婄偢寮瑰姞鍏ョ瀛愭暟缁勮繘琛孉*璁＄畻
    Position temp_boxes[MAX_BOXES + 1];
    for (int i = 0; i < boxes_cnt; i++)
        temp_boxes[i] = boxes[i];

    int virtual_bomb_box_index = boxes_cnt;
    temp_boxes[virtual_bomb_box_index] = bombs[bomb_index];
    int temp_boxes_cnt = boxes_cnt + 1;

    // 鍓旈櫎杩欓鐐稿脊
    Position temp_bombs[MAX_BOMBS + 2];
    int temp_bombs_cnt = 0;
    for (int i = 0; i < bombs_cnt; i++)
    {
        if (i != bomb_index)
            temp_bombs[temp_bombs_cnt++] = bombs[i];
    }

    // 鍓旈櫎鐩爣澧欏锛屼互渚垮彲浠ュ皢鐐稿脊鐩存帴鎺ㄥ叆澧欏鐨勫潗鏍囧唴鐖嗙牬
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
    // Phase 1: push bomb to the chosen wall.
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
    if (max_total_steps > 0 && phase1_steps >= max_total_steps)
        return -1;

    // 淇敼鍦板舰
    Position temp_obstacles[grid_size];
    for (int i = 0; i < obstacles_cnt; i++)
        temp_obstacles[i] = obstacles[i];
    int temp_obs_cnt = obstacles_cnt;
    simulate_bomb_explosion(temp_obstacles, &temp_obs_cnt, best_bomb_target);

    // 浜ゆ帴灏忚溅鍧愭爣
    Position new_car_start = out_full_path[phase1_steps - 1];
    // Phase 2: push target box on updated terrain.
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
    if (max_total_steps > 0 && (phase1_steps + phase2_steps) >= max_total_steps)
        return -1;

    if (out_bomb_target)
        *out_bomb_target = best_bomb_target;
    if (out_bomb_event_index)
        *out_bomb_event_index = phase1_steps - 1;

    return phase1_steps + phase2_steps;
}

/*
 * 对外统一入口：单箱子综合规划
 * 1) 先求直推方案；
 * 2) 若存在炸弹，再评估“先炸后推”方案；
 * 3) 返回所有可行方案中的最短路径。 */
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
    /* 初始化输出结构体，默认不可达。 */
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
        Position eval_walls[grid_size];
        int eval_bomb_indices[MAX_BOMBS];
        int box_to_target_lb = min_target_dist(boxes[box_index], targets, targets_cnt);
        int wall_cnt;
        int eval_wall_cnt = 0;
        int eval_bomb_cnt = 0;
        int enable_bomb_search = 1;

        if (simple_len < 0)
        {
            /* 直推失败：先用“箱子-最近目标”的局部墙候选，避免全图暴力枚举。 */
            Position nearest_target = targets[0];
            int nearest_dist = 0x3fffffff;
            for (int ti = 0; ti < targets_cnt; ti++)
            {
                int d = manhattan_dist(boxes[box_index], targets[ti]);
                if (d < nearest_dist)
                {
                    nearest_dist = d;
                    nearest_target = targets[ti];
                }
            }
            wall_cnt = get_candidate_walls(obstacles, obstacles_cnt,
                                           boxes[box_index], nearest_target,
                                           candidate_walls);
            if (wall_cnt <= 0)
            {
                wall_cnt = obstacles_cnt;
                if (wall_cnt > grid_size)
                    wall_cnt = grid_size;
                for (int i = 0; i < wall_cnt; i++)
                {
                    candidate_walls[i] = obstacles[i];
                }
            }
        }
        else
        {
            wall_cnt = get_candidate_walls(obstacles, obstacles_cnt,
                                           boxes[box_index], simple_target,
                                           candidate_walls);
            if (simple_len <= FAST_BOMB_SHORTCUT_TRIGGER_STEPS)
            {
                /* 直推已经很短时，跳过炸弹搜索以节省算力。 */
                enable_bomb_search = 0;
            }
        }

        if (enable_bomb_search && wall_cnt > 0)
        {
            int wall_limit = (simple_len < 0) ? FAST_BOMB_MAX_WALLS_WHEN_BLOCKED
                                              : FAST_BOMB_MAX_WALLS_WHEN_REACHABLE;
            int bomb_limit = (simple_len < 0) ? bombs_cnt
                                              : FAST_BOMB_MAX_BOMBS_WHEN_REACHABLE;

            eval_wall_cnt = select_top_walls_by_score(candidate_walls,
                                                      wall_cnt,
                                                      wall_limit,
                                                      boxes[box_index],
                                                      targets,
                                                      targets_cnt,
                                                      car_start,
                                                      eval_walls);
            if (eval_wall_cnt <= 0)
                eval_wall_cnt = wall_cnt;
            if (eval_wall_cnt == wall_cnt)
            {
                memcpy(eval_walls, candidate_walls, (size_t)wall_cnt * sizeof(Position));
            }

            eval_bomb_cnt = select_top_bomb_indices_by_score(bombs,
                                                             bombs_cnt,
                                                             bomb_limit,
                                                             boxes[box_index],
                                                             car_start,
                                                             eval_bomb_indices);
            if (eval_bomb_cnt <= 0)
            {
                for (int i = 0; i < bombs_cnt; i++)
                    eval_bomb_indices[i] = i;
                eval_bomb_cnt = bombs_cnt;
            }

            for (int bi = 0; bi < eval_bomb_cnt; bi++)
            {
                int b = eval_bomb_indices[bi];
                for (int w = 0; w < eval_wall_cnt; w++)
                {
                    int car_to_bomb_lb = manhattan_dist(car_start, bombs[b]);
                    int bomb_to_wall_lb = manhattan_dist(bombs[b], eval_walls[w]);
                    int shortcut_lb;

                    if (car_to_bomb_lb > 0)
                        car_to_bomb_lb -= 1;
                    shortcut_lb = car_to_bomb_lb + bomb_to_wall_lb + box_to_target_lb;
                    if (best_len > 0 && shortcut_lb >= best_len)
                        continue;

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
                                                            eval_walls[w],
                                                            best_len,
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
                        result->wall_target = eval_walls[w];
                        result->bomb_target = actual_bomb_target;
                        result->box_target = actual_box_target;
                        memcpy(result->car_path, temp_path, (size_t)bomb_steps * sizeof(Position));
                        best_bomb_event_index = bomb_event_index;
                    }
                }
            }

            /* 保底：若快速筛选失败，再做一次受限扩展枚举，兼顾可达性与耗时。 */
            if (simple_len < 0 && best_len < 0 &&
                (eval_wall_cnt < wall_cnt || eval_bomb_cnt < bombs_cnt))
            {
                Position fallback_walls[grid_size];
                int fallback_bomb_indices[MAX_BOMBS];
                int fallback_wall_cnt = wall_cnt;
                int fallback_bomb_cnt = bombs_cnt;
                int full_combo = bombs_cnt * wall_cnt;

                if (full_combo > 72)
                {
                    fallback_wall_cnt = select_top_walls_by_score(candidate_walls,
                                                                  wall_cnt,
                                                                  24,
                                                                  boxes[box_index],
                                                                  targets,
                                                                  targets_cnt,
                                                                  car_start,
                                                                  fallback_walls);
                    if (fallback_wall_cnt <= 0 || fallback_wall_cnt > wall_cnt)
                    {
                        fallback_wall_cnt = wall_cnt;
                        memcpy(fallback_walls, candidate_walls, (size_t)wall_cnt * sizeof(Position));
                    }

                    fallback_bomb_cnt = select_top_bomb_indices_by_score(bombs,
                                                                          bombs_cnt,
                                                                          6,
                                                                          boxes[box_index],
                                                                          car_start,
                                                                          fallback_bomb_indices);
                    if (fallback_bomb_cnt <= 0 || fallback_bomb_cnt > bombs_cnt)
                    {
                        fallback_bomb_cnt = bombs_cnt;
                        for (int i = 0; i < bombs_cnt; i++)
                            fallback_bomb_indices[i] = i;
                    }
                }
                else
                {
                    memcpy(fallback_walls, candidate_walls, (size_t)wall_cnt * sizeof(Position));
                    for (int i = 0; i < bombs_cnt; i++)
                        fallback_bomb_indices[i] = i;
                }

                for (int bi = 0; bi < fallback_bomb_cnt; bi++)
                {
                    int b = fallback_bomb_indices[bi];
                    for (int w = 0; w < fallback_wall_cnt; w++)
                    {
                        int car_to_bomb_lb = manhattan_dist(car_start, bombs[b]);
                        int bomb_to_wall_lb = manhattan_dist(bombs[b], fallback_walls[w]);
                        int shortcut_lb;

                        if (car_to_bomb_lb > 0)
                            car_to_bomb_lb -= 1;
                        shortcut_lb = car_to_bomb_lb + bomb_to_wall_lb + box_to_target_lb;
                        if (best_len > 0 && shortcut_lb >= best_len)
                            continue;

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
                                                                fallback_walls[w],
                                                                best_len,
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
                            result->wall_target = fallback_walls[w];
                            result->bomb_target = actual_bomb_target;
                            result->box_target = actual_box_target;
                            memcpy(result->car_path, temp_path, (size_t)bomb_steps * sizeof(Position));
                            best_bomb_event_index = bomb_event_index;
                        }
                    }
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

