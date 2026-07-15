#include "TopGrid.h"

#include <math.h>
#include <string.h>

#define TOP_LINE_HALF_EXTENT 0.999f
#define TOP_FLOAT_EPS 1.0e-6f

typedef struct
{
    uint8_t cell[TOP_CELL_COUNT];
    int16_t position[TOP_CELL_COUNT];
    uint16_t size;
    const uint32_t *distance;
} top_cell_heap_t;

static int top_wall_test(const top_wall_bits_t *walls, uint8_t index)
{
    uint8_t word;
    uint8_t bit;

    if (walls == NULL || index >= TOP_CELL_COUNT)
    {
        return 0;
    }
    word = (uint8_t)(index >> 5);
    bit = (uint8_t)(index & 31U);
    return (walls->word[word] & (UINT32_C(1) << bit)) != 0U;
}

int top_cell_valid(top_cell_t cell)
{
    return cell.row >= 0 && cell.row < (int8_t)TOP_ROWS &&
           cell.col >= 0 && cell.col < (int8_t)TOP_COLS;
}

uint8_t top_cell_index(top_cell_t cell)
{
    return (uint8_t)((uint8_t)cell.row * TOP_COLS + (uint8_t)cell.col);
}

top_cell_t top_index_cell(uint8_t index)
{
    top_cell_t cell;
    cell.row = (int8_t)(index / TOP_COLS);
    cell.col = (int8_t)(index % TOP_COLS);
    return cell;
}

int top_cell_equal(top_cell_t a, top_cell_t b)
{
    return a.row == b.row && a.col == b.col;
}

uint32_t top_distance_ms(top_cell_t a, top_cell_t b, const top_config_t *config)
{
    float dr;
    float dc;
    float grid_distance;
    float milliseconds;

    if (config == NULL || config->translation_speed_mmps == 0U)
    {
        return TOP_GRID_INF_MS;
    }
    dr = (float)((int)b.row - (int)a.row);
    dc = (float)((int)b.col - (int)a.col);
    grid_distance = sqrtf(dr * dr + dc * dc);
    milliseconds = grid_distance * (float)config->cell_size_mm * 1000.0f /
                   (float)config->translation_speed_mmps;
    if (milliseconds >= (float)TOP_GRID_INF_MS)
    {
        return TOP_GRID_INF_MS;
    }
    return (uint32_t)(milliseconds + 0.5f);
}

uint32_t top_rotation_ms(top_heading_t from, top_heading_t to, const top_config_t *config)
{
    int delta;

    if (config == NULL)
    {
        return 0U;
    }
    delta = ((int)to - (int)from) & 3;
    if (delta > 2)
    {
        delta = 4 - delta;
    }
    return (uint32_t)delta * config->rotate_90_ms;
}

static int top_cell_in_list(const top_cell_t *list,
                            uint8_t count,
                            int8_t ignored,
                            top_cell_t cell)
{
    uint8_t i;

    if (list == NULL)
    {
        return 0;
    }
    for (i = 0U; i < count; ++i)
    {
        if ((int8_t)i == ignored)
        {
            continue;
        }
        if (top_cell_valid(list[i]) && top_cell_equal(list[i], cell))
        {
            return 1;
        }
    }
    return 0;
}

int top_grid_cell_blocked(const top_grid_blockers_t *blockers, top_cell_t cell)
{
    if (!top_cell_valid(cell))
    {
        return 1;
    }
    if (blockers == NULL)
    {
        return 0;
    }
    if (top_wall_test(blockers->walls, top_cell_index(cell)))
    {
        return 1;
    }
    if (top_cell_in_list(blockers->boxes,
                         blockers->box_count,
                         blockers->ignored_box,
                         cell))
    {
        return 1;
    }
    return top_cell_in_list(blockers->bombs,
                            blockers->bomb_count,
                            blockers->ignored_bomb,
                            cell);
}

static int top_heap_less(const top_cell_heap_t *heap, uint16_t a, uint16_t b)
{
    uint8_t ca = heap->cell[a];
    uint8_t cb = heap->cell[b];
    if (heap->distance[ca] != heap->distance[cb])
    {
        return heap->distance[ca] < heap->distance[cb];
    }
    return ca < cb;
}

static void top_heap_swap(top_cell_heap_t *heap, uint16_t a, uint16_t b)
{
    uint8_t ca = heap->cell[a];
    uint8_t cb = heap->cell[b];
    heap->cell[a] = cb;
    heap->cell[b] = ca;
    heap->position[ca] = (int16_t)b;
    heap->position[cb] = (int16_t)a;
}

static void top_heap_up(top_cell_heap_t *heap, uint16_t at)
{
    while (at > 0U)
    {
        uint16_t parent = (uint16_t)((at - 1U) >> 1);
        if (!top_heap_less(heap, at, parent))
        {
            break;
        }
        top_heap_swap(heap, at, parent);
        at = parent;
    }
}

static void top_heap_down(top_cell_heap_t *heap, uint16_t at)
{
    for (;;)
    {
        uint16_t left = (uint16_t)(at * 2U + 1U);
        uint16_t right = (uint16_t)(left + 1U);
        uint16_t best = at;
        if (left < heap->size && top_heap_less(heap, left, best))
        {
            best = left;
        }
        if (right < heap->size && top_heap_less(heap, right, best))
        {
            best = right;
        }
        if (best == at)
        {
            break;
        }
        top_heap_swap(heap, at, best);
        at = best;
    }
}

static void top_heap_push_or_update(top_cell_heap_t *heap, uint8_t cell)
{
    int16_t existing = heap->position[cell];
    if (existing >= 0)
    {
        top_heap_up(heap, (uint16_t)existing);
        return;
    }
    if (heap->size >= TOP_CELL_COUNT)
    {
        return;
    }
    heap->cell[heap->size] = cell;
    heap->position[cell] = (int16_t)heap->size;
    ++heap->size;
    top_heap_up(heap, (uint16_t)(heap->size - 1U));
}

static int top_heap_pop(top_cell_heap_t *heap)
{
    uint8_t result;
    if (heap->size == 0U)
    {
        return -1;
    }
    result = heap->cell[0];
    heap->position[result] = -1;
    --heap->size;
    if (heap->size > 0U)
    {
        heap->cell[0] = heap->cell[heap->size];
        heap->position[heap->cell[0]] = 0;
        top_heap_down(heap, 0U);
    }
    return result;
}

int top_grid_shortest_paths(top_cell_t start,
                            const top_grid_blockers_t *blockers,
                            const top_config_t *config,
                            top_grid_paths_t *paths)
{
    static const int8_t dr[8] = {-1, 0, 1, 0, -1, 1, 1, -1};
    static const int8_t dc[8] = {0, 1, 0, -1, 1, 1, -1, -1};
    top_cell_heap_t heap;
    uint8_t visited[TOP_CELL_COUNT];
    uint16_t i;
    uint8_t start_index;

    if (paths == NULL || config == NULL || !top_cell_valid(start))
    {
        return 0;
    }
    memset(&heap, 0, sizeof(heap));
    memset(visited, 0, sizeof(visited));
    for (i = 0U; i < TOP_CELL_COUNT; ++i)
    {
        paths->dist_ms[i] = TOP_GRID_INF_MS;
        paths->previous[i] = -1;
        heap.position[i] = -1;
    }
    heap.distance = paths->dist_ms;
    start_index = top_cell_index(start);
    paths->dist_ms[start_index] = 0U;
    top_heap_push_or_update(&heap, start_index);

    while (heap.size > 0U)
    {
        int popped = top_heap_pop(&heap);
        top_cell_t current;
        uint8_t k;
        if (popped < 0)
        {
            break;
        }
        if (visited[popped])
        {
            continue;
        }
        visited[popped] = 1U;
        current = top_index_cell((uint8_t)popped);

        for (k = 0U; k < 8U; ++k)
        {
            top_cell_t next;
            uint8_t next_index;
            uint32_t edge_ms;
            uint32_t candidate;
            int diagonal = dr[k] != 0 && dc[k] != 0;

            if (diagonal && !config->enable_diagonal)
            {
                continue;
            }
            next.row = (int8_t)(current.row + dr[k]);
            next.col = (int8_t)(current.col + dc[k]);
            if (top_grid_cell_blocked(blockers, next))
            {
                continue;
            }
            if (diagonal)
            {
                top_cell_t side_a = {(int8_t)(current.row + dr[k]), current.col};
                top_cell_t side_b = {current.row, (int8_t)(current.col + dc[k])};
                if (top_grid_cell_blocked(blockers, side_a) ||
                    top_grid_cell_blocked(blockers, side_b))
                {
                    continue;
                }
            }
            next_index = top_cell_index(next);
            if (visited[next_index])
            {
                continue;
            }
            edge_ms = top_distance_ms(current, next, config);
            candidate = paths->dist_ms[popped] + edge_ms;
            if (candidate < paths->dist_ms[next_index])
            {
                paths->dist_ms[next_index] = candidate;
                paths->previous[next_index] = (int16_t)popped;
                top_heap_push_or_update(&heap, next_index);
            }
        }
    }
    return 1;
}

int top_grid_reconstruct(top_cell_t start,
                         top_cell_t goal,
                         const top_grid_paths_t *paths,
                         top_cell_t *out,
                         uint16_t capacity,
                         uint16_t *count)
{
    uint8_t reverse[TOP_CELL_COUNT];
    uint16_t reverse_count = 0U;
    int current;
    uint8_t start_index;
    uint16_t i;

    if (count != NULL)
    {
        *count = 0U;
    }
    if (paths == NULL || out == NULL || count == NULL || capacity == 0U ||
        !top_cell_valid(start) || !top_cell_valid(goal))
    {
        return 0;
    }
    start_index = top_cell_index(start);
    current = top_cell_index(goal);
    if (paths->dist_ms[current] >= TOP_GRID_INF_MS)
    {
        return 0;
    }
    while (reverse_count < TOP_CELL_COUNT)
    {
        reverse[reverse_count++] = (uint8_t)current;
        if (current == start_index)
        {
            break;
        }
        current = paths->previous[current];
        if (current < 0)
        {
            return 0;
        }
    }
    if (reverse_count > capacity || reverse[reverse_count - 1U] != start_index)
    {
        return 0;
    }
    for (i = 0U; i < reverse_count; ++i)
    {
        out[i] = top_index_cell(reverse[reverse_count - 1U - i]);
    }
    *count = reverse_count;
    return 1;
}

static int top_segment_hits_open_box(top_cell_t start,
                                     top_cell_t end,
                                     top_cell_t blocker)
{
    float p[2] = {(float)start.row, (float)start.col};
    float d[2] = {(float)(end.row - start.row), (float)(end.col - start.col)};
    float center[2] = {(float)blocker.row, (float)blocker.col};
    float t_min = 0.0f;
    float t_max = 1.0f;
    uint8_t axis;

    for (axis = 0U; axis < 2U; ++axis)
    {
        float low = center[axis] - TOP_LINE_HALF_EXTENT;
        float high = center[axis] + TOP_LINE_HALF_EXTENT;
        if (fabsf(d[axis]) < TOP_FLOAT_EPS)
        {
            if (p[axis] < low || p[axis] > high)
            {
                return 0;
            }
        }
        else
        {
            float a = (low - p[axis]) / d[axis];
            float b = (high - p[axis]) / d[axis];
            float enter = a < b ? a : b;
            float leave = a < b ? b : a;
            if (enter > t_min)
            {
                t_min = enter;
            }
            if (leave < t_max)
            {
                t_max = leave;
            }
            if (t_min > t_max)
            {
                return 0;
            }
        }
    }
    return t_max >= 0.0f && t_min <= 1.0f;
}

int top_grid_line_clear(top_cell_t start,
                        top_cell_t end,
                        const top_grid_blockers_t *blockers)
{
    uint16_t i;

    if (!top_cell_valid(start) || !top_cell_valid(end))
    {
        return 0;
    }
    if (blockers == NULL)
    {
        return 1;
    }
    for (i = 0U; i < TOP_CELL_COUNT; ++i)
    {
        if (top_wall_test(blockers->walls, (uint8_t)i) &&
            top_segment_hits_open_box(start, end, top_index_cell((uint8_t)i)))
        {
            return 0;
        }
    }
    for (i = 0U; i < blockers->box_count; ++i)
    {
        if ((int8_t)i != blockers->ignored_box && top_cell_valid(blockers->boxes[i]) &&
            top_segment_hits_open_box(start, end, blockers->boxes[i]))
        {
            return 0;
        }
    }
    for (i = 0U; i < blockers->bomb_count; ++i)
    {
        if ((int8_t)i != blockers->ignored_bomb && top_cell_valid(blockers->bombs[i]) &&
            top_segment_hits_open_box(start, end, blockers->bombs[i]))
        {
            return 0;
        }
    }
    return 1;
}

int top_grid_optimize_walk(const top_cell_t *raw,
                           uint16_t raw_count,
                           const top_grid_blockers_t *blockers,
                           const top_config_t *config,
                           top_cell_t *out,
                           uint16_t capacity,
                           uint16_t *out_count,
                           uint32_t *duration_ms)
{
    uint32_t cost[TOP_CELL_COUNT];
    uint16_t points[TOP_CELL_COUNT];
    int16_t previous[TOP_CELL_COUNT];
    uint8_t reverse[TOP_CELL_COUNT];
    uint16_t reverse_count = 0U;
    uint16_t i;
    uint16_t j;
    int current;

    if (out_count != NULL)
    {
        *out_count = 0U;
    }
    if (duration_ms != NULL)
    {
        *duration_ms = 0U;
    }
    if (raw == NULL || out == NULL || out_count == NULL || duration_ms == NULL ||
        config == NULL || raw_count == 0U || raw_count > TOP_CELL_COUNT)
    {
        return 0;
    }
    for (i = 0U; i < raw_count; ++i)
    {
        cost[i] = TOP_GRID_INF_MS;
        points[i] = UINT16_MAX;
        previous[i] = -1;
    }
    cost[0] = 0U;
    points[0] = 1U;
    for (j = 1U; j < raw_count; ++j)
    {
        for (i = 0U; i < j; ++i)
        {
            uint32_t edge;
            uint32_t candidate;
            uint16_t candidate_points;
            if (cost[i] >= TOP_GRID_INF_MS ||
                !top_grid_line_clear(raw[i], raw[j], blockers))
            {
                continue;
            }
            edge = top_distance_ms(raw[i], raw[j], config);
            candidate = cost[i] + edge;
            candidate_points = (uint16_t)(points[i] + 1U);
            if (candidate < cost[j] ||
                (candidate == cost[j] && candidate_points < points[j]))
            {
                cost[j] = candidate;
                points[j] = candidate_points;
                previous[j] = (int16_t)i;
            }
        }
    }
    if (cost[raw_count - 1U] >= TOP_GRID_INF_MS)
    {
        return 0;
    }
    current = (int)raw_count - 1;
    while (current >= 0 && reverse_count < TOP_CELL_COUNT)
    {
        reverse[reverse_count++] = (uint8_t)current;
        if (current == 0)
        {
            break;
        }
        current = previous[current];
    }
    if (reverse_count > capacity || reverse[reverse_count - 1U] != 0U)
    {
        return 0;
    }
    for (i = 0U; i < reverse_count; ++i)
    {
        out[i] = raw[reverse[reverse_count - 1U - i]];
    }
    *out_count = reverse_count;
    *duration_ms = cost[raw_count - 1U];
    return 1;
}
