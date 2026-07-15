#ifndef TOP_GRID_H
#define TOP_GRID_H

#include "TopPlanner.h"

#define TOP_GRID_INF_MS UINT32_C(0x3fffffff)

typedef struct
{
    uint32_t dist_ms[TOP_CELL_COUNT];
    int16_t previous[TOP_CELL_COUNT];
} top_grid_paths_t;

typedef struct
{
    const top_wall_bits_t *walls;
    const top_cell_t *boxes;
    uint8_t box_count;
    const top_cell_t *bombs;
    uint8_t bomb_count;
    int8_t ignored_box;
    int8_t ignored_bomb;
} top_grid_blockers_t;

int top_cell_valid(top_cell_t cell);
uint8_t top_cell_index(top_cell_t cell);
top_cell_t top_index_cell(uint8_t index);
int top_cell_equal(top_cell_t a, top_cell_t b);
uint32_t top_distance_ms(top_cell_t a, top_cell_t b, const top_config_t *config);
uint32_t top_rotation_ms(top_heading_t from, top_heading_t to, const top_config_t *config);

int top_grid_cell_blocked(const top_grid_blockers_t *blockers, top_cell_t cell);
int top_grid_shortest_paths(top_cell_t start,
                            const top_grid_blockers_t *blockers,
                            const top_config_t *config,
                            top_grid_paths_t *paths);
int top_grid_reconstruct(top_cell_t start,
                         top_cell_t goal,
                         const top_grid_paths_t *paths,
                         top_cell_t *out,
                         uint16_t capacity,
                         uint16_t *count);
int top_grid_line_clear(top_cell_t start,
                        top_cell_t end,
                        const top_grid_blockers_t *blockers);
int top_grid_optimize_walk(const top_cell_t *raw,
                           uint16_t raw_count,
                           const top_grid_blockers_t *blockers,
                           const top_config_t *config,
                           top_cell_t *out,
                           uint16_t capacity,
                           uint16_t *out_count,
                           uint32_t *duration_ms);

#endif
