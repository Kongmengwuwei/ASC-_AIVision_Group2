#include "Preset_Route_Data.h"

/*
 * Route tables use map coordinates:
 *   row: 0..MAP_ROWS-1, top to bottom
 *   col: 0..MAP_COLS-1, left to right
 *
 * All routes start and end at landing_grid (5,1). They are the fixed
 * school-match paths for the three known maps.
 */

#define ROUTE_POINT(row, col) {row, col, PRESET_ACTION_NONE, PRESET_FACE_KEEP, 0U}
#define ROUTE_FAKE(row, col, face) {row, col, PRESET_ACTION_FAKE_IDENTIFY, face, 0U}
#define ROUTE_STOP(row, col) {row, col, PRESET_ACTION_STOP_ONLY, PRESET_FACE_KEEP, 0U}
#define ROUTE_END(row, col) {row, col, PRESET_ACTION_ROUTE_END, PRESET_FACE_KEEP, 0U}


//static const preset_point_t s_route1_points[] = {
//    ROUTE_POINT(5U, 1U),
//    ROUTE_POINT(5U, 2U),
//    ROUTE_POINT(5U, 3U),
//	ROUTE_POINT(5U, 4U),
//	ROUTE_POINT(5U, 5U),
//	ROUTE_POINT(5U, 6U),
//	ROUTE_POINT(5U, 7U),
//	ROUTE_POINT(5U, 8U),
//	ROUTE_POINT(5U, 7U),
//	ROUTE_POINT(5U, 6U),
//	ROUTE_POINT(5U, 5U),
//	ROUTE_POINT(5U, 4U),
//	ROUTE_POINT(5U, 3U),
//	ROUTE_POINT(5U, 2U),
//    ROUTE_END(5U, 1U),
//};
static const preset_point_t s_route1_points[] = {
    ROUTE_POINT(5U, 1U),
    ROUTE_POINT(4U, 1U),
    ROUTE_POINT(3U, 1U),
    ROUTE_POINT(3U, 2U),
    ROUTE_POINT(3U, 3U),
    ROUTE_POINT(3U, 4U),
    ROUTE_FAKE(3U, 5U, PRESET_FACE_MAP_RIGHT),
    ROUTE_POINT(3U, 6U),
    ROUTE_POINT(3U, 7U),
    ROUTE_POINT(4U, 7U),
    ROUTE_POINT(5U, 7U),
    ROUTE_POINT(6U, 7U),
    ROUTE_POINT(6U, 6U),
    ROUTE_POINT(5U, 6U),
    ROUTE_POINT(4U, 6U),
    ROUTE_POINT(4U, 5U),
    ROUTE_POINT(3U, 5U),
    ROUTE_POINT(3U, 6U),
    ROUTE_POINT(3U, 7U),
    ROUTE_POINT(3U, 8U),
    ROUTE_FAKE(4U, 8U, PRESET_FACE_MAP_UP),
    ROUTE_POINT(5U, 8U),
    ROUTE_POINT(5U, 7U),
    ROUTE_POINT(5U, 6U),
    ROUTE_POINT(5U, 5U),
    ROUTE_POINT(5U, 4U),
    ROUTE_POINT(5U, 3U),
    ROUTE_POINT(5U, 2U),
    ROUTE_END(5U, 1U),
};

static const preset_point_t s_route2_points[] = {
    ROUTE_POINT(5U, 1U),
    ROUTE_POINT(4U, 1U),
    ROUTE_FAKE(3U, 1U, PRESET_FACE_MAP_UP),
    ROUTE_FAKE(2U, 1U, PRESET_FACE_MAP_UP),
    ROUTE_POINT(3U, 1U),
    ROUTE_POINT(4U, 1U),
    ROUTE_POINT(5U, 1U),
    ROUTE_POINT(5U, 2U),
    ROUTE_POINT(5U, 3U),
    ROUTE_POINT(5U, 4U),
    ROUTE_POINT(5U, 5U),
    ROUTE_FAKE(5U, 6U, PRESET_FACE_MAP_RIGHT),
    ROUTE_POINT(5U, 7U),
    ROUTE_POINT(5U, 8U),
    ROUTE_POINT(6U, 8U),
    ROUTE_POINT(6U, 9U),
    ROUTE_POINT(5U, 9U),
    ROUTE_FAKE(4U, 9U, PRESET_FACE_MAP_UP),
    ROUTE_POINT(3U, 9U),
    ROUTE_POINT(2U, 9U),
    ROUTE_POINT(2U, 10U),
    ROUTE_POINT(2U, 11U),
    ROUTE_POINT(3U, 11U),
    ROUTE_POINT(4U, 11U),
    ROUTE_POINT(5U, 11U),
    ROUTE_POINT(5U, 10U),
    ROUTE_POINT(6U, 10U),
    ROUTE_POINT(6U, 9U),
    ROUTE_POINT(5U, 9U),
    ROUTE_POINT(5U, 8U),
    ROUTE_POINT(5U, 7U),
    ROUTE_POINT(5U, 6U),
    ROUTE_POINT(5U, 5U),
    ROUTE_POINT(5U, 4U),
    ROUTE_POINT(5U, 3U),
    ROUTE_POINT(5U, 2U),
    ROUTE_END(5U, 1U),
};

static const preset_point_t s_route3_points[] = {
    ROUTE_POINT(5U, 1U),
    ROUTE_POINT(5U, 0U),
    ROUTE_POINT(6U, 0U),
    ROUTE_POINT(7U, 0U),
    ROUTE_POINT(7U, 1U),
    ROUTE_POINT(6U, 1U),
    ROUTE_STOP(5U, 1U),
    ROUTE_POINT(5U, 2U),
    ROUTE_POINT(5U, 3U),
    ROUTE_POINT(5U, 4U),
    ROUTE_POINT(5U, 5U),
    ROUTE_POINT(6U, 5U),
    ROUTE_POINT(6U, 6U),
    ROUTE_POINT(6U, 7U),
    ROUTE_POINT(6U, 8U),
    ROUTE_POINT(6U, 9U),
    ROUTE_STOP(6U, 10U),
    ROUTE_POINT(6U, 11U),
    ROUTE_STOP(7U, 11U),
    ROUTE_POINT(6U, 11U),
    ROUTE_POINT(6U, 10U),
    ROUTE_POINT(6U, 9U),
    ROUTE_POINT(6U, 8U),
    ROUTE_POINT(6U, 7U),
    ROUTE_POINT(6U, 6U),
    ROUTE_POINT(6U, 5U),
    ROUTE_POINT(6U, 4U),
    ROUTE_POINT(6U, 3U),
    ROUTE_POINT(7U, 3U),
    ROUTE_POINT(8U, 3U),
    ROUTE_FAKE(8U, 2U, PRESET_FACE_MAP_UP),
    ROUTE_POINT(7U, 2U),
    ROUTE_POINT(6U, 2U),
    ROUTE_POINT(6U, 1U),
    ROUTE_POINT(5U, 1U),
    ROUTE_POINT(5U, 2U),
    ROUTE_POINT(5U, 3U),
    ROUTE_POINT(5U, 4U),
    ROUTE_POINT(5U, 5U),
    ROUTE_POINT(6U, 5U),
    ROUTE_POINT(6U, 6U),
    ROUTE_POINT(5U, 6U),
    ROUTE_POINT(5U, 5U),
    ROUTE_POINT(4U, 5U),
    ROUTE_POINT(4U, 6U),
    ROUTE_POINT(4U, 7U),
    ROUTE_POINT(4U, 8U),
    ROUTE_POINT(4U, 9U),
    ROUTE_POINT(5U, 9U),
    ROUTE_POINT(5U, 10U),
    ROUTE_POINT(4U, 10U),
    ROUTE_POINT(3U, 10U),
    ROUTE_POINT(3U, 9U),
    ROUTE_POINT(2U, 9U),
    ROUTE_POINT(2U, 10U),
    ROUTE_POINT(2U, 11U),
    ROUTE_POINT(2U, 12U),
    ROUTE_POINT(3U, 12U),
    ROUTE_POINT(3U, 13U),
    ROUTE_POINT(2U, 13U),
    ROUTE_FAKE(1U, 13U, PRESET_FACE_MAP_UP),
    ROUTE_POINT(2U, 13U),
    ROUTE_POINT(3U, 13U),
    ROUTE_POINT(4U, 13U),
    ROUTE_POINT(5U, 13U),
    ROUTE_POINT(6U, 13U),
    ROUTE_POINT(6U, 12U),
    ROUTE_POINT(6U, 11U),
    ROUTE_POINT(6U, 10U),
    ROUTE_POINT(6U, 9U),
    ROUTE_POINT(6U, 8U),
    ROUTE_POINT(6U, 7U),
    ROUTE_POINT(6U, 6U),
    ROUTE_POINT(6U, 5U),
    ROUTE_POINT(6U, 4U),
    ROUTE_POINT(6U, 3U),
    ROUTE_POINT(7U, 3U),
    ROUTE_POINT(8U, 3U),
    ROUTE_POINT(9U, 3U),
    ROUTE_POINT(9U, 2U),
    ROUTE_FAKE(9U, 1U, PRESET_FACE_MAP_UP),
    ROUTE_POINT(8U, 1U),
    ROUTE_POINT(7U, 1U),
    ROUTE_POINT(7U, 0U),
    ROUTE_POINT(6U, 0U),
    ROUTE_POINT(6U, 1U),
    ROUTE_POINT(6U, 2U),
    ROUTE_POINT(6U, 3U),
    ROUTE_POINT(6U, 4U),
    ROUTE_POINT(6U, 5U),
    ROUTE_POINT(6U, 6U),
    ROUTE_POINT(6U, 7U),
    ROUTE_POINT(6U, 8U),
    ROUTE_POINT(6U, 9U),
    ROUTE_POINT(6U, 10U),
    ROUTE_POINT(5U, 10U),
    ROUTE_POINT(5U, 11U),
    ROUTE_POINT(6U, 11U),
    ROUTE_POINT(7U, 11U),
    ROUTE_POINT(8U, 11U),
    ROUTE_POINT(8U, 12U),
    ROUTE_POINT(9U, 12U),
    ROUTE_POINT(9U, 11U),
    ROUTE_POINT(9U, 10U),
    ROUTE_POINT(9U, 9U),
    ROUTE_POINT(9U, 8U),
    ROUTE_POINT(9U, 7U),
    ROUTE_FAKE(9U, 6U, PRESET_FACE_MAP_LEFT),
    ROUTE_POINT(9U, 7U),
    ROUTE_POINT(9U, 8U),
    ROUTE_POINT(9U, 9U),
    ROUTE_POINT(9U, 10U),
    ROUTE_POINT(8U, 10U),
    ROUTE_POINT(7U, 10U),
    ROUTE_POINT(6U, 10U),
    ROUTE_POINT(6U, 11U),
    ROUTE_POINT(5U, 11U),
    ROUTE_POINT(5U, 10U),
    ROUTE_POINT(5U, 9U),
    ROUTE_POINT(5U, 8U),
    ROUTE_POINT(5U, 7U),
    ROUTE_POINT(5U, 6U),
    ROUTE_POINT(5U, 5U),
    ROUTE_POINT(5U, 4U),
    ROUTE_POINT(5U, 3U),
    ROUTE_POINT(6U, 3U),
    ROUTE_POINT(6U, 2U),
    ROUTE_POINT(5U, 2U),
    ROUTE_POINT(4U, 2U),
    ROUTE_POINT(4U, 3U),
    ROUTE_POINT(3U, 3U),
    ROUTE_POINT(3U, 2U),
    ROUTE_POINT(4U, 2U),
    ROUTE_POINT(5U, 2U),
    ROUTE_END(5U, 1U),
};

const preset_route_t g_preset_routes[PRESET_ROUTE_COUNT] = {
    {
        s_route1_points,
        sizeof(s_route1_points) / sizeof(s_route1_points[0]),
        {5U, 1U, 0U},
        PRESET_DEFAULT_PRE_IDENTIFY_PAUSE_MS,
        PRESET_DEFAULT_MAP_END_WAIT_MS,
        1U,
    },
    {
        s_route2_points,
        sizeof(s_route2_points) / sizeof(s_route2_points[0]),
        {5U, 1U, 0U},
        PRESET_DEFAULT_PRE_IDENTIFY_PAUSE_MS,
        PRESET_DEFAULT_MAP_END_WAIT_MS,
        1U,
    },
    {
        s_route3_points,
        sizeof(s_route3_points) / sizeof(s_route3_points[0]),
        {5U, 1U, 0U},
        PRESET_DEFAULT_PRE_IDENTIFY_PAUSE_MS,
        PRESET_DEFAULT_MAP_END_WAIT_MS,
        1U,
    },
};

#undef ROUTE_POINT
#undef ROUTE_FAKE
#undef ROUTE_STOP
#undef ROUTE_END


/*
MAP1:
################
#..............#
#..............#
#..............#
#......B（1）.T（1）T....#
#..............#
#......B.......#
#..............#
#..............#
#..............#
#..............#
################

MAP2:
################
#.#............#
#.T（1）......#####.#
##B（1）###...#...#.#
#....#...#T（2）#.#.#
#....#####T#.#.#
#.......B（2）..B.#.#
#...........##.#
#..............#
#.....####.....#
#..............#
################

MAP3:
################
#.#...#..#....T#
#...#..#.####..#
##.###.#.#.....#
#.T#.#.#.#..#..#
#.##.#.........#
#..#.......##..#
#.D.......D.#..#
#..B.#.....BD..#
#.B..########..#
#....#T........#
################

加入对应关系：
################
#..............#
#..............#
#..............#
#......B.TT....#
#..............#
#......B.......#
#..............#
#..............#
#..............#
#..............#
################

MAP2:
################
#.#............#
#.T......#####.#
##B###...#...#.#
#....#...#T#.#.#
#....#####T#.#.#
#.......B..B.#.#
#...........##.#
#..............#
#.....####.....#
#..............#
################

MAP3:
################
#.#...#..#....T（2）#
#...#..#.####..#
##.###.#.#.....#
#.T（1）#.#.#.#..#..#
#.##.#.........#
#..#.......##..#
#.D.......D.#..#
#..B（2）.#.....B（1）D..#
#.B..########..#
#....#T........#
################
*/
