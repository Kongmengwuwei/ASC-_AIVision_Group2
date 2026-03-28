#ifndef _ALGORITHM_H
#define _ALGORITHM_H

#include "Camera_handler.h"

/*方格识别码*/
#define OBSTACLE     0x01
#define BOMB         0x02
#define BOX          0x04

static void grid_build(int rows, int cols, 
                       const Point *obstacles, int obstacles_cnt,
                       const Point *boxes, int boxes_cnt,
                       const Point *bombs, int bombs_cnt,
                       uint8_t *grid);

#endif 