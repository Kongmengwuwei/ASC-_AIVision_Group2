#ifndef _GAME_LOGIC_H
#define _GAME_LOGIC_H

#include "Map_Route_Data.h"
#include "Algorithm.h"

// Mode1: plan all boxes without id constraint.
void Plan_path_Mode1(void);

// Mode2: each box must be pushed to target with the same id.
void Plan_path_Mode2(void);

#endif
