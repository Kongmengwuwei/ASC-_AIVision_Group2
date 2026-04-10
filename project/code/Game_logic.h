#ifndef _GAME_LOGIC_H
#define _GAME_LOGIC_H

#include "Map_Route_Data.h"
#include "Algorithm.h"

/* 模式1：不按 ID 配对，按全局最短策略规划所有箱子。 */
void Plan_path_Mode1(void);

/* 模式2：按 ID 一一配对（箱子必须推到同 ID 目标点）。 */
void Plan_path_Mode2(void);

#endif
