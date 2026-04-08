#ifndef _GAME_LOGIC_H
#define _GAME_LOGIC_H

#include "Map_Route_Data.h"
#include "Algorithm.h"

// 模式1：不区分 id，按全局最优策略规划所有箱子。
void Plan_path_Mode1(void);

// 模式2：每个箱子必须对应推到相同 id 的目标点。
void Plan_path_Mode2(void);

#endif
