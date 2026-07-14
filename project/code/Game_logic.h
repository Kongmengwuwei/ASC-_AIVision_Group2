#ifndef _GAME_LOGIC_H
#define _GAME_LOGIC_H

#include "Map_Path_Data.h"
#include "Algorithm.h"

/* 模式1：不按 ID 配对，按全局最短策略规划所有箱子。 */
void Plan_path_Mode1(void);

/* 模式2：按 ID 一一配对（箱子必须推到同 ID 目标点）。 */
void Plan_path_Mode2(void);

/* 模式3：识别模式 */
void Plan_path_Identify(void);

/* 识别访问顺序；真实相机识别前 g_identify_seq_id 中的值可能为 0。 */
extern uint8 g_identify_seq_kind[MAX_BOXES + MAX_TARGETS];
extern uint8 g_identify_seq_id[MAX_BOXES + MAX_TARGETS];
extern int g_identify_seq_len;

/* 本次识别规划预计炸毁的初始障碍格。 */
extern Position g_blown_cell[MAX_OBSTACLES];
extern int g_blown_count;

#endif
