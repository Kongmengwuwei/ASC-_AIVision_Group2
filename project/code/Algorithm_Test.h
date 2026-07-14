#ifndef _ALGORITHM_TEST_H
#define _ALGORITHM_TEST_H

#include "Map_Path_Data.h"

#ifndef ALGORITHM_TEST_PRESET_INDEX 
#define ALGORITHM_TEST_PRESET_INDEX 0U
#endif

void Algorithm_Test_PresetInput_Init(size_t preset_index);
void Algorithm_Test_PresetInput_SetEnabled(uint8 enabled, size_t preset_index);
uint8 Algorithm_Test_PresetInput_IsEnabled(void);
const MapPresetConfig *Algorithm_Test_PresetInput_GetActive(void);
map_preset_plan_mode_t Algorithm_Test_PresetInput_GetPlanMode(void);
uint8 Algorithm_Test_PresetInput_ProvideMapFrame(void);
uint8 Algorithm_Test_PresetInput_ProvideCarPoseFrame(void);
uint8 Algorithm_Test_PresetInput_GetObjectId(Position object_pos, uint8 is_target, uint8 *id_out);

#endif
