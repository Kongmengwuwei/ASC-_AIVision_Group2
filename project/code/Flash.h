#ifndef _FLASH_H
#define _FLASH_H

#include "zf_driver_flash.h"

#define FLASH_SECTION_INDEX 127U
#define FLASH_PAGE_INDEX FLASH_PAGE_3

typedef struct
{
    uint8 start_dir;
    uint8 continuous_levels;
    uint8 diagonal_path;
    uint8 followup_vision;
    uint8 identify_prerotate;
    uint8 preset_input;
    uint8 preset_map_index;
    uint8 show_map;
    uint8 show_data;
    uint8 blue_serial;
} menu_flash_config_t;

uint8 Data_save_to_flash(const menu_flash_config_t *config);
uint8 Data_load_from_flash(menu_flash_config_t *config);
uint8 Data_clear_flash(void);

#endif
