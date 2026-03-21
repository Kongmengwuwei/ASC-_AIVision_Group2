#ifndef _FLASH_H
#define _FLASH_H

#include "zf_driver_flash.h"

#define FLASH_SECTION_INDEX 127
#define FLASH_PAGE_INDEX FLASH_PAGE_3
#define PATH_ITEMS_PER_PAGE 32

uint8 Data_save_to_flash(void);
uint8 Data_load_from_flash(void);
uint8 Data_clear_flash(void);

#endif
