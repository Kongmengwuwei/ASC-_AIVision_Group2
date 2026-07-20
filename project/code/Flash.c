#include "Flash.h"
#include "Control.h"

#define MENU_FLASH_MAGIC 0x4D454E55U
#define MENU_FLASH_VERSION 1U
#define MENU_FLASH_CHECK_XOR 0xA5A55A5AU

#define MENU_FLASH_WORD_MAGIC 0U
#define MENU_FLASH_WORD_VERSION 1U
#define MENU_FLASH_WORD_FLAGS 2U
#define MENU_FLASH_WORD_START_DIR 3U
#define MENU_FLASH_WORD_MAP_INDEX 4U
#define MENU_FLASH_WORD_CHECKSUM 5U

#define MENU_FLAG_CONTINUOUS_LEVELS (1UL << 0)
/* Bits 1 and 3 are reserved for compatibility with old saved settings.
 * Diagonal paths and identify pre-rotation are now permanently enabled. */
/* Bit 2 was reserved; old saved configurations therefore keep this risky fallback off. */
#define MENU_FLAG_IDENTIFY_ID_FALLBACK (1UL << 2)
#define MENU_FLAG_PRESET_INPUT (1UL << 4)
#define MENU_FLAG_SHOW_MAP (1UL << 5)
#define MENU_FLAG_SHOW_DATA (1UL << 6)
#define MENU_FLAG_BLUE_SERIAL (1UL << 7)
#define MENU_FLAG_CHECKPOINT_VISION (1UL << 8)
#define MENU_FLAG_LAST_PAIR_INSURANCE (1UL << 9)
#define MENU_FLAG_CHECKPOINT_VISION_EVERY_POINT (1UL << 10)
#define MENU_FLAG_CHECKPOINT_VISION_REDUCED (1UL << 11)
#define MENU_FLAG_CHECKPOINT_VISION_MINIMAL (1UL << 12)

static uint32 menu_flash_checksum(uint32 flags, uint32 start_dir, uint32 map_index)
{
    return MENU_FLASH_MAGIC ^ MENU_FLASH_VERSION ^ flags ^ start_dir ^ map_index ^ MENU_FLASH_CHECK_XOR;
}

static uint32 menu_flash_build_flags(const menu_flash_config_t *config)
{
    uint32 flags = 0U;

    if (config->continuous_levels) flags |= MENU_FLAG_CONTINUOUS_LEVELS;
    if (config->preset_input) flags |= MENU_FLAG_PRESET_INPUT;
    if (config->show_map) flags |= MENU_FLAG_SHOW_MAP;
    if (config->show_data) flags |= MENU_FLAG_SHOW_DATA;
    if (config->blue_serial) flags |= MENU_FLAG_BLUE_SERIAL;
    if (config->checkpoint_vision_mode == CONTROL_CHECKPOINT_VISION_MODE_MINIMAL)
    {
        flags |= MENU_FLAG_CHECKPOINT_VISION_MINIMAL;
    }
    else if (config->checkpoint_vision_mode == CONTROL_CHECKPOINT_VISION_MODE_REDUCED)
    {
        flags |= MENU_FLAG_CHECKPOINT_VISION_REDUCED;
    }
    else if (config->checkpoint_vision_mode == CONTROL_CHECKPOINT_VISION_MODE_STANDARD)
    {
        flags |= MENU_FLAG_CHECKPOINT_VISION;
    }
    else if (config->checkpoint_vision_mode >= CONTROL_CHECKPOINT_VISION_MODE_EVERY_POINT)
    {
        /* Keep the legacy enabled bit so an older firmware still sees ON. */
        flags |= MENU_FLAG_CHECKPOINT_VISION |
                 MENU_FLAG_CHECKPOINT_VISION_EVERY_POINT;
    }
    if (config->identify_id_fallback) flags |= MENU_FLAG_IDENTIFY_ID_FALLBACK;
    if (config->last_pair_insurance) flags |= MENU_FLAG_LAST_PAIR_INSURANCE;
    return flags;
}

static uint8 menu_flash_buffer_valid(void)
{
    uint32 flags = flash_union_buffer[MENU_FLASH_WORD_FLAGS].uint32_type;
    uint32 start_dir = flash_union_buffer[MENU_FLASH_WORD_START_DIR].uint32_type;
    uint32 map_index = flash_union_buffer[MENU_FLASH_WORD_MAP_INDEX].uint32_type;

    return (flash_union_buffer[MENU_FLASH_WORD_MAGIC].uint32_type == MENU_FLASH_MAGIC &&
            flash_union_buffer[MENU_FLASH_WORD_VERSION].uint32_type == MENU_FLASH_VERSION &&
            flash_union_buffer[MENU_FLASH_WORD_CHECKSUM].uint32_type ==
                menu_flash_checksum(flags, start_dir, map_index)) ? 1U : 0U;
}

uint8 Data_save_to_flash(const menu_flash_config_t *config)
{
    uint32 flags;

    if (config == NULL)
    {
        return 0U;
    }

    flags = menu_flash_build_flags(config);
    flash_buffer_clear();
    flash_union_buffer[MENU_FLASH_WORD_MAGIC].uint32_type = MENU_FLASH_MAGIC;
    flash_union_buffer[MENU_FLASH_WORD_VERSION].uint32_type = MENU_FLASH_VERSION;
    flash_union_buffer[MENU_FLASH_WORD_FLAGS].uint32_type = flags;
    flash_union_buffer[MENU_FLASH_WORD_START_DIR].uint32_type = config->start_dir;
    flash_union_buffer[MENU_FLASH_WORD_MAP_INDEX].uint32_type = config->preset_map_index;
    flash_union_buffer[MENU_FLASH_WORD_CHECKSUM].uint32_type =
        menu_flash_checksum(flags, config->start_dir, config->preset_map_index);

    if (flash_check(FLASH_SECTION_INDEX, FLASH_PAGE_INDEX) &&
        flash_erase_page(FLASH_SECTION_INDEX, FLASH_PAGE_INDEX) != 0U)
    {
        return 0U;
    }
    if (flash_write_page_from_buffer(FLASH_SECTION_INDEX, FLASH_PAGE_INDEX) != 0U)
    {
        return 0U;
    }

    flash_read_page_to_buffer(FLASH_SECTION_INDEX, FLASH_PAGE_INDEX);
    return menu_flash_buffer_valid();
}

uint8 Data_load_from_flash(menu_flash_config_t *config)
{
    uint32 flags;

    if (config == NULL || !flash_check(FLASH_SECTION_INDEX, FLASH_PAGE_INDEX))
    {
        return 0U;
    }

    flash_read_page_to_buffer(FLASH_SECTION_INDEX, FLASH_PAGE_INDEX);
    if (!menu_flash_buffer_valid())
    {
        return 0U;
    }

    flags = flash_union_buffer[MENU_FLASH_WORD_FLAGS].uint32_type;
    config->start_dir = (uint8)flash_union_buffer[MENU_FLASH_WORD_START_DIR].uint32_type;
    config->continuous_levels = (flags & MENU_FLAG_CONTINUOUS_LEVELS) ? 1U : 0U;
    config->preset_input = (flags & MENU_FLAG_PRESET_INPUT) ? 1U : 0U;
    config->preset_map_index = (uint8)flash_union_buffer[MENU_FLASH_WORD_MAP_INDEX].uint32_type;
    config->show_map = (flags & MENU_FLAG_SHOW_MAP) ? 1U : 0U;
    config->show_data = (flags & MENU_FLAG_SHOW_DATA) ? 1U : 0U;
    config->blue_serial = (flags & MENU_FLAG_BLUE_SERIAL) ? 1U : 0U;
    if (flags & MENU_FLAG_CHECKPOINT_VISION_EVERY_POINT)
    {
        config->checkpoint_vision_mode = CONTROL_CHECKPOINT_VISION_MODE_EVERY_POINT;
    }
    else if (flags & MENU_FLAG_CHECKPOINT_VISION_MINIMAL)
    {
        config->checkpoint_vision_mode = CONTROL_CHECKPOINT_VISION_MODE_MINIMAL;
    }
    else if (flags & MENU_FLAG_CHECKPOINT_VISION_REDUCED)
    {
        config->checkpoint_vision_mode = CONTROL_CHECKPOINT_VISION_MODE_REDUCED;
    }
    else if (flags & MENU_FLAG_CHECKPOINT_VISION)
    {
        /* Backward compatible with the former boolean setting. */
        config->checkpoint_vision_mode = CONTROL_CHECKPOINT_VISION_MODE_STANDARD;
    }
    else
    {
        config->checkpoint_vision_mode = CONTROL_CHECKPOINT_VISION_MODE_OFF;
    }
    config->identify_id_fallback = (flags & MENU_FLAG_IDENTIFY_ID_FALLBACK) ? 1U : 0U;
    config->last_pair_insurance = (flags & MENU_FLAG_LAST_PAIR_INSURANCE) ? 1U : 0U;
    return 1U;
}

uint8 Data_clear_flash(void)
{
    flash_buffer_clear();
    return (flash_erase_page(FLASH_SECTION_INDEX, FLASH_PAGE_INDEX) == 0U) ? 1U : 0U;
}
