#include"Flash.h"

uint8 Data_save_to_flash(void)
{
    flash_buffer_clear();

    if (flash_check(FLASH_SECTION_INDEX, FLASH_PAGE_INDEX))
    {
        flash_erase_page(FLASH_SECTION_INDEX, FLASH_PAGE_INDEX);
    }

    // flash_union_buffer[0].int32_type = straight_speed;
    // flash_union_buffer[1].int32_type = turn_speed;
    // flash_union_buffer[2].float_type = speed_k;
    // flash_union_buffer[3].int32_type = speed_limit;
    // flash_union_buffer[4].int32_type = yuanhuan_speed;
    // flash_union_buffer[5].float_type = pid_world_x.fKp;
    // flash_union_buffer[6].float_type = pid_world_x.fKi;
    // flash_union_buffer[7].float_type = pid_world_x.fKd;

    // flash_union_buffer[8].int32_type = start_accelerate;
    // flash_union_buffer[9].int32_type = end_decelerate;
    // flash_union_buffer[10].int32_type = slow_turn_speed;
    // flash_union_buffer[11].int32_type = turn_num;

    // flash_union_buffer[12].float_type = pid_world_y.fKp;
    // flash_union_buffer[13].float_type = pid_world_y.fKi;
    // flash_union_buffer[14].float_type = pid_world_y.fKd;
    // flash_union_buffer[15].float_type = pid_yaw.fKp;
    // flash_union_buffer[16].float_type = pid_yaw.fKi;
    // flash_union_buffer[17].float_type = pid_yaw.fKd;
    // flash_union_buffer[18].float_type = pid_accel_yaw.fKp;
    // flash_union_buffer[19].float_type = pid_accel_yaw.fKi;
    // flash_union_buffer[20].float_type = pid_accel_yaw.fKd;

    if (flash_write_page_from_buffer(FLASH_SECTION_INDEX, FLASH_PAGE_INDEX) != 0)
    {
        return 0;
    }
    return 1;
}

uint8 Data_load_from_flash(void)
{
    flash_buffer_clear();
    flash_read_page_to_buffer(FLASH_SECTION_INDEX, FLASH_PAGE_INDEX);

    if (flash_check(FLASH_SECTION_INDEX, FLASH_PAGE_INDEX) == 0)
    {
        return 0;
    }

    // straight_speed = flash_union_buffer[0].int32_type;
    // turn_speed = flash_union_buffer[1].int32_type;
    // speed_k = flash_union_buffer[2].float_type;
    // speed_limit = flash_union_buffer[3].int32_type;
    // yuanhuan_speed = flash_union_buffer[4].int32_type;
    // pid_world_x.fKp = flash_union_buffer[5].float_type;
    // pid_world_x.fKi = flash_union_buffer[6].float_type;
    // pid_world_x.fKd = flash_union_buffer[7].float_type;

    // start_accelerate = flash_union_buffer[8].int32_type;
    // end_decelerate = flash_union_buffer[9].int32_type;
    // slow_turn_speed = flash_union_buffer[10].int32_type;
    // turn_num = flash_union_buffer[11].int32_type;

    // pid_world_y.fKp = flash_union_buffer[12].float_type;
    // pid_world_y.fKi = flash_union_buffer[13].float_type;
    // pid_world_y.fKd = flash_union_buffer[14].float_type;
    // pid_yaw.fKp = flash_union_buffer[15].float_type;
    // pid_yaw.fKi = flash_union_buffer[16].float_type;
    // pid_yaw.fKd = flash_union_buffer[17].float_type;
    // pid_accel_yaw.fKp = flash_union_buffer[18].float_type;
    // pid_accel_yaw.fKi = flash_union_buffer[19].float_type;
    // pid_accel_yaw.fKd = flash_union_buffer[20].float_type;

    return 1;
}

uint8 Data_clear_flash(void)
{
    flash_buffer_clear();
    if (flash_erase_page(FLASH_SECTION_INDEX, FLASH_PAGE_INDEX) != 0)
    {
        return 0;
    }
    return 1;
}
