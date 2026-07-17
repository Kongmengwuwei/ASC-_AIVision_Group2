#include "Mymenu.h"
#include "path.h"
#include "BlueSerial.h"
#include "Flash.h"

#define MENU_SHOW_PERIOD_LOOPS 3U

Menu_Item Root;     // 根目�?
Menu_Item *pointer; // 指针

uint8 car_go_flag = 0;   // 运行标志
uint8 car_stop_flag = 0; // 停车标志

static bool startup_start_switch = false;
static bool startup_reset_switch = false;
static uint8 startup_depart_dir_value = 0U;
static bool diagonal_path_switch = true;
static bool identify_prerotate_switch = true;
static bool continuous_levels_switch = false;
static bool blue_serial_switch = true;
static bool checkpoint_vision_switch = false;
static bool preset_input_switch = false;
static bool show_map_switch = true;
static bool show_data_switch = true;
static uint8 preset_map_index = ALGORITHM_TEST_PRESET_INDEX;

path_follow_status_t path_follow_status = {0};
uint8 plan_mode = 0U;
static uint8 menu_show_divider = MENU_SHOW_PERIOD_LOOPS;
static uint8 map_display_force_redraw = 1U;
static bool last_show_map_switch = true;
static bool last_show_data_switch = true;
static bool menu_config_dirty = false;
static bool menu_config_save_attempted = false;

static void Menu_Request_Redraw(void)
{
    menu_show_divider = MENU_SHOW_PERIOD_LOOPS;
}

static void Menu_Mark_Config_Dirty(void)
{
    menu_config_dirty = true;
    menu_config_save_attempted = false;
}

static void clear_display_rect(uint16 x0, uint16 y0, uint16 x1, uint16 y1)
{
    for (uint16 y = y0; y <= y1; y++)
    {
        ips200_draw_line(x0, y, x1, y, RGB565_BLACK);
    }
}

static void clear_map_display(void)
{
    clear_display_rect(0U, 199U, 160U, 319U);
    clear_display_rect(168U, 279U, 239U, 319U);
}

static void clear_data_display(void)
{
    clear_display_rect(0U, 144U, 239U, 191U);
    clear_display_rect(168U, 199U, 239U, 231U);
}

static uint8 display_object_id(uint8 id)
{
    return (id == 0xFFU) ? 0U : id;
}

static uint8 get_display_plan_mode(void)
{
    control_stage_t stage = control_get_stage();

    if (stage >= CONTROL_STAGE_PUSHBOX_LOCALIZE &&
        stage <= CONTROL_STAGE_PUSHBOX_FINISHED)
    {
        return (control_get_plan_mode() == CONTROL_PLAN_MODE_2) ? 2U : 1U;
    }

    return 0U;
}

static uint8 clamp_preset_map_index(uint8 index)
{
    if (Map_preset_count == 0U)
    {
        return 0U;
    }
    if ((size_t)index >= Map_preset_count)
    {
        return (uint8)(Map_preset_count - 1U);
    }
    return index;
}

uint8 Menu_Get_Preset_Map_Index(void)
{
    preset_map_index = clamp_preset_map_index(preset_map_index);
    return preset_map_index;
}

static void Menu_Apply_Preset_Map_Index(void)
{
    if (control_get_stage() == CONTROL_STAGE_IDLE &&
        Algorithm_Test_PresetInput_IsEnabled())
    {
        Algorithm_Test_PresetInput_Init(Menu_Get_Preset_Map_Index());
    }
}

uint8 Menu_Get_Preset_Input_Enabled(void)
{
    return preset_input_switch ? 1U : 0U;
}

static void Menu_Fill_Flash_Config(menu_flash_config_t *config)
{
    if (config == NULL)
    {
        return;
    }

    config->start_dir = startup_depart_dir_value;
    config->continuous_levels = continuous_levels_switch ? 1U : 0U;
    config->diagonal_path = diagonal_path_switch ? 1U : 0U;
    config->identify_prerotate = identify_prerotate_switch ? 1U : 0U;
    config->preset_input = preset_input_switch ? 1U : 0U;
    config->preset_map_index = Menu_Get_Preset_Map_Index();
    config->show_map = show_map_switch ? 1U : 0U;
    config->show_data = show_data_switch ? 1U : 0U;
    config->blue_serial = blue_serial_switch ? 1U : 0U;
    config->checkpoint_vision = checkpoint_vision_switch ? 1U : 0U;
}

static void Menu_Load_Flash_Config(void)
{
    menu_flash_config_t config = {0};

    if (!Data_load_from_flash(&config))
    {
        return;
    }

    startup_depart_dir_value = config.start_dir;
    if (startup_depart_dir_value > CONTROL_PRESTART_DEPART_DIR_MAX)
    {
        startup_depart_dir_value = CONTROL_PRESTART_DEPART_DIR_MAX;
    }
    continuous_levels_switch = (config.continuous_levels != 0U);
    diagonal_path_switch = (config.diagonal_path != 0U);
    identify_prerotate_switch = (config.identify_prerotate != 0U);
    preset_input_switch = (config.preset_input != 0U);
    preset_map_index = clamp_preset_map_index(config.preset_map_index);
    show_map_switch = (config.show_map != 0U);
    show_data_switch = (config.show_data != 0U);
    blue_serial_switch = (config.blue_serial != 0U);
    checkpoint_vision_switch = (config.checkpoint_vision != 0U);

    control_set_prestart_depart_dir(startup_depart_dir_value);
    control_set_continuous_levels_enabled(continuous_levels_switch ? 1U : 0U);
    control_set_diagonal_path_enabled(diagonal_path_switch ? 1U : 0U);
    control_set_identify_prerotate_enabled(identify_prerotate_switch ? 1U : 0U);
    control_set_checkpoint_vision_localization_enabled(checkpoint_vision_switch ? 1U : 0U);
    BlueSerial_SetEnabled(blue_serial_switch ? 1U : 0U);
}

static void Menu_Save_Flash_Config_If_Ready(void)
{
    menu_flash_config_t config;

    if (!menu_config_dirty || menu_config_save_attempted ||
        pointer == NULL || pointer->selected ||
        control_get_start_enabled())
    {
        return;
    }

    Menu_Fill_Flash_Config(&config);
    menu_config_save_attempted = true;
    if (Data_save_to_flash(&config))
    {
        menu_config_dirty = false;
    }
}

// 创建菜单
void Menu_Create(void)
{
    // 在此动态创建文件夹 //
    Menu_Item *Startup = Create_Menu_Folder_dynamic(&Root, "Startup");
    Menu_Item *Setting = Create_Menu_Folder_dynamic(&Root, "Setting");
    Menu_Item *Data = Create_Menu_Folder_dynamic(&Root, "Data");

    // 在此动态创建文�?//
    Create_Menu_File_dynamic(Startup, "Start", &startup_start_switch, bool_Box);
    Create_Menu_File_dynamic(Startup, "Reset", &startup_reset_switch, bool_Box);
    Create_Menu_File_dynamic(Startup, "Start_Dir", &startup_depart_dir_value, uint8_Box);

    Create_Menu_File_dynamic(Data, "Preset", &preset_input_switch, bool_Box);
    Create_Menu_File_dynamic(Data, "Map", &preset_map_index, uint8_Box);
    Create_Menu_File_dynamic(Data, "ShowMap", &show_map_switch, bool_Box);
    Create_Menu_File_dynamic(Data, "ShowData", &show_data_switch, bool_Box);

    Create_Menu_File_dynamic(Setting, "ContRun", &continuous_levels_switch, bool_Box);
    Create_Menu_File_dynamic(Setting, "DiagPath", &diagonal_path_switch, bool_Box);
    Create_Menu_File_dynamic(Setting, "PreRotate", &identify_prerotate_switch, bool_Box);
    Create_Menu_File_dynamic(Setting, "ChkVision", &checkpoint_vision_switch, bool_Box);
    Create_Menu_File_dynamic(Setting, "BlueEn", &blue_serial_switch, bool_Box);
}

static void Menu_Sync_Control_State(void)
{
    startup_start_switch = (control_get_start_enabled() != 0U);
    startup_reset_switch = false;
    startup_depart_dir_value = control_get_prestart_depart_dir();
    diagonal_path_switch = (control_get_diagonal_path_enabled() != 0U);
    identify_prerotate_switch = (control_get_identify_prerotate_enabled() != 0U);
    continuous_levels_switch = (control_get_continuous_levels_enabled() != 0U);
    checkpoint_vision_switch = (control_get_checkpoint_vision_localization_enabled() != 0U);
    blue_serial_switch = (BlueSerial_GetEnabled() != 0U);
    preset_input_switch = (Algorithm_Test_PresetInput_IsEnabled() != 0U);
}


static bool Menu_Handle_Control_Bool(Menu_Item *item, bool value)
{
    if (item == NULL || item->kind != bool_Box)
    {
        return false;
    }

    if (item->data == &startup_start_switch)
    {
        startup_start_switch = value;
        control_set_start_enabled(value ? 1U : 0U);
        return true;
    }

    if (item->data == &startup_reset_switch)
    {
        if (value)
        {
            control_restart();
            startup_start_switch = false;
        }
        startup_reset_switch = false;
        return true;
    }

    if (item->data == &diagonal_path_switch)
    {
        diagonal_path_switch = value;
        control_set_diagonal_path_enabled(value ? 1U : 0U);
        Menu_Mark_Config_Dirty();
        return true;
    }

    if (item->data == &identify_prerotate_switch)
    {
        identify_prerotate_switch = value;
        control_set_identify_prerotate_enabled(value ? 1U : 0U);
        Menu_Mark_Config_Dirty();
        return true;
    }

    if (item->data == &continuous_levels_switch)
    {
        continuous_levels_switch = value;
        control_set_continuous_levels_enabled(value ? 1U : 0U);
        Menu_Mark_Config_Dirty();
        return true;
    }

    if (item->data == &blue_serial_switch)
    {
        /* 比赛流程运行中不允许切换调车控制权。 */
        if (control_get_stage() != CONTROL_STAGE_IDLE)
        {
            blue_serial_switch = (BlueSerial_GetEnabled() != 0U);
            return true;
        }
        blue_serial_switch = value;
        BlueSerial_SetEnabled(value ? 1U : 0U);
        Menu_Mark_Config_Dirty();
        return true;
    }

    if (item->data == &checkpoint_vision_switch)
    {
        /* 推箱路径会按检查点分段，运行中切换会破坏当前分段状态。 */
        if (control_get_stage() != CONTROL_STAGE_IDLE)
        {
            checkpoint_vision_switch =
                (control_get_checkpoint_vision_localization_enabled() != 0U);
            return true;
        }
        checkpoint_vision_switch = value;
        control_set_checkpoint_vision_localization_enabled(value ? 1U : 0U);
        Menu_Mark_Config_Dirty();
        return true;
    }

    if (item->data == &preset_input_switch)
    {
        if (control_get_stage() != CONTROL_STAGE_IDLE)
        {
            control_set_start_enabled(0U);
            startup_start_switch = false;
        }
        Algorithm_Test_PresetInput_SetEnabled(value ? 1U : 0U,
                                              Menu_Get_Preset_Map_Index());
        preset_input_switch = (Algorithm_Test_PresetInput_IsEnabled() != 0U);
        Menu_Mark_Config_Dirty();
        return true;
    }

    if (item->data == &show_map_switch)
    {
        show_map_switch = value;
        Menu_Mark_Config_Dirty();
        return true;
    }

    if (item->data == &show_data_switch)
    {
        show_data_switch = value;
        Menu_Mark_Config_Dirty();
        return true;
    }

    return false;
}

static bool Menu_Handle_Control_Uint8(Menu_Item *item, int16 delta)
{
    int16 value = 0;

    if (item == NULL || item->kind != uint8_Box)
    {
        return false;
    }

    if (item->data == &startup_depart_dir_value)
    {
        value = (int16)startup_depart_dir_value + delta;
        if (value < (int16)CONTROL_PRESTART_DEPART_DIR_MIN)
        {
            value = (int16)CONTROL_PRESTART_DEPART_DIR_MIN;
        }
        else if (value > (int16)CONTROL_PRESTART_DEPART_DIR_MAX)
        {
            value = (int16)CONTROL_PRESTART_DEPART_DIR_MAX;
        }

        startup_depart_dir_value = (uint8)value;
        control_set_prestart_depart_dir(startup_depart_dir_value);
        Menu_Mark_Config_Dirty();
        return true;
    }

    if (item->data == &preset_map_index)
    {
        value = (int16)preset_map_index + delta;
        if (value < 0)
        {
            value = 0;
        }
        else if (Map_preset_count > 0U && value >= (int16)Map_preset_count)
        {
            value = (int16)Map_preset_count - 1;
        }

        preset_map_index = clamp_preset_map_index((uint8)value);
        Menu_Apply_Preset_Map_Index();
        Menu_Mark_Config_Dirty();
        return true;
    }

    return false;
}

// 菜单初始�?
void Menu_Init(void)
{
    // 显示配置
    ips200_set_dir(IPS200_PORTAIT);
    ips200_set_font(IPS200_8X16_FONT);
    ips200_set_color(RGB565_WHITE, RGB565_BLACK);
    ips200_init(IPS200_TYPE_SPI);

    // 按键初始�?
    key_init(20);

    // 菜单节点初始�?
    Root.name = "MENU";
    Root.kind = MENU_Folder;
    Root.rank = 0;
    Root.sons = 0;
    Root.data = NULL;
    Root.Father = NULL;
    Root.First_Son = NULL;
    Root.Next_Brother = NULL;
    Root.Last_Brother = NULL;

    // 菜单创建
    Menu_Create();

    // 菜单初始化处�?
    if (Root.sons != 0)
        pointer = Root.First_Son; // 指针默认指向第一个节�?
    All_Folder_Menu_Init(&Root);  // 初始化所有文件夹菜单

    Menu_Load_Flash_Config();

    ips200_draw_line(0, 129, 239, 129, RGB565_WHITE);
}

// 显示菜单标题
static void Show_title(void)
{
    char tmpchar[COLS_SUM_LEN - SETUP_NUMBER_LEN + 1];
    for (int i = 0; i < COLS_SUM_LEN - SETUP_NUMBER_LEN + 1; i++)
        tmpchar[i] = ' ';
    sprintf(tmpchar, "%s/", pointer->Father->name);
    tmpchar[strlen(tmpchar)] = ' ';
    tmpchar[COLS_SUM_LEN - SETUP_NUMBER_LEN] = '\0';
    ips200_show_string(0, SHOW_START_Y, tmpchar);
}

// 显示指针位置
void Show_Key(void)
{
    Menu_Item *r = pointer->Father;
    Menu_Item *s = r->First_Son;

    for (int i = 1; i < r->sons + 1; i++)
    {
        if (s == pointer)
        {
            ips200_show_string(0, SHOW_START_Y + FONT_H * i, "->");
        }
        else
        {
            ips200_show_string(0, SHOW_START_Y + FONT_H * i, "  ");
        }
        s = s->Next_Brother;
    }
}

// 显示步进参数
static void Show_Setup(void)
{
    char tmpchar[SETUP_NUMBER_LEN + 1];
    for (int i = 0; i < SETUP_NUMBER_LEN + 1; i++)
        tmpchar[i] = ' ';
    if (SetupNumber[SetupIndex] < 1)
        sprintf(tmpchar, "<%.2f>", SetupNumber[SetupIndex]);
    else
        sprintf(tmpchar, "<%.0f>", SetupNumber[SetupIndex]);
    tmpchar[strlen(tmpchar)] = ' ';
    tmpchar[SETUP_NUMBER_LEN] = '\0';
    ips200_show_string(COLS_SUM_LEN * FONT_W - FONT_W * SETUP_NUMBER_LEN, SHOW_START_Y, tmpchar);
}

// 显示各类数据
void Show_Number(void)
{
    Menu_Item *r = pointer->Father;
    Menu_Item *s = r->First_Son;

    for (int i = 1; i < r->sons + 1; i++)
    {
        if (s->selected)
            ips200_show_string(FONT_W * (COLS_SUM_LEN - FOLDER_NUMBER_LEN - 2), SHOW_START_Y + FONT_H * i, "*");
        else
            ips200_show_string(FONT_W * (COLS_SUM_LEN - FOLDER_NUMBER_LEN - 2), SHOW_START_Y + FONT_H * i, " ");

        switch (s->kind)
        {
        case int32_Box:
            ips200_show_int(FONT_W * (COLS_SUM_LEN - FOLDER_NUMBER_LEN - 1), SHOW_START_Y + FONT_H * i, *(int32_t *)s->data, FOLDER_NUMBER_LEN);
            break;
        case uint32_Box:
            ips200_show_uint(FONT_W * (COLS_SUM_LEN - FOLDER_NUMBER_LEN - 1), SHOW_START_Y + FONT_H * i, *(uint32_t *)(s->data), FOLDER_NUMBER_LEN);
            break;
        case int16_Box:
            ips200_show_int(FONT_W * (COLS_SUM_LEN - FOLDER_NUMBER_LEN - 1), SHOW_START_Y + FONT_H * i, *(int16_t *)(s->data), FOLDER_NUMBER_LEN);
            break;
        case uint16_Box:
            ips200_show_uint(FONT_W * (COLS_SUM_LEN - FOLDER_NUMBER_LEN - 1), SHOW_START_Y + FONT_H * i, *(uint16_t *)(s->data), FOLDER_NUMBER_LEN);
            break;
        case int8_Box:
            ips200_show_int(FONT_W * (COLS_SUM_LEN - FOLDER_NUMBER_LEN - 1), SHOW_START_Y + FONT_H * i, *(int8_t *)(s->data), FOLDER_NUMBER_LEN);
            break;
        case uint8_Box:
            ips200_show_uint(FONT_W * (COLS_SUM_LEN - FOLDER_NUMBER_LEN - 1), SHOW_START_Y + FONT_H * i, *(uint8_t *)(s->data), FOLDER_NUMBER_LEN);
            break;
        case float_Box:
            ips200_show_float(FONT_W * (COLS_SUM_LEN - FOLDER_NUMBER_LEN - 1), SHOW_START_Y + FONT_H * i, *(float *)(s->data), FOLDER_NUMBER_LEN - 4, 3);
            break;
        case bool_Box:
            if (*(bool *)(s->data))
                ips200_show_string(FONT_W * (COLS_SUM_LEN - FOLDER_NUMBER_LEN - 1), SHOW_START_Y + FONT_H * i, "On");
            else
                ips200_show_string(FONT_W * (COLS_SUM_LEN - FOLDER_NUMBER_LEN - 1), SHOW_START_Y + FONT_H * i, "Of");
        default:
            break;
        }
        s = s->Next_Brother;
    }
}

// 显示地图
void Show_Map(void)
{
    // 地图参数配置
    const uint16 map_rows = 12;
    const uint16 map_cols = 16;
    const uint16 inner_rows = 10;
    const uint16 inner_cols = 14;
    const uint16 inner_row_offset = 1;
    const uint16 inner_col_offset = 1;
    const uint16 cell_size = 10;
    // 地图起始坐标（左上角�?
    const uint16 start_x = 0;
    const uint16 start_y = 199;
    // 地图元素状态缓�?
    static uint8 inited = 0U;
    static uint8 last_cells[12][16] = {{0}};
    static uint32 last_path_sig = 0U;
    static uint32 last_exec_path_sig = 0U;
    static uint32 last_id_sig = 0U;
    uint8 curr_cells[12][16] = {{0}};
    size_t exec_steps = 0U;
    const Position *exec_path = control_get_exec_path(&exec_steps);
    // 地图元素类型位定�?
    enum
    {
        MAP_OBS = 0x01,
        MAP_BOX = 0x02,
        MAP_TAR = 0x04,
        MAP_BOM = 0x08,
        MAP_CAR = 0x10,
        MAP_PATH = 0x20
    };

    // 固定绘制外圈一整圈障碍物（12x16 边框�?
    for (uint16 r = 0; r < map_rows; r++)
    {
        curr_cells[r][0] |= MAP_OBS;
        curr_cells[r][map_cols - 1] |= MAP_OBS;
    }
    for (uint16 c = 0; c < map_cols; c++)
    {
        curr_cells[0][c] |= MAP_OBS;
        curr_cells[map_rows - 1][c] |= MAP_OBS;
    }

    // 内圈元素坐标：基�?10x14（row:0~9, col:0~13），显示时映射到 [1..10][1..14]
    for (size_t i = 0; i < Obstacles_count; i++)
    {
        int r = obstacles[i].row;
        int c = obstacles[i].col;
        if (r >= 0 && c >= 0 && r < (int)inner_rows && c < (int)inner_cols)
        {
            curr_cells[r + inner_row_offset][c + inner_col_offset] |= MAP_OBS;
        }
    }
    for (size_t i = 0; i < Boxes_count; i++)
    {
        int r = boxes[i].row;
        int c = boxes[i].col;
        if (r >= 0 && c >= 0 && r < (int)inner_rows && c < (int)inner_cols)
        {
            curr_cells[r + inner_row_offset][c + inner_col_offset] |= MAP_BOX;
        }
    }
    for (size_t i = 0; i < Targets_count; i++)
    {
        int r = targets[i].row;
        int c = targets[i].col;
        if (r >= 0 && c >= 0 && r < (int)inner_rows && c < (int)inner_cols)
        {
            curr_cells[r + inner_row_offset][c + inner_col_offset] |= MAP_TAR;
        }
    }
    for (size_t i = 0; i < Bombs_count; i++)
    {
        int r = bombs[i].row;
        int c = bombs[i].col;
        if (r >= 0 && c >= 0 && r < (int)inner_rows && c < (int)inner_cols)
        {
            curr_cells[r + inner_row_offset][c + inner_col_offset] |= MAP_BOM;
        }
    }
    for (size_t i = 0; i < Car_path_count; i++)
    {
        int r = car_path[i].row;
        int c = car_path[i].col;
        if (r >= 0 && c >= 0 && r < (int)inner_rows && c < (int)inner_cols)
        {
            curr_cells[r + inner_row_offset][c + inner_col_offset] |= MAP_PATH;
        }
    }
    if (car.row >= 0 && car.col >= 0 && car.row < (int)inner_rows && car.col < (int)inner_cols)
    {
        curr_cells[car.row + inner_row_offset][car.col + inner_col_offset] |= MAP_CAR;
    }

    // 检测地图状态是否变化（含路径序列变化），未变化则不重绘
    uint32 path_sig = 2166136261u;
    path_sig = (path_sig ^ (uint32)Car_path_count) * 16777619u;
    for (size_t i = 0; i < Car_path_count; i++)
    {
        path_sig = (path_sig ^ (uint32)(uint16)car_path[i].row) * 16777619u;
        path_sig = (path_sig ^ (uint32)(uint16)car_path[i].col) * 16777619u;
    }

    uint32 exec_path_sig = 2166136261u;
    exec_path_sig = (exec_path_sig ^ (uint32)exec_steps) * 16777619u;
    if (exec_path != NULL)
    {
        for (size_t i = 0; i < exec_steps; i++)
        {
            exec_path_sig = (exec_path_sig ^ (uint32)(uint16)exec_path[i].row) * 16777619u;
            exec_path_sig = (exec_path_sig ^ (uint32)(uint16)exec_path[i].col) * 16777619u;
            exec_path_sig = (exec_path_sig ^ (uint32)(uint16)exec_path[i].id) * 16777619u;
        }
    }

    uint32 id_sig = 2166136261u;
    id_sig = (id_sig ^ (uint32)Boxes_count) * 16777619u;
    for (size_t i = 0; i < Boxes_count; i++)
    {
        id_sig = (id_sig ^ (uint32)(uint16)boxes[i].row) * 16777619u;
        id_sig = (id_sig ^ (uint32)(uint16)boxes[i].col) * 16777619u;
        id_sig = (id_sig ^ (uint32)(uint16)display_object_id(boxes[i].id)) * 16777619u;
    }
    id_sig = (id_sig ^ (uint32)Targets_count) * 16777619u;
    for (size_t i = 0; i < Targets_count; i++)
    {
        id_sig = (id_sig ^ (uint32)(uint16)targets[i].row) * 16777619u;
        id_sig = (id_sig ^ (uint32)(uint16)targets[i].col) * 16777619u;
        id_sig = (id_sig ^ (uint32)(uint16)display_object_id(targets[i].id)) * 16777619u;
    }

    uint8 force_redraw = (map_display_force_redraw != 0U ||
                          inited == 0U ||
                          path_sig != last_path_sig ||
                          exec_path_sig != last_exec_path_sig ||
                          id_sig != last_id_sig) ? 1U : 0U;
    uint8 changed = force_redraw;
    for (uint16 r = 0; r < map_rows && !changed; r++)
    {
        for (uint16 c = 0; c < map_cols; c++)
        {
            if (curr_cells[r][c] != last_cells[r][c])
            {
                changed = 1U;
                break;
            }
        }
    }
    if (!changed)
    {
        return;
    }

    // 绘制地图元素
    for (uint16 r = 0; r < map_rows; r++)
    {
        for (uint16 c = 0; c < map_cols; c++)
        {
            if (force_redraw || curr_cells[r][c] != last_cells[r][c])
            {
                uint16 cell_x = start_x + c * cell_size;
                uint16 cell_y = start_y + r * cell_size;

                for (uint16 y = cell_y + 1U; y < cell_y + cell_size; y++)
                {
                    ips200_draw_line(cell_x + 1U, y, cell_x + cell_size - 1U, y, RGB565_BLACK);
                }

                if (curr_cells[r][c] & MAP_CAR)
                {
                    for (uint16 y = cell_y + 1U; y < cell_y + cell_size; y++)
                    {
                        ips200_draw_line(cell_x + 1U, y, cell_x + cell_size - 1U, y, RGB565_CYAN);
                    }
                }
                else if (curr_cells[r][c] & MAP_BOM)
                {
                    for (uint16 y = cell_y + 1U; y < cell_y + cell_size; y++)
                    {
                        ips200_draw_line(cell_x + 1U, y, cell_x + cell_size - 1U, y, RGB565_RED);
                    }
                }
                else if (curr_cells[r][c] & MAP_BOX)
                {
                    for (uint16 y = cell_y + 1U; y < cell_y + cell_size; y++)
                    {
                        ips200_draw_line(cell_x + 1U, y, cell_x + cell_size - 1U, y, RGB565_YELLOW);
                    }
                }
                else if (curr_cells[r][c] & MAP_TAR)
                {
                    for (uint16 y = cell_y + 1U; y < cell_y + cell_size; y++)
                    {
                        ips200_draw_line(cell_x + 1U, y, cell_x + cell_size - 1U, y, RGB565_PURPLE);
                    }
                }
                else if (curr_cells[r][c] & MAP_OBS)
                {
                    uint16 x_min = cell_x + 1U;
                    uint16 y_min = cell_y + 1U;
                    uint16 x_max = cell_x + cell_size - 1U;
                    uint16 y_max = cell_y + cell_size - 1U;
                    uint16 span = cell_size - 2U;
                    const uint16 hatch_step = 2U;

                    for (uint16 k = 0; k <= span; k += hatch_step)
                    {
                        ips200_draw_line(x_min + k, y_min, x_min, y_min + k, RGB565_WHITE);
                    }
                    for (uint16 k = hatch_step; k <= span; k += hatch_step)
                    {
                        ips200_draw_line(x_max, y_min + k, x_min + k, y_max, RGB565_WHITE);
                    }
                }
            }
        }
    }
    // 绘制网格�?
    for (uint16 i = 0; i <= map_cols; i++)
    {
        uint16 x = start_x + i * cell_size;
        ips200_draw_line(x, start_y, x, start_y + map_rows * cell_size, RGB565_WHITE);
    }
    for (uint16 j = 0; j <= map_rows; j++)
    {
        uint16 y = start_y + j * cell_size;
        ips200_draw_line(start_x, y, start_x + map_cols * cell_size, y, RGB565_WHITE);
    }

    // 显示箱子、目标点 ID（在元素上叠加数字）
    ips200_set_font(IPS200_6X8_FONT);
    for (size_t i = 0; i < Boxes_count; i++)
    {
        int r = boxes[i].row;
        int c = boxes[i].col;
        if (r < 0 || c < 0 || r >= (int)inner_rows || c >= (int)inner_cols)
        {
            continue;
        }

        uint16 cell_x = start_x + (uint16)(c + inner_col_offset) * cell_size;
        uint16 cell_y = start_y + (uint16)(r + inner_row_offset) * cell_size;

        char id_buf[8];
        sprintf(id_buf, "%u", (unsigned int)display_object_id(boxes[i].id));
        uint16 text_w = (uint16)strlen(id_buf) * 6U;
        uint16 text_x = cell_x + ((text_w < cell_size) ? ((cell_size - text_w) / 2U) : 0U);
        uint16 text_y = cell_y + 1U;

        ips200_set_color(RGB565_BLACK, RGB565_YELLOW);
        ips200_show_string(text_x, text_y, id_buf);
    }

    for (size_t i = 0; i < Targets_count; i++)
    {
        int r = targets[i].row;
        int c = targets[i].col;
        if (r < 0 || c < 0 || r >= (int)inner_rows || c >= (int)inner_cols)
        {
            continue;
        }

        uint16 cell_x = start_x + (uint16)(c + inner_col_offset) * cell_size;
        uint16 cell_y = start_y + (uint16)(r + inner_row_offset) * cell_size;

        char id_buf[8];
        sprintf(id_buf, "%u", (unsigned int)display_object_id(targets[i].id));
        uint16 text_w = (uint16)strlen(id_buf) * 6U;
        uint16 text_x = cell_x + ((text_w < cell_size) ? ((cell_size - text_w) / 2U) : 0U);
        uint16 text_y = cell_y + 1U;

        ips200_set_color(RGB565_WHITE, RGB565_PURPLE);
        ips200_show_string(text_x, text_y, id_buf);
    }
    ips200_set_font(IPS200_8X16_FONT);
    ips200_set_color(RGB565_WHITE, RGB565_BLACK);

    // 路径显示：绘制中心到中心的绿色直线，覆盖在其他元素之�?
    if (Car_path_count >= 2U)
    {
        for (size_t i = 1; i < Car_path_count; i++)
        {
            int r0 = car_path[i - 1].row;
            int c0 = car_path[i - 1].col;
            int r1 = car_path[i].row;
            int c1 = car_path[i].col;

            if (r0 < 0 || c0 < 0 || r1 < 0 || c1 < 0 ||
                r0 >= (int)inner_rows || c0 >= (int)inner_cols ||
                r1 >= (int)inner_rows || c1 >= (int)inner_cols)
            {
                continue;
            }

            uint16 x0 = start_x + (uint16)(c0 + inner_col_offset) * cell_size + cell_size / 2U;
            uint16 y0 = start_y + (uint16)(r0 + inner_row_offset) * cell_size + cell_size / 2U;
            uint16 x1 = start_x + (uint16)(c1 + inner_col_offset) * cell_size + cell_size / 2U;
            uint16 y1 = start_y + (uint16)(r1 + inner_row_offset) * cell_size + cell_size / 2U;

            ips200_draw_line(x0, y0, x1, y1, RGB565_GREEN);
        }
    }

    // 执行路径显示：执行路径来�?path_follow 坐标系，显示前先转回地图栅格坐标�?
    if (exec_path != NULL && exec_steps >= 1U)
    {
        if (exec_steps >= 2U)
        {
            for (size_t i = 1; i < exec_steps; i++)
            {
                Position p0 = exec_path[i - 1U];
                Position p1 = exec_path[i];
                path_inverse_remap_exec_point(&p0);
                path_inverse_remap_exec_point(&p1);

                int r0 = p0.row;
                int c0 = p0.col;
                int r1 = p1.row;
                int c1 = p1.col;

                if (r0 < 0 || c0 < 0 || r1 < 0 || c1 < 0 ||
                    r0 >= (int)inner_rows || c0 >= (int)inner_cols ||
                    r1 >= (int)inner_rows || c1 >= (int)inner_cols)
                {
                    continue;
                }

                uint16 x0 = start_x + (uint16)(c0 + inner_col_offset) * cell_size + cell_size / 2U;
                uint16 y0 = start_y + (uint16)(r0 + inner_row_offset) * cell_size + cell_size / 2U;
                uint16 x1 = start_x + (uint16)(c1 + inner_col_offset) * cell_size + cell_size / 2U;
                uint16 y1 = start_y + (uint16)(r1 + inner_row_offset) * cell_size + cell_size / 2U;

                ips200_draw_line(x0, y0, x1, y1, RGB565_BLUE);
            }
        }

    }

    // 记录当前地图状�?
    for (uint16 r = 0; r < map_rows; r++)
    {
        for (uint16 c = 0; c < map_cols; c++)
        {
            last_cells[r][c] = curr_cells[r][c];
        }
    }
    inited = 1U;
    last_path_sig = path_sig;
    last_exec_path_sig = exec_path_sig;
    last_id_sig = id_sig;
    map_display_force_redraw = 0U;
    // 显示地图数据
    // ips200_show_string(start_x + (map_cols + 1) * cell_size, start_y, "BOX:");
    // ips200_show_uint(start_x + (map_cols + 1) * cell_size + FONT_W * 5, start_y, Boxes_count, 2);
    // ips200_show_string(start_x + (map_cols + 1) * cell_size, start_y + FONT_H, "TAR:");
    // ips200_show_uint(start_x + (map_cols + 1) * cell_size + FONT_W * 5, start_y + FONT_H, Targets_count, 2);
    // ips200_show_string(start_x + (map_cols + 1) * cell_size, start_y + FONT_H * 2, "BOM:");
    // ips200_show_uint(start_x + (map_cols + 1) * cell_size + FONT_W * 5, start_y + FONT_H * 2, Bombs_count, 2);

}

void State_Show(void)
{
    char bt_info[27];

    ips200_show_string(168, 216, "State:");
    /*
     * 状态编号约定：
     * 0/1 为全局状态；20~24 为识别阶段；25 为识别结束回正；30~35 为推箱子阶段；99 为错误重试。
     * 这样现场调车时只看两位数字，就能先判断当前属于哪条流程�?
     */
    switch (g_control_stage)
    {
    case CONTROL_STAGE_IDLE:
        ips200_show_uint(216, 216, 0, 2);
        break;
    case CONTROL_STAGE_PRESTART_MOVE:
        ips200_show_uint(216, 216, 1, 2);
        break;
    case CONTROL_STAGE_IDENTIFY_LOCALIZE:
        ips200_show_uint(216, 216, 20, 2);
        break;
    case CONTROL_STAGE_IDENTIFY_WAIT_CAMERA_DATA:
        ips200_show_uint(216, 216, 21, 2);
        break;
    case CONTROL_STAGE_IDENTIFY_PLAN_PATH:
        ips200_show_uint(216, 216, 22, 2);
        break;
    case CONTROL_STAGE_IDENTIFY_LOAD_PATH:
        ips200_show_uint(216, 216, 23, 2);
        break;
    case CONTROL_STAGE_IDENTIFY_EXECUTE_PATH:
        ips200_show_uint(216, 216, 24, 2);
        break;
    case CONTROL_STAGE_IDENTIFY_RETURN_HEADING:
        ips200_show_uint(216, 216, 25, 2);
        break;
    case CONTROL_STAGE_PUSHBOX_LOCALIZE:
        ips200_show_uint(216, 216, 30, 2);
        break;
    case CONTROL_STAGE_PUSHBOX_WAIT_CAMERA_DATA:
        ips200_show_uint(216, 216, 31, 2);
        break;
    case CONTROL_STAGE_PUSHBOX_PLAN_PATH:
        ips200_show_uint(216, 216, 32, 2);
        break;
    case CONTROL_STAGE_PUSHBOX_LOAD_PATH:
        ips200_show_uint(216, 216, 33, 2);
        break;
    case CONTROL_STAGE_PUSHBOX_EXECUTE_PATH:
        ips200_show_uint(216, 216, 34, 2);
        break;
    case CONTROL_STAGE_PUSHBOX_FINISHED:
        ips200_show_uint(216, 216, 35, 2);
        break;
    case CONTROL_STAGE_ERROR:
        ips200_show_uint(216, 216, 99, 2);
        break;
    }

    path_follow_get_status(&path_follow_status);

    BlueSerial_GetLastRxFrame(bt_info, sizeof(bt_info));
    ips200_show_string(0, 144, "                              ");
    ips200_show_string(0, 144, "BT:");
    ips200_show_string(24, 144, bt_info);

    ips200_show_string(0, 160, "Cur:");
    ips200_show_float(40, 160, path_follow_status.x_m, 1, 3);
    ips200_show_char(88, 160, ',');
    ips200_show_float(96, 160, path_follow_status.y_m, 1, 3);
    ips200_show_float(160, 160, path_follow_status.yaw_deg, 3, 1);

    ips200_show_string(0, 176, "Tar:");
    ips200_show_float(40, 176, path_follow_status.target_x_m, 1, 3);
    ips200_show_char(88, 176, ',');
    ips200_show_float(96, 176, path_follow_status.target_y_m, 1, 3);
    ips200_show_float(160, 176, path_follow_status.target_yaw_deg, 3, 1);

    plan_mode = get_display_plan_mode();
    ips200_show_string(170, 200, "Mode:");
    ips200_show_uint(170 + FONT_W * 6, 200, plan_mode, 2);

}

// 菜单显示
void Menu_Show(void)
{

    Menu_Item *r = pointer->Father;
    Menu_Item *s = r->First_Son;

    Menu_Sync_Control_State();
    Menu_Save_Flash_Config_If_Ready();

    if (menu_show_divider < MENU_SHOW_PERIOD_LOOPS)
    {
        menu_show_divider++;
        return;
    }
    menu_show_divider = 0U;

    Show_title();
    for (int i = 1; i < r->sons + 1; i++)
    {
        ips200_show_string(FONT_W * 2, SHOW_START_Y + FONT_H * i, s->name);
        s = s->Next_Brother;
    }
    Show_Key();
    Show_Setup();
    Show_Number();

    if (!show_map_switch && last_show_map_switch)
    {
        clear_map_display();
    }
    else if (show_map_switch && !last_show_map_switch)
    {
        map_display_force_redraw = 1U;
    }

    if (!show_data_switch && last_show_data_switch)
    {
        clear_data_display();
    }

    last_show_map_switch = show_map_switch;
    last_show_data_switch = show_data_switch;

    if (show_map_switch)
    {
        Show_Map();
    }
    if (show_data_switch)
    {
        State_Show();
    }
}

// 各类按键处理
void Key_Up(void) // 指针上移
{
    Menu_Request_Redraw();
    if (pointer->Last_Brother != NULL)
        pointer = pointer->Last_Brother;
}
void Key_Down(void) // 指针下移
{
    Menu_Request_Redraw();
    if (pointer->Next_Brother != NULL)
        pointer = pointer->Next_Brother;
}
void Key_Plus(void) // 数据增大
{
    Menu_Request_Redraw();
    switch (pointer->kind)
    {
    case int32_Box:
        *(int32_t *)pointer->data += (int32_t)SetupNumber[SetupIndex];
        break;
    case uint32_Box:
        *(uint32_t *)pointer->data += (uint32_t)SetupNumber[SetupIndex];
        break;
    case int16_Box:
        *(int16_t *)pointer->data += (int16_t)SetupNumber[SetupIndex];
        break;
    case uint16_Box:
        *(uint16_t *)pointer->data += (uint16_t)SetupNumber[SetupIndex];
        break;
    case int8_Box:
        *(int8_t *)pointer->data += (int8_t)SetupNumber[SetupIndex];
        break;
    case uint8_Box:
        if (!Menu_Handle_Control_Uint8(pointer, (int16)SetupNumber[SetupIndex]))
        {
            *(uint8_t *)pointer->data += (uint8_t)SetupNumber[SetupIndex];
        }
        break;
    case float_Box:
        *(float *)pointer->data += SetupNumber[SetupIndex];
        break;
    case bool_Box:
        if (!Menu_Handle_Control_Bool(pointer, true))
        {
            *(bool *)pointer->data = true;
        }
        break;
    default:
        break;
    }
}
void Key_Sub(void) // 数据减小
{
    Menu_Request_Redraw();
    switch (pointer->kind)
    {
    case int32_Box:
        *(int32_t *)pointer->data -= (int32_t)SetupNumber[SetupIndex];
        break;
    case uint32_Box:
        *(uint32_t *)pointer->data -= (uint32_t)SetupNumber[SetupIndex];
        break;
    case int16_Box:
        *(int16_t *)pointer->data -= (int16_t)SetupNumber[SetupIndex];
        break;
    case uint16_Box:
        *(uint16_t *)pointer->data -= (uint16_t)SetupNumber[SetupIndex];
        break;
    case int8_Box:
        *(int8_t *)pointer->data -= (int8_t)SetupNumber[SetupIndex];
        break;
    case uint8_Box:
        if (!Menu_Handle_Control_Uint8(pointer, -(int16)SetupNumber[SetupIndex]))
        {
            *(uint8_t *)pointer->data -= (uint8_t)SetupNumber[SetupIndex];
        }
        break;
    case float_Box:
        *(float *)pointer->data -= SetupNumber[SetupIndex];
        break;
    case bool_Box:
        if (!Menu_Handle_Control_Bool(pointer, false))
        {
            *(bool *)pointer->data = false;
        }
        break;
    default:
        break;
    }
}
void Key_Enter(void) // 进入文件�?
{
    Menu_Request_Redraw();
    if (pointer->kind == MENU_Folder)
    {
        pointer = pointer->First_Son;
        ips200_show_string(0, 0, "                              ");
        ips200_show_string(0, 16, "                              ");
        ips200_show_string(0, 32, "                              ");
        ips200_show_string(0, 48, "                              ");
        ips200_show_string(0, 64, "                              ");
        ips200_show_string(0, 80, "                              ");
        ips200_show_string(0, 96, "                              ");
        ips200_show_string(0, 112, "                              ");
    }
}
void Key_Exit(void) // 退出文件夹
{
    Menu_Request_Redraw();
    if (pointer->Father->Father != NULL)
    {
        pointer = pointer->Father;
        ips200_show_string(0, 0, "                              ");
        ips200_show_string(0, 16, "                              ");
        ips200_show_string(0, 32, "                              ");
        ips200_show_string(0, 48, "                              ");
        ips200_show_string(0, 64, "                              ");
        ips200_show_string(0, 80, "                              ");
        ips200_show_string(0, 96, "                              ");
        ips200_show_string(0, 112, "                              ");
    }
}
void Key_Select(void) // 取消选中
{
    Menu_Request_Redraw();
    if (pointer->kind != MENU_Folder)
        pointer->selected = 1;
}
void Key_Deselect(void) // 取消选中
{
    Menu_Request_Redraw();
    if (pointer->kind != MENU_Folder)
        pointer->selected = 0;
    Menu_Save_Flash_Config_If_Ready();
}
void Key_SetupCtrl_Plus(void) // 步进值增�?
{
    Menu_Request_Redraw();
    SetupIndex = (SetupIndex + 1) % SETUP_LEN;
}
void Key_SetupCtrl_Sub(void) // 步进值减�?可返回最大�?
{
    Menu_Request_Redraw();
    SetupIndex = (SetupIndex - 1 + SETUP_LEN) % SETUP_LEN;
}

// 菜单切换
void Menu_Switch(void)
{
    // 按键在此更改
    key_state_enum k3 = key_get_state(KEY_1);
    key_state_enum k1 = key_get_state(KEY_2);
    key_state_enum k4 = key_get_state(KEY_3);
    key_state_enum k2 = key_get_state(KEY_4);

    if (k1 == KEY_SHORT_PRESS)
    {
        // 常规模式下菜单操作：选中文件时增大数据，未选中时指针上�?
        if (pointer->selected == false)
            Key_Up();
        else
            Key_Plus();
    }
    else if (k2 == KEY_SHORT_PRESS)
    {
        // 常规模式下菜单操作：选中文件时减小数据，未选中时指针下�?
        if (pointer->selected == false)
            Key_Down();
        else
            Key_Sub();
    }
    else if (k3 == KEY_SHORT_PRESS)
    {
        // control_set_start_enabled(1);

        // 常规模式下菜单操作：进入文件夹或选中/取消选中文件
        if (pointer->kind == MENU_Folder)
            Key_Enter();
        else if (pointer->selected == false)
            Key_Select();
        else
            Key_SetupCtrl_Sub();
    }

    else if (k4 == KEY_SHORT_PRESS)
    {
        // 常规模式下菜单操作：选中文件时调整步进值，未选中时退出文件夹
        if (pointer->kind != MENU_Folder && pointer->selected == true)
            Key_Deselect();
        else
            Key_Exit();
    }

    key_clear_state(KEY_1);
    key_clear_state(KEY_2);
    key_clear_state(KEY_3);
    key_clear_state(KEY_4);
}
