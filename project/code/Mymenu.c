#include "Mymenu.h"

Menu_Item Root; // 根目录
Menu_Item *Key; // 指针

int32_t test1 = 1234; // 测试数据
float test2 = 123.45; // 测试数据
uint8_t test3 = 1; // 测试数据
bool test4 = true; // 测试数据

bool Algo_Test = false; // 算法测试开关

// 创建菜单
void Menu_Create(void)
{
    // 在此动态创建文件夹 //
    Menu_Item *Folder1 = Create_Menu_Folder_dynamic(&Root, "Mode");
    Menu_Item *Folder2 = Create_Menu_Folder_dynamic(&Root, "Folder2");
    Menu_Item *Folder3 = Create_Menu_Folder_dynamic(&Root, "Folder3");
    Menu_Item *Folder4 = Create_Menu_Folder_dynamic(&Root, "Folder4");
    Menu_Item *Folder5 = Create_Menu_Folder_dynamic(&Root, "Folder5");

    // 在此动态创建文件 //
    Create_Menu_File_dynamic(Folder1, "ALgo_Test", &Algo_Test, bool_Box);
    Create_Menu_File_dynamic(Folder2, "File2", &test2, float_Box);
    Create_Menu_File_dynamic(Folder3, "File3", &test3, uint8_Box);
    Create_Menu_File_dynamic(Folder4, "File4", &test4, bool_Box);
    Create_Menu_File_dynamic(Folder5, "File5", &test1, int32_Box);
}

// 菜单初始化
void Menu_Init(void)
{
    // 显示配置
    ips200_set_dir(IPS200_PORTAIT);
    ips200_set_font(IPS200_8X16_FONT);
    ips200_set_color(RGB565_WHITE, RGB565_BLACK);
    ips200_init(IPS200_TYPE_SPI);

    // 按键初始化
    key_init(20);

    // 菜单节点初始化
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

    // 菜单初始化处理
    if (Root.sons != 0)
        Key = Root.First_Son;    // 指针默认指向第一个节点
    All_Folder_Menu_Init(&Root); // 初始化所有文件夹菜单
}

// 显示菜单标题
static void Show_title(void)
{
    char tmpchar[COLS_SUM_LEN - SETUP_NUMBER_LEN + 1];
    for (int i = 0; i < COLS_SUM_LEN - SETUP_NUMBER_LEN + 1; i++)
        tmpchar[i] = ' ';
    sprintf(tmpchar, "%s/", Key->Father->name);
    tmpchar[strlen(tmpchar)] = ' ';
    tmpchar[COLS_SUM_LEN - SETUP_NUMBER_LEN] = '\0';
    ips200_show_string(0, SHOW_START_Y, tmpchar);
}

// 显示指针位置
void Show_Key(void)
{
    Menu_Item *r = Key->Father;
    Menu_Item *s = r->First_Son;

    for (int i = 1; i < r->sons + 1; i++)
    {
        if (s == Key)
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
    Menu_Item *r = Key->Father;
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
                ips200_show_string(FONT_W * (COLS_SUM_LEN - FOLDER_NUMBER_LEN - 1), SHOW_START_Y + FONT_H * i, "Off");
        default:
            break;
        }
        s = s->Next_Brother;
    }
}

// 显示地图
void Show_Map(void)
{
    const uint16 map_rows = 12;
    const uint16 map_cols = 16;
    const uint16 cell_size = 10;
    const uint16 start_x = 0;
    const uint16 start_y = 199;

    static uint8 inited = 0U;
    static uint8 last_cells[12][16] = {{0}};
    uint8 curr_cells[12][16] = {{0}};

    enum
    {
        OBS = 0x01,
        BOX = 0x02,
        TAR = 0x04,
        BOM = 0x08,
        CAR = 0x10
    };

    for (size_t i = 0; i < actual_obstacles_count; i++)
    {
        int r = obstacles[i].row;
        int c = obstacles[i].col;
        if (r >= 0 && c >= 0 && r < (int)map_rows && c < (int)map_cols)
        {
            curr_cells[r][c] |= OBS;
        } 
    }

    for (size_t i = 0; i < actual_boxes_count; i++)
    {
        int r = boxes[i].row;
        int c = boxes[i].col;
        if (r >= 0 && c >= 0 && r < (int)map_rows && c < (int)map_cols)
        {
            curr_cells[r][c] |= CELL_BOX;
        }
    }

    for (size_t i = 0; i < actual_targets_count; i++)
    {
        int r = targets[i].row;
        int c = targets[i].col;
        if (r >= 0 && c >= 0 && r < (int)map_rows && c < (int)map_cols)
        {
            curr_cells[r][c] |= TAR;
        }
    }

    for (size_t i = 0; i < actual_bombs_count; i++)
    {
        int r = bombs[i].row;
        int c = bombs[i].col;
        if (r >= 0 && c >= 0 && r < (int)map_rows && c < (int)map_cols)
        {
            curr_cells[r][c] |= BOM;
        }
    }

    if (car.row >= 0 && car.col >= 0 && car.row < (int)map_rows && car.col < (int)map_cols)
    {
        curr_cells[car.row][car.col] |= CAR;
    }

    uint8 changed = (inited == 0U) ? 1U : 0U;
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

    for (uint16 r = 0; r < map_rows; r++)
    {
        for (uint16 c = 0; c < map_cols; c++)
        {
            if (!inited || curr_cells[r][c] != last_cells[r][c])
            {
                uint16 cell_x = start_x + c * cell_size;
                uint16 cell_y = start_y + r * cell_size;

                for (uint16 y = cell_y + 1U; y < cell_y + cell_size; y++)
                {
                    ips200_draw_line(cell_x + 1U, y, cell_x + cell_size - 1U, y, RGB565_BLACK);
                }

                if (curr_cells[r][c] & CAR)
                {
                    for (uint16 y = cell_y + 1U; y < cell_y + cell_size; y++)
                    {
                        ips200_draw_line(cell_x + 1U, y, cell_x + cell_size - 1U, y, RGB565_CYAN);
                    }
                }
                else if (curr_cells[r][c] & BOM)
                {
                    for (uint16 y = cell_y + 1U; y < cell_y + cell_size; y++)
                    {
                        ips200_draw_line(cell_x + 1U, y, cell_x + cell_size - 1U, y, RGB565_RED);
                    }
                }
                else if (curr_cells[r][c] & CELL_BOX)
                {
                    for (uint16 y = cell_y + 1U; y < cell_y + cell_size; y++)
                    {
                        ips200_draw_line(cell_x + 1U, y, cell_x + cell_size - 1U, y, RGB565_YELLOW);
                    }
                }
                else if (curr_cells[r][c] & TAR)
                {
                    for (uint16 y = cell_y + 1U; y < cell_y + cell_size; y++)
                    {
                        ips200_draw_line(cell_x + 1U, y, cell_x + cell_size - 1U, y, RGB565_PURPLE);
                    }
                }
                else if (curr_cells[r][c] & OBS)
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

    for (uint16 r = 0; r < map_rows; r++)
    {
        for (uint16 c = 0; c < map_cols; c++)
        {
            last_cells[r][c] = curr_cells[r][c];
        }
    }
    inited = 1U;

    ips200_show_string(start_x + (map_cols + 1) * cell_size, start_y, "BOX:");
    ips200_show_uint(start_x + (map_cols + 1) * cell_size + FONT_W * 5, start_y, actual_boxes_count, 2);
    ips200_show_string(start_x + (map_cols + 1) * cell_size, start_y + FONT_H, "TAR:");
    ips200_show_uint(start_x + (map_cols + 1) * cell_size + FONT_W * 5, start_y + FONT_H, actual_targets_count, 2);
    ips200_show_string(start_x + (map_cols + 1) * cell_size, start_y + FONT_H * 2, "BOM:");
    ips200_show_uint(start_x + (map_cols + 1) * cell_size + FONT_W * 5, start_y + FONT_H * 2, actual_bombs_count, 2);
}

// 菜单显示
void Menu_Show(void)
{
    Menu_Item *r = Key->Father;
    Menu_Item *s = r->First_Son;

    Show_title();
    for (int i = 1; i < r->sons + 1; i++)
    {
        ips200_show_string(FONT_W * 2, SHOW_START_Y + FONT_H * i, s->name);
        s = s->Next_Brother;
    }
    Show_Key();
    Show_Setup();
    Show_Number();

    Show_Map();
}

// 各类按键处理
void Key_Up(void) // 指针上移
{
    if (Key->Last_Brother != NULL)
        Key = Key->Last_Brother;
}
void Key_Down(void) // 指针下移
{
    if (Key->Next_Brother != NULL)
        Key = Key->Next_Brother;
}
void Key_Plus(void) // 数据增大
{
    switch (Key->kind)
    {
    case int32_Box:
        *(int32_t *)Key->data += (int32_t)SetupNumber[SetupIndex];
        break;
    case uint32_Box:
        *(uint32_t *)Key->data += (uint32_t)SetupNumber[SetupIndex];
        break;
    case int16_Box:
        *(int16_t *)Key->data += (int16_t)SetupNumber[SetupIndex];
        break;
    case uint16_Box:
        *(uint16_t *)Key->data += (uint16_t)SetupNumber[SetupIndex];
        break;
    case int8_Box:
        *(int8_t *)Key->data += (int8_t)SetupNumber[SetupIndex];
        break;
    case uint8_Box:
        *(uint8_t *)Key->data += (uint8_t)SetupNumber[SetupIndex];
        break;
    case float_Box:
        *(float *)Key->data += SetupNumber[SetupIndex];
        break;
    case bool_Box:
        *(bool *)Key->data = true;
    default:
        break;
    }
}
void Key_Sub(void) // 数据减小
{
    switch (Key->kind)
    {
    case int32_Box:
        *(int32_t *)Key->data -= (int32_t)SetupNumber[SetupIndex];
        break;
    case uint32_Box:
        *(uint32_t *)Key->data -= (uint32_t)SetupNumber[SetupIndex];
        break;
    case int16_Box:
        *(int16_t *)Key->data -= (int16_t)SetupNumber[SetupIndex];
        break;
    case uint16_Box:
        *(uint16_t *)Key->data -= (uint16_t)SetupNumber[SetupIndex];
        break;
    case int8_Box:
        *(int8_t *)Key->data -= (int8_t)SetupNumber[SetupIndex];
        break;
    case uint8_Box:
        *(uint8_t *)Key->data -= (uint8_t)SetupNumber[SetupIndex];
        break;
    case float_Box:
        *(float *)Key->data -= SetupNumber[SetupIndex];
        break;
    case bool_Box:
        *(bool *)Key->data = false;
        break;
    default:
        break;
    }
}
void Key_Enter(void) // 进入文件夹
{
    if (Key->kind == MENU_Folder)
    {
        Key = Key->First_Son;
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
    if (Key->Father->Father != NULL)
    {
        Key = Key->Father;
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
void Key_Select(void) // 选中/取消选中
{
    if (Key->kind != MENU_Folder)
        Key->selected = !Key->selected;
}
void Key_SetupCtrl_Plus(void) // 步进值增大
{
    SetupIndex = (SetupIndex + 1) % SETUP_LEN;
}
void Key_SetupCtrl_Sub(void) // 步进值减小(可返回最大值)
{
    SetupIndex = (SetupIndex - 1 + SETUP_LEN) % SETUP_LEN;
}

// 菜单切换
void Menu_Switch(void)
{
    // 按键在此更改
    key_state_enum k2 = key_get_state(KEY_1);
    key_state_enum k1 = key_get_state(KEY_2);
    key_state_enum k3 = key_get_state(KEY_3);
    key_state_enum k4 = key_get_state(KEY_4);

    if (k1 == KEY_SHORT_PRESS)
    {   
        Move_car('W'); // 测试用小车向上移动

        if (Key->selected == false)
            Key_Up();
        else
            Key_Plus();
    }
    else if (k2 == KEY_SHORT_PRESS)
    {
        Move_car('S'); // 测试用小车向下移动

        if (Key->selected == false)
            Key_Down();
        else
            Key_Sub();
    }
    else if (k3 == KEY_SHORT_PRESS)
    {
        Move_car('A'); // 测试用小车向左移动

        if (Key->kind == MENU_Folder)
            Key_Enter();
        else
            Key_Select();
    }
    else if (k4 == KEY_SHORT_PRESS)
    {
        Move_car('D'); // 测试用小车向右移动

        if (Key->kind != MENU_Folder && Key->selected == true)
            Key_SetupCtrl_Sub();
        else
            Key_Exit();
    }

    key_clear_state(KEY_1);
    key_clear_state(KEY_2);
    key_clear_state(KEY_3);
    key_clear_state(KEY_4);
}
