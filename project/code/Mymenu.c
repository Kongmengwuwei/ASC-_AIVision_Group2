#include "Mymenu.h"
#include "zf_device_key.h"
#include "zf_common_font.h"
#include "zf_device_ips200.h"

Menu_Item Root; // 根节点
Menu_Item *Key; // 当前节点指针

int32_t test1 = 1234; // 测试数据
float test2 = 123.45; // 测试数据
uint8_t test3 = 1; // 测试数据
bool test4 = true; // 测试数据

// 菜单创建
void Menu_Create(void)
{
    // 在此动态创建文件夹//
    Menu_Item *Folder1 = Create_Menu_Folder_dynamic(&Root, "Folder1");
    Menu_Item *Folder2 = Create_Menu_Folder_dynamic(&Root, "Folder2");
    Menu_Item *Folder3 = Create_Menu_Folder_dynamic(&Root, "Folder3");
    Menu_Item *Folder4 = Create_Menu_Folder_dynamic(&Root, "Folder4");
    Menu_Item *Folder5 = Create_Menu_Folder_dynamic(&Root, "Folder5");

    // 在此动态创建各类文件//
    Create_Menu_File_dynamic(Folder1, "File1", &test1, int32_Box);
    Create_Menu_File_dynamic(Folder1, "File2", &test2, float_Box);
    Create_Menu_File_dynamic(Folder1, "File3", &test3, uint8_Box);
    Create_Menu_File_dynamic(Folder2, "File4", &test4, bool_Box);
}

// 菜单初始化
void Menu_Init(void)
{
    // 显示屏初始化
    ips200_set_dir(IPS200_PORTAIT);
    ips200_set_font(IPS200_8X16_FONT);
    ips200_set_color(RGB565_WHITE, RGB565_BLACK);
    ips200_init(IPS200_TYPE_SPI);

    // 按键初始化
    key_init(20);

    // 根节点初始化
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
    All_Folder_Menu_Init(&Root); // 使节点首位相连
}

// 显示路径标题
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

// 显示步进值
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
}

// 各类指针操作
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
        ips200_clear();
    }
}
void Key_Exit(void) // 退出文件夹
{
    if (Key->Father->Father != NULL)
    {
        Key = Key->Father;
        ips200_clear();
    }
}
void Key_Select(void) // 选择/取消选择
{
    if (Key->kind != MENU_Folder)
        Key->selected = !Key->selected;
}
void Key_SetupCtrl_Plus(void) // 步进参数减小
{
    SetupIndex = (SetupIndex + 1) % SETUP_LEN;
}
void Key_SetupCtrl_Sub(void) // 步进参数增大(可回到最小值)
{
    SetupIndex = (SetupIndex - 1 + SETUP_LEN) % SETUP_LEN;
}

// 菜单切换
void Menu_Switch(void)
{
    key_state_enum k3 = key_get_state(KEY_1);
    key_state_enum k2 = key_get_state(KEY_2);
    key_state_enum k1 = key_get_state(KEY_3);
    key_state_enum k4 = key_get_state(KEY_4);

    if (k1 == KEY_SHORT_PRESS)
    {
        if (Key->selected == false)
            Key_Up();
        else
            Key_Plus();
    }
    else if (k2 == KEY_SHORT_PRESS)
    {
        if (Key->selected == false)
            Key_Down();
        else
            Key_Sub();
    }
    else if (k3 == KEY_SHORT_PRESS)
    {
        if (Key->kind == MENU_Folder)
            Key_Enter();
        else
            Key_Select();
    }
    else if (k4 == KEY_SHORT_PRESS)
    {
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
