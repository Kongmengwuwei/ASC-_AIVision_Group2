#include "Mymenu.h"
#include "zf_device_key.h"
#include "zf_common_font.h"
#include "zf_device_ips200.h"

Menu_Item Root;      //根节点
Menu_Item *Key;      //当前节点指针

int32_t test = 1234; //测试数据

//菜单创建
void Menu_Create(void)
{
    //在此动态创建文件夹//
    Menu_Item *Folder1 = Create_Menu_Folder_dynamic(&Root,"Folder1");
    Menu_Item *Folder2 = Create_Menu_Folder_dynamic(&Root,"Folder2");
    Menu_Item *Folder3 = Create_Menu_Folder_dynamic(&Root,"Folder3");
    Menu_Item *Folder4 = Create_Menu_Folder_dynamic(&Root,"Folder4");
    Menu_Item *Folder5 = Create_Menu_Folder_dynamic(&Root,"Folder5");

    //在此动态创建各类文件//
    Create_Menu_File_dynamic(Folder1, "File1", &test, int32_Box);
    Create_Menu_File_dynamic(Folder1, "File2", &test, int32_Box);
    Create_Menu_File_dynamic(Folder1, "File3", &test, int32_Box);
}

//菜单初始化
void Menu_Init(void)
{
    //显示屏初始化
    ips200_set_dir(IPS200_PORTAIT);
    ips200_set_font(IPS200_8X16_FONT);
    ips200_set_color(RGB565_WHITE, RGB565_BLACK);
    ips200_init(IPS200_TYPE_SPI);

    //按键初始化
    key_init(20);
	
    //根节点初始化
    Root.name = "MENU";
    Root.kind = MENU_Folder;
    Root.rank = 0;
    Root.sons = 0;
    Root.data = NULL;
    Root.Father = NULL;
    Root.First_Son = NULL;  
    Root.Next_Brother = NULL;
    Root.Last_Brother = NULL;
    
    //菜单创建
    Menu_Create();

    //菜单初始化处理
    if(Root.sons != 0) Key = Root.First_Son;           //指针默认指向第一个节点
    All_Folder_Menu_Init(&Root);
}

//显示指针位置
void Show_Key(void)
{
    Menu_Item *r = Key->Father;
    Menu_Item *s = r->First_Son;

    for(int i = 1; i < r->sons + 1; i++)
    {
        if(s == Key)
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

//显示各类数据
void Show_Number(void)
{
    Menu_Item *r = Key->Father;
    Menu_Item *s = r->First_Son;

    for(int i = 1; i < r->sons + 1; i++)
    {
        if(s->selected)
        {
            ips200_show_string(72, SHOW_START_Y + FONT_H * i, "*");
        }
        else
        {
            ips200_show_string(72, SHOW_START_Y + FONT_H * i, " ");
        }
        
        switch (s->kind)
        { 
            case int32_Box:
                ips200_show_int(80, SHOW_START_Y + FONT_H * i, *(int32_t *)s->data, 10);
                break;
            case uint32_Box:
                ips200_show_uint(80, SHOW_START_Y + FONT_H * i, *(uint32_t *)(s->data), 10);
                break;
            case int16_Box:
                ips200_show_int(80, SHOW_START_Y + FONT_H * i, *(int16_t *)(s->data), 10);
                break;
            case uint16_Box:
                ips200_show_uint(80, SHOW_START_Y + FONT_H * i, *(uint16_t *)(s->data), 10);
                break;
            case int8_Box:
                ips200_show_int(80, SHOW_START_Y + FONT_H * i, *(int8_t *)(s->data), 10);
                break;
            case uint8_Box:
                ips200_show_uint(80, SHOW_START_Y + FONT_H * i, *(uint8_t *)(s->data), 10);
                break;
            case float_Box:
                ips200_show_float(80, SHOW_START_Y + FONT_H * i, *(float *)(s->data), 10, 2);
                break;
            case bool_Box:
                if(*(bool *)(s->data))
                    ips200_show_string(80, SHOW_START_Y + FONT_H * i, "1");
                else
                    ips200_show_string(80, SHOW_START_Y + FONT_H * i, "0");
            default:
                break;
        }
        s = s->Next_Brother;
    }
}

//菜单显示
void Menu_Show(void)
{
    Menu_Item *r = Key->Father;
    Menu_Item *s = r->First_Son;

    ips200_show_string(0, SHOW_START_Y, Key->Father->name);

    for(int i = 1; i < r->sons + 1; i++)
    {
        ips200_show_string(FONT_W * 2, SHOW_START_Y + FONT_H * i, s->name);
        s = s->Next_Brother;
    }

    Show_Key();
    Show_Number();
}

//各类指针操作
void Key_Up(void)       //指针上移
{
    if(Key->Last_Brother != NULL)
        Key = Key->Last_Brother;
}
void Key_Down(void)     //指针下移
{
    if(Key->Next_Brother != NULL)
        Key = Key->Next_Brother;
}
void Key_Plus(void)     //数值加
{
    switch(Key->kind)
    {
        case int32_Box:
            *(int32_t *)Key->data +=1;
            break;
        default:
            break;
    }
}
void Key_Sub(void)      //数值减
{
    switch(Key->kind)
    {
        case int32_Box:
            *(int32_t *)Key->data -=1;
            break;
        default:
            break;
    }
}
void Key_Enter(void)    //进入文件夹
{
    if(Key->kind == MENU_Folder)
    {    
        Key = Key->First_Son;
        ips200_clear();
    }
}
void Key_Exit(void)     //退出文件夹
{
    if(Key->Father->Father != NULL)
    {
        Key = Key->Father;
        ips200_clear();
    }
}   
void Key_Select(void)   //选择/取消选择
{
    if(Key->kind != MENU_Folder && Key->kind != bool_Box)
        Key->selected = !Key->selected;
}

//菜单切换   
void Menu_Switch(void)
{
    key_state_enum k3 = key_get_state(KEY_1);
    key_state_enum k2 = key_get_state(KEY_2);
    key_state_enum k1 = key_get_state(KEY_3);
    key_state_enum k4 = key_get_state(KEY_4);

    if (k1 == KEY_SHORT_PRESS)
    {
        if(Key->selected == false)
            Key_Up();
        else
            Key_Plus();
    } 
    else if (k2 == KEY_SHORT_PRESS)
    {
        if(Key->selected == false)
            Key_Down();
        else
            Key_Sub();
    }
    else if (k3 == KEY_SHORT_PRESS)
    {
        switch (Key->kind)
        {
        case MENU_Folder:
            Key_Enter();
            break;       
        default:
            Key_Select();
            break;
        }
    }
    else if (k4 == KEY_SHORT_PRESS)
    {
        Key_Exit();
    }

    key_clear_state(KEY_1);
    key_clear_state(KEY_2);
    key_clear_state(KEY_3);
    key_clear_state(KEY_4);
}
