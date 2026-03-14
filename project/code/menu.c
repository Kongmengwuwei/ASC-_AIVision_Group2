#include "menu.h"

static Menu_Item menu_item_arr[MENU_MAX_SIZE];  //菜单项静态内存池  
static uint8_t menu_arr_index = 0;              //菜单项内存池索引

//初始化节点
static void Create_Menu_Item(Menu_Item *Father, Menu_Item *me, 
                             const char name[], void *data, Menu_Kind kind)
{
    if(Father->kind != MENU_Folder) return;     //如果父节点不是文件夹

    me->name = name;
    me->data = data;
    me->kind = kind;
    me->selected = false;
    me->sons = 0;
    me->Father = Father;
    me->First_Son = NULL;
    me->Next_Brother = NULL;
    me->Last_Brother = NULL;

    if(Father ->sons == 0)      //如果父节点没有子节点
    {
        Father->First_Son = me;
    }
    else                        //如果父节点已经有子节点
    {
        Menu_Item *Brother = Father->First_Son;
        while(Brother->Next_Brother != NULL)
        {
            Brother = Brother->Next_Brother;
        }
        Brother->Next_Brother = me;
        me->Last_Brother = Brother;
    }
    Father->sons++;
    me->rank = Father->sons;
}

//使链表节点首尾相连循环实现文件夹内文件的循环切换
void All_Folder_Menu_Init(Menu_Item *Menu)
{
    if (Menu->First_Son == NULL)
    {
        return;
    }
    Menu_Item *hp = Menu->First_Son;
    Menu_Item *p = Menu->First_Son;

    if (hp->Next_Brother == NULL)
    {
        All_Folder_Menu_Init(p);
    }
    while (p->Next_Brother != NULL)
    {
        if (p->kind == MENU_Folder)
        {
            All_Folder_Menu_Init(p);
        }
        p = p->Next_Brother;
    }
    if (hp->Next_Brother != NULL)
    {
        All_Folder_Menu_Init(p);
    }

    p->Next_Brother = hp;
    hp->Last_Brother = p;
}

//创建文件夹
void Create_Menu_Folder(Menu_Item *Father, Menu_Item *me, const char name[])
{
    Create_Menu_Item(Father, me, name, NULL, MENU_Folder);
}

//创建文件
void Create_Menu_File(Menu_Item *Father, Menu_Item *me, const char name[], void *data, Menu_Kind kind)
{
    Create_Menu_Item(Father, me, name, data, kind);
}

//动态创建文件夹
Menu_Item* Create_Menu_Folder_dynamic(Menu_Item *Father, const char name[])
{
    if(menu_arr_index >= MENU_MAX_SIZE) return NULL;    //如果内存池已满
    Menu_Item *me = &menu_item_arr[menu_arr_index++];
    Create_Menu_Item(Father, me, name, NULL, MENU_Folder);
    return me;
}

//动态创建文件
void Create_Menu_File_dynamic(Menu_Item *Father, const char name[], void *data, Menu_Kind kind)
{
    if(menu_arr_index >= MENU_MAX_SIZE) return;         //如果内存池已满
    Menu_Item *me = &menu_item_arr[menu_arr_index++];
    Create_Menu_Item(Father, me, name, data, kind);
}
