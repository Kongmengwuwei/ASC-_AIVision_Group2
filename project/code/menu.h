#ifndef __MENU_H
#define __MENU_H

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#define MENU_MAX_SIZE 64         //菜单项最大数量

//菜单项类型
typedef enum Menu_Kind
{
    MENU_Folder = 0,
    int32_Box,
    uint32_Box,
    int16_Box,
    uint16_Box,
    int8_Box,
    uint8_Box,
    float_Box,
    bool_Box, 
} Menu_Kind;

//菜单项结构体
typedef struct Menu_Item
{
    const char *name;                   //菜单项名称
    void *data;                         //指向存放的变量
    Menu_Kind kind;                     //记录节点的类型

    uint8_t sons;                       //记录子文件数量
    uint8_t rank;                       //记录当前所处文件夹下的位次
    bool selected;                      //记录是否被选中

    struct Menu_Item *Father;           //所处文件夹节点
    struct Menu_Item *First_Son;        //第一个子文件/子文件夹节点
    struct Menu_Item *Next_Brother;     //下一个文件/文件夹节点
    struct Menu_Item *Last_Brother;     //上一个文件/文件夹节点
    
} Menu_Item;

// 使链表节点首尾相连循环实现文件夹内文件的循环切换
void All_Folder_Menu_Init(Menu_Item *Menu);    

// 创建文件夹     参数：(父文件夹地址，新建文件夹地址，文件夹名称)
void Create_Menu_Folder(Menu_Item *Father, Menu_Item *me, const char name[]);
//创建文件       参数：(父文件夹地址，新建文件夹地址，文件名称，文件数据的地址，文件类型)
void Create_Menu_File(Menu_Item *Father, Menu_Item *me, const char name[], void *data, Menu_Kind kind);

//动态创建文件夹  参数：(父文件夹地址，新建文件夹名称)
Menu_Item* Create_Menu_Folder_dynamic(Menu_Item *Father, const char name[]);
//动态创建文件    参数：(父文件夹地址，新建文件名称，文件数据的地址，文件类型)
void Create_Menu_File_dynamic(Menu_Item *Father, const char name[], void *data, Menu_Kind kind);

#endif
