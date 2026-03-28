#ifndef __MYMENU_H__
#define __MYMENU_H__

#include "menu.h"
#include "Algorithm_Test.h"
#include "Camera_handler.h"
#include "zf_device_key.h"
#include "zf_common_font.h"
#include "zf_device_ips200.h"

#define FONT_W (8)               //字体宽
#define FONT_H (16)              //字体高
#define SHOW_START_Y (0)         //开始行
#define COLS_SUM_LEN (30)        //屏幕一行容纳字符数量 (屏幕像素/字体宽度)
#define FOLDER_NAME_LEN (10)     //名字长度限制
#define FOLDER_NUMBER_LEN (10)   //参数长度限制
#define EVERY_FOLDER_NUMBER (7)  //每页文件数量限制
#define SETUP_LEN (5)            //步进参数数量
#define SETUP_NUMBER_LEN (6)     //步进参数显示长度限制（数值长度+小数点）

// 调参步进值
static float SetupNumber[SETUP_LEN] = {0.01, 0.1, 1, 10, 100};
static uint8_t SetupIndex = 2;

//菜单初始化
void Menu_Init(void);
//菜单显示
void Menu_Show(void);
//菜单切换
void Menu_Switch(void);

#endif
