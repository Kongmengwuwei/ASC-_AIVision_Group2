#include "Map_Route_Data.h"

// 当前生效地图对象数量
size_t Obstacles_count = 0; // 当前障碍物数量
size_t Boxes_count = 0;     // 当前箱子数量
size_t Targets_count = 0;   // 当前目标点数量
size_t Bombs_count = 0;     // 当前炸弹数量
size_t Car_path_count = 0;  // 当前路径点数量

// 当前生效地图对象位置
Position obstacles[MAX_OBSTACLES] = {{0}}; // 当前障碍物坐标列表
Position boxes[MAX_BOXES] = {{0}};         // 当前箱子坐标列表
Position targets[MAX_TARGETS] = {{0}};     // 当前目标点坐标列表
Position bombs[MAX_BOMBS] = {{0}};         // 当前炸弹坐标列表
Position car = {1, 2};                     // 车辆整数栅格位置
Position car_path[MAX_CAR_PATH] = {{0}};   // 预留路径坐标列表
CarPose car_position = {0.0f, 0.0f};   // 车辆浮点栅格位置
CarPose car_position_m = {0.0f, 0.0f}; // 车辆米制坐标
