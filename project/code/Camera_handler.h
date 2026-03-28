#ifndef __CAMERA_HANDLER_H_
#define __CAMERA_HANDLER_H_

// #include "assigned_box_planner_greedy_2.h"
#include "zf_common_fifo.h"
#include "zf_common_typedef.h"
#include "zf_driver_uart.h"

// 串口配置：用于接收摄像头发送的数据流
#define UART_INDEX UART_1
#define UART_BAUDRATE 115200
#define UART_TX_PIN UART1_TX_B12
#define UART_RX_PIN UART1_RX_B13

// 缓冲区与图像尺寸相关配置
#define FIFO_SIZE 1024
#define FIFO_READ_BUF_SIZE 512
#define FRAME_BUF_SIZE 512
#define MID_CAMERA_WIDTH 160
#define MID_CAMERA_HEIGHT 120
#define CAMERA_WIDTH 320
#define CAMERA_HEIGHT 240

// 地图尺寸与符号协议定义（与串口文本协议对应）
#define MAP_ROWS 12
#define MAP_COLS 16
#define MAP_SYMBOL_OBSTACLE '#'
#define MAP_SYMBOL_EMPTY '.'
#define MAP_SYMBOL_CAR 'C'
#define MAP_SYMBOL_BOX 'B'
#define MAP_SYMBOL_TARGET 'T'
#define MAP_SYMBOL_BOMB 'D'

typedef struct
{
    int row; // 栅格行号
    int col; // 栅格列号
} Point;

typedef struct
{
    float row; // 车辆行坐标（可带小数）
    float col; // 车辆列坐标（可带小数）
} CarPosition;

typedef struct
{
    int16_t cx;   // 色块中心x（像素）
    int16_t cy;   // 色块中心y（像素）
    int16_t w;    // 色块宽度（像素）
    int16_t h;    // 色块高度（像素）
    int32_t area; // 色块面积（像素^2）
    uint8_t valid; // 识别结果是否有效
    int w_error;   // 相对期望宽度的误差
    float distance; // 距离估计值（单位由上层约定）
} BlobInfo;

#define MAX_OBSTACLES 100
#define MAX_BOXES 10
#define MAX_TARGETS 10
#define MAX_BOMBS 10
#define MAX_CAR_PATH 250

#define NO_PIXEL_TO_GRID_RATIO 1
#define ROW_PIXEL_TO_GRID_RATIO 10.67f
#define COL_PIXEL_TO_GRID_RATIO 10.13f

#define GRID_SIZE_M 0.20f
#define ENABLE_DYNAMIC_MAP 0U

// 地图接收状态机：先接收初始全图，再按需进入动态更新
typedef enum
{
    MAP_STATE_INIT = 0,
    MAP_STATE_DYNAMIC
} map_state_t;

// 初始化串口/FIFO与模块内部状态
void uart_blob_init(void);
// 主处理入口：从FIFO取串口数据并完成协议解析
void process_blob_data(void);
// 像素坐标转换为栅格坐标
void pixel_to_grid(int pixel_row, int pixel_col, float *grid_row, float *grid_col,
                   float grid_ratio_row, float grid_ratio_col);
// 直接从多行字符串解析地图（不经过串口帧）
bool parse_map_from_string(const char *map_text);

// 模块运行状态
extern map_state_t map_state;              // 当前地图解析状态（初始/动态）
extern bool initial_map_ready;             // 初始地图是否已经解析成功
extern bool uart_data_processing_enabled;  // 串口数据解析总开关
extern int32 dynamic_map_enable;           // 是否启用动态更新解析（0关闭，非0开启）

// 当前生效对象数量
extern size_t actual_obstacles_count; // 当前障碍物数量
extern size_t actual_boxes_count;     // 当前箱子数量
extern size_t actual_targets_count;   // 当前目标点数量
extern size_t actual_bombs_count;     // 当前炸弹数量
extern size_t actual_car_path_count;  // 当前路径点数量（若使用）

// 当前生效地图对象与车辆位置
extern Point obstacles[MAX_OBSTACLES];                   // 障碍物坐标列表
extern Point boxes[MAX_BOXES];                           // 箱子坐标列表
extern Point targets[MAX_TARGETS];                       // 目标点坐标列表
extern Point bombs[MAX_BOMBS];                       // 炸弹坐标列表
extern Point car_path[MAX_CAR_PATH];                     // 规划路径坐标列表（命名沿用原工程）
extern Point car;                                        // 车辆整数栅格位置
extern CarPosition car_position;            // 车辆浮点栅格位置
extern CarPosition car_position_m;          // 车辆米制位置（row/col * GRID_SIZE_M）
extern volatile bool car_position_valid;    // 车辆位置是否有效

// 帧级状态
extern uint8_t init_map_received_count; // 成功接收到初始地图的次数
extern bool current_round_complete;     // 当前一轮解析是否完成
extern bool data_reception_complete;    // 初始地图接收是否完成

// 数据到位标记（供其他模块快速判断）
extern uint8_t get_data_1; // 车辆数据到位标记
extern uint8_t get_data_2; // 障碍物数据到位标记
extern uint8_t get_data_3; // 箱子数据到位标记
extern uint8_t get_data_4; // 目标点数据到位标记

// 串口FIFO与解析辅助状态
extern fifo_struct uart_data_fifo; // 串口原始字节FIFO
extern BlobInfo blob_info;         // 色块识别信息缓存
extern uint32 format_count;        // 成功完成的解析轮次计数
extern uint8_t image_find_flag;    // 图像定位异常标记（越界等）

#endif
