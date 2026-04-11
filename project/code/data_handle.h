#ifndef __DATA_HANDLE_H_
#define __DATA_HANDLE_H_

#include "zf_common_fifo.h"
#include "zf_common_typedef.h"
#include "zf_driver_uart.h"
#include "Map_Route_Data.h"

/* 摄像头通信串口参数 */
#define UART_INDEX UART_1
#define UART_BAUDRATE 115200
#define UART_TX_PIN UART1_TX_B12
#define UART_RX_PIN UART1_RX_B13

/* FIFO 与解析缓冲大小 */
#define FIFO_SIZE 512
#define FIFO_READ_BUF_SIZE 128
#define FRAME_BUF_SIZE 256

/* 协议标记：帧头/帧尾 */
#define MAP_FRAME_HEADER_TAG "$MAP"
#define CAR_FRAME_HEADER_TAG "$CAR"
#define FRAME_TAIL_TAG "$END"

/* 请求命令：向上位机主动请求地图或车姿 */
#define UART_CMD_MAP "MAP"
#define UART_CMD_CAR "CAR"

/* 地图字符语义（供上层业务使用） */
#define MAP_SYMBOL_OBSTACLE '#'
#define MAP_SYMBOL_EMPTY '.'
#define MAP_SYMBOL_CAR 'C'
#define MAP_SYMBOL_BOX 'B'
#define MAP_SYMBOL_TARGET 'T'
#define MAP_SYMBOL_BOMB 'D'

/* 初始化串口接收、FIFO 与模块状态 */
void uart_blob_init(void);
/* 主循环调用：读取 FIFO 并尝试解析完整帧 */
void process_blob_data(void);
/* 直接从多行字符串解析地图（不经过串口帧） */
bool parse_map_from_string(const char *map_text);

/* 串口接收中断回调（由 LPUART1_IRQHandler 调用） */
void uart_blob_rx_interrupt_handler(void);
/* 发送 "MAP" 请求命令 */
void uart_send_map_request(void);
/* 发送 "CAR" 请求命令 */
void uart_send_car_request(void);

/* 总开关：false 时 process_blob_data 不处理数据 */
extern bool uart_data_processing_enabled;

/* 最新地图数据与状态 */
extern char map_data[MAP_ROWS][MAP_COLS];
extern bool map_data_ready;
extern bool map_data_updated;

/* 最新车姿数据与状态 */
extern CarPose car_pose;
extern bool car_pose_ready;
extern bool car_pose_updated;

/* 解析计数与串口溢出标志 */
extern uint8_t map_frame_count;
extern uint8_t car_frame_count;
extern bool uart_rx_overflow_flag;

#endif
