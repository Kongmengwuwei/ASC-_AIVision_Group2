#ifndef __DATA_HANDLE_H_
#define __DATA_HANDLE_H_

#include "zf_common_fifo.h"
#include "zf_common_typedef.h"
#include "zf_driver_uart.h"
#include "Map_Path_Data.h"

/* 摄像头通信串口参数 */
#define UART_INDEX UART_1
#define UART_BAUDRATE 115200
#define UART_TX_PIN UART1_TX_B12
#define UART_RX_PIN UART1_RX_B13

/*
 * 图案/数字识别摄像头通信串口参数。
 *
 * loadmode.py 中 OpenMV 端使用 UART(12), baudrate=115200，
 * 主控端这里默认使用 UART4 与之相连。若实际接线不是 UART4，
 * 只需要改下面 4 个宏，不影响地图/车姿串口 UART1 的解析。
 */
#define VISION_UART_INDEX UART_4
#define VISION_UART_BAUDRATE 115200
#define VISION_UART_TX_PIN UART4_TX_C16
#define VISION_UART_RX_PIN UART4_RX_C17

/* FIFO 与解析缓冲大小 */
#define FIFO_SIZE 512
#define FIFO_READ_BUF_SIZE 128
#define FRAME_BUF_SIZE 256

/* 视觉识别结果是一行短文本，单独使用一套 FIFO 与行缓冲。 */
#define VISION_FIFO_SIZE 256
#define VISION_FIFO_READ_BUF_SIZE 64
#define VISION_LINE_BUF_SIZE 64
#define VISION_LABEL_MAX_LEN 24

/* 协议标记：帧头/帧尾 */
#define MAP_FRAME_HEADER_TAG "$MAP"
#define CAR_FRAME_HEADER_TAG "$CAR"
#define FRAME_TAIL_TAG "$END"

/* 请求命令：向上位机主动请求地图或车姿 */
#define UART_CMD_MAP "MAP"
#define UART_CMD_CAR "CAR"

/*
 * loadmode.py 支持的识别命令：
 * - 主控发送 "NUM\n"：调用数字模型；
 * - 主控发送 "IMG\n"：调用图案模型。
 *
 * OpenMV 返回：
 * - "NUM,<label>,<score>\n"
 * - "IMG,<label>,<score>\n"
 * 其中 score 是 0~100 的置信度百分比；识别失败返回 "<cmd>,-1,-1"。
 */
#define UART_CMD_VISION_NUM "NUM\n"
#define UART_CMD_VISION_IMG "IMG\n"

/* 地图字符语义（供上层业务使用） */
#define MAP_SYMBOL_OBSTACLE '#'
#define MAP_SYMBOL_EMPTY '.'
#define MAP_SYMBOL_CAR 'C'
#define MAP_SYMBOL_BOX 'B'
#define MAP_SYMBOL_TARGET 'T'
#define MAP_SYMBOL_BOMB 'D'

typedef enum {
    VISION_RECOGNITION_NONE = 0U,
    VISION_RECOGNITION_NUM = 1U,
    VISION_RECOGNITION_IMG = 2U
} VisionRecognitionType;

typedef struct {
    VisionRecognitionType type;       /* 本条结果来自数字模型还是图案模型。 */
    char label[VISION_LABEL_MAX_LEN]; /* 原始标签字符串，例如 "3"、"box"、"-1"。 */
    int32 label_value;                /* 当 label 是纯数字时保存数值；否则为 -1。 */
    bool label_is_number;             /* label_value 是否由有效数字转换得到。 */
    int16 score;                      /* 置信度百分比，范围 0~100；失败帧为 -1。 */
    bool success;                     /* true=识别成功；false=OpenMV 返回 -1,-1。 */
} VisionRecognitionResult;

/* 初始化串口接收、FIFO 与模块状态 */
void uart_blob_init(void);
/* 主循环调用：读取 FIFO 并尝试解析完整帧 */
void process_blob_data(void);
/* 主循环调用：读取视觉识别 UART FIFO，并按行解析 NUM/IMG 结果。 */
void process_vision_data(void);
/* 直接从多行字符串解析地图（不经过串口帧） */
bool parse_map_from_string(const char *map_text);

/* 串口接收中断回调（由 LPUART1_IRQHandler 调用） */
void uart_blob_rx_interrupt_handler(void);
/* 视觉识别串口接收中断回调（默认由 LPUART4_IRQHandler 调用）。 */
void vision_uart_rx_interrupt_handler(void);
/* 发送 "MAP" 请求命令 */
void uart_send_map_request(void);
/* 发送 "CAR" 请求命令 */
void uart_send_car_request(void);
/* 向 loadmode.py 发送数字识别请求 "NUM\n"。 */
void uart_send_vision_num_request(void);
/* 向 loadmode.py 发送图案识别请求 "IMG\n"。 */
void uart_send_vision_img_request(void);
/* 读取并清除“最新数字识别结果已更新”标志。 */
bool vision_take_num_result(VisionRecognitionResult *out_result);
/* 读取并清除“最新图案识别结果已更新”标志。 */
bool vision_take_img_result(VisionRecognitionResult *out_result);
/* 仅读取最近一次数字/图案识别结果，不清除 updated 标志。 */
bool vision_get_latest_num_result(VisionRecognitionResult *out_result);
bool vision_get_latest_img_result(VisionRecognitionResult *out_result);
/* 清空视觉识别 FIFO、半包行缓冲和 updated 标志，常用于开始一次新识别前。 */
void vision_clear_pending_data(void);

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

/* 视觉识别串口总开关：false 时 process_vision_data 直接返回。 */
extern bool vision_data_processing_enabled;

/* 最新数字识别结果与状态位。ready 表示至少收到过响应，updated 表示有新响应待处理。 */
extern VisionRecognitionResult vision_num_result;
extern bool vision_num_result_ready;
extern bool vision_num_result_updated;

/* 最新图案识别结果与状态位。ready 表示至少收到过响应，updated 表示有新响应待处理。 */
extern VisionRecognitionResult vision_img_result;
extern bool vision_img_result_ready;
extern bool vision_img_result_updated;

/* 视觉识别响应计数与异常标志。 */
extern uint8_t vision_num_frame_count;
extern uint8_t vision_img_frame_count;
extern uint8_t vision_bad_line_count;
extern bool vision_uart_rx_overflow_flag;
extern bool vision_line_overflow_flag;
extern bool vision_parse_error_flag;

#endif
