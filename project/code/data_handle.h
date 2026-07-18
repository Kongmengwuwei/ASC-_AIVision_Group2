#ifndef __DATA_HANDLE_H_
#define __DATA_HANDLE_H_

#include "zf_common_fifo.h"
#include "zf_common_typedef.h"
#include "zf_driver_uart.h"
#include "Map_Path_Data.h"
#include "Motor.h"

/* 摄像头通信串口参数 */
#define UART_INDEX UART_1
#define UART_BAUDRATE 115200
#define UART_TX_PIN UART1_TX_B12
#define UART_RX_PIN UART1_RX_B13

/*
 * 图案/数字识别摄像头通信串口参数。
 *
 * OpenMV 端使用 UART(12), baudrate=115200。
 * 新主板使用 UART4/C16/C17；旧主板使用 UART8/D16/D17。
 * 地图/车姿串口 UART1 不受影响。
 */
#if MOTOR_BOARD_USE_NEW
#define VISION_UART_INDEX UART_4
#define VISION_UART_BAUDRATE 115200
#define VISION_UART_TX_PIN UART4_TX_C16
#define VISION_UART_RX_PIN UART4_RX_C17
#else
#define VISION_UART_INDEX UART_8
#define VISION_UART_BAUDRATE 115200
#define VISION_UART_TX_PIN UART8_TX_D16
#define VISION_UART_RX_PIN UART8_RX_D17
#endif

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
/* Legacy receive-only frame tag. New camera position frames are Cx,y lines. */
#define CAR_FRAME_HEADER_TAG "$CAR"
#define FRAME_TAIL_TAG "$END"

/* Map/position camera line commands (CRLF terminated). */
#define UART_CMD_MAP "START\r\n"
#define UART_CMD_CAR "CARPOS\r\n"
#define UART_CMD_CAR_STREAM_START "CARINIT\r\n"
#define UART_CMD_CAR_STREAM_STOP "CARSTOP\r\n"

/*
 * 右侧识别摄像头协议（main(视觉).py）：
 * - 主控发送 SNAP=<mode>,<seq>\n；mode=0/1/2/3 分别是近图像、近数字、远图像、远数字；
 * - 摄像头对每一条请求均回传 B=<seq>,<label>,<score>\r\n（图像）
 *   或 T=<seq>,<label>,<score>\r\n（数字），score 范围为 0~100；
 * - 低置信度也会回包（通常 label=-1），由主控决定是否重试，避免静默等待。
 */
#define VISION_SNAP_MODE_IMG_ONE_GRID 0U
#define VISION_SNAP_MODE_NUM_ONE_GRID 1U
#define VISION_SNAP_MODE_IMG_TWO_GRID 2U
#define VISION_SNAP_MODE_NUM_TWO_GRID 3U

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

typedef enum {
    VISION_RECOGNITION_DISTANCE_NONE = 0U,
    VISION_RECOGNITION_DISTANCE_TWO_GRID = 1U,
    VISION_RECOGNITION_DISTANCE_ONE_GRID = 2U
} VisionRecognitionDistance;

typedef struct {
    VisionRecognitionType type;       /* 本条结果来自数字模型还是图案模型。 */
    uint16 request_seq;                /* 与 SNAP 请求对应的序号，用于丢弃迟到回包。 */
    char label[VISION_LABEL_MAX_LEN]; /* 原始标签字符串，例如 "3"、"box"、"-1"。 */
    int32 label_value;                /* 当 label 是纯数字时保存数值；否则为 -1。 */
    bool label_is_number;             /* label_value 是否由有效数字转换得到。 */
    int16 score;                      /* 摄像头返回的置信度百分比，范围 0~100。 */
    bool success;                     /* true=收到格式正确的 B=/T= 回包（低置信度也为 true）。 */
    bool mode_marker;                 /* 新摄像头未使用，固定为 false。 */
} VisionRecognitionResult;

/* 初始化串口接收、FIFO 与模块状态 */
void uart_blob_init(void);
/* 主循环调用：读取 FIFO 并尝试解析完整帧 */
void process_blob_data(void);
/* Discard pending map/position UART bytes and any incomplete frame. */
void uart_blob_clear_pending_data(void);
/* 主循环调用：读取视觉识别 UART FIFO，并按行解析 B=/T= 序号、标签、置信度结果。 */
void process_vision_data(void);
/* 直接从多行字符串解析地图（不经过串口帧） */
bool parse_map_from_string(const char *map_text);

/* 串口接收中断回调（由 LPUART1_IRQHandler 调用） */
void uart_blob_rx_interrupt_handler(void);
/* 视觉识别串口接收中断回调（默认由 LPUART4_IRQHandler 调用）。 */
void vision_uart_rx_interrupt_handler(void);
/* 发送 "MAP" 请求命令 */
void uart_send_map_request(void);
/* Request one C<x100>,<y100> position line with CARPOS. */
void uart_send_car_request(void);
/* Optional continuous position reporting controls. */
void uart_start_car_stream(void);
void uart_stop_car_stream(void);
/* 按“识别类型 + 识别距离”发送 SNAP=<mode>,<seq> 命令。 */
bool uart_send_vision_request(VisionRecognitionType type, VisionRecognitionDistance distance);
/* 同上，并返回本次请求序号，自动识别流程据此过滤迟到的旧回包。 */
bool uart_send_vision_request_with_sequence(VisionRecognitionType type,
                                            VisionRecognitionDistance distance,
                                            uint16 *out_sequence);
/* 向视觉摄像头发送数字识别请求：distance=1 两格，distance=2 一格。 */
bool uart_send_vision_num_request_by_distance(VisionRecognitionDistance distance);
/* 向视觉摄像头发送图像识别请求：distance=1 两格，distance=2 一格。 */
bool uart_send_vision_img_request_by_distance(VisionRecognitionDistance distance);
/* 兼容调用：默认使用近距离数字识别。 */
void uart_send_vision_num_request(void);
/* 兼容调用：默认使用近距离图像识别。 */
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
