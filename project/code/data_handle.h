#ifndef __DATA_HANDLE_H_
#define __DATA_HANDLE_H_

#include "zf_common_fifo.h"
#include "zf_common_typedef.h"
#include "zf_driver_uart.h"

#define UART_INDEX              UART_1
#define UART_BAUDRATE           115200
#define UART_TX_PIN             UART1_TX_B12
#define UART_RX_PIN             UART1_RX_B13

#define FIFO_SIZE               512
#define FIFO_READ_BUF_SIZE      128
#define FRAME_BUF_SIZE          256

#define MAP_ROWS                14
#define MAP_COLS                10

#define MAP_FRAME_HEADER_TAG    "$MAP"
#define CAR_FRAME_HEADER_TAG    "$CAR"
#define FRAME_TAIL_TAG          "$END"

#define UART_CMD_MAP            "MAP"
#define UART_CMD_CAR            "CAR"

#define MAP_SYMBOL_OBSTACLE     '#'
#define MAP_SYMBOL_EMPTY        '.'
#define MAP_SYMBOL_CAR          'C'
#define MAP_SYMBOL_BOX          'B'
#define MAP_SYMBOL_TARGET       'T'
#define MAP_SYMBOL_BOMB         'D'



typedef struct {
    int32 x_raw;
    int32 y_raw;
    int32 yaw_raw;
    float x;
    float y;
    float yaw;
} CarPose;

void uart_blob_init(void);
void process_blob_data(void);

void uart_blob_rx_interrupt_handler(void);
void uart_send_map_request(void);
void uart_send_car_request(void);

extern bool uart_data_processing_enabled;

extern char map_data[MAP_ROWS][MAP_COLS];
extern bool map_data_ready;
extern bool map_data_updated;

extern CarPose car_pose;
extern bool car_pose_ready;
extern bool car_pose_updated;

extern uint8_t map_frame_count;
extern uint8_t car_frame_count;
extern bool uart_rx_overflow_flag;

#endif
