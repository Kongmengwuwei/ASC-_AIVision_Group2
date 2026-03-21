#ifndef __CAMERA_HANDLER_H_
#define __CAMERA_HANDLER_H_

// #include "assigned_box_planner_greedy_2.h"
#include "zf_common_fifo.h"
#include "zf_common_typedef.h"
#include "zf_driver_uart.h"

#define UART_INDEX UART_1
#define UART_BAUDRATE 115200
#define UART_TX_PIN UART1_TX_B12
#define UART_RX_PIN UART1_RX_B13

#define FIFO_SIZE 1024
#define FIFO_READ_BUF_SIZE 512
#define FRAME_BUF_SIZE 512

#define MID_CAMERA_WIDTH 160
#define MID_CAMERA_HEIGHT 120
#define CAMERA_WIDTH 320
#define CAMERA_HEIGHT 240

#define MAP_ROWS 16
#define MAP_COLS 12

#define MAP_SYMBOL_OBSTACLE '#'
#define MAP_SYMBOL_EMPTY '.'
#define MAP_SYMBOL_CAR 'C'
#define MAP_SYMBOL_BOX 'B'
#define MAP_SYMBOL_TARGET 'T'
#define MAP_SYMBOL_BOMB 'D'

typedef struct
{
    int row;
    int col;
} Point;

typedef Point PlannerPointV3_BFS;

typedef struct
{
    int16_t cx, cy;
    int16_t w, h;
    int32_t area;
    uint8_t valid;
    int w_error;
    float distance;
} BlobInfo;

typedef struct
{
    float row;
    float col;
} CarPosition;

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

typedef enum
{
    MAP_STATE_INIT = 0,
    MAP_STATE_DYNAMIC
} map_state_t;

void uart_blob_init(void);
void process_blob_data(void);
void pixel_to_grid(int pixel_row, int pixel_col, float *grid_row, float *grid_col,
                   float grid_ratio_row, float grid_ratio_col);

extern map_state_t map_state;
extern bool initial_map_ready;
extern bool uart_data_processing_enabled;
extern int32 dynamic_map_enable;

extern size_t actual_obstacles_count;
extern size_t actual_boxes_count;
extern size_t actual_targets_count;
extern size_t actual_bombs_count;
extern size_t actual_car_path_count;

extern PlannerPointV3_BFS obstacles[MAX_OBSTACLES];
extern PlannerPointV3_BFS boxes[MAX_BOXES];
extern PlannerPointV3_BFS targets[MAX_TARGETS];
extern PlannerPointV3_BFS map_bombs[MAX_BOMBS];
extern PlannerPointV3_BFS cat_turth_path[MAX_CAR_PATH];
extern PlannerPointV3_BFS car;
extern CarPosition car_position;
extern CarPosition car_position_m;
extern volatile bool car_position_valid;

extern uint8_t init_map_received_count;
extern bool current_round_complete;
extern bool data_reception_complete;

extern uint8_t get_data_1;
extern uint8_t get_data_2;
extern uint8_t get_data_3;
extern uint8_t get_data_4;

extern fifo_struct uart_data_fifo;
extern BlobInfo blob_info;
extern uint32 format_count;
extern uint8_t image_find_flag;

#endif
