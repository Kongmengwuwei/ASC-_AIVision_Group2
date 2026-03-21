#include "Camera_handler.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

#define MAP_FRAME_HEADER_TAG "$MAP"
#define MAP_FRAME_TAIL_TAG "$END"
#define MAP_FRAME_HEADER_TAG_LEN 4U
#define MAP_FRAME_TAIL_TAG_LEN 4U
#define DYNAMIC_LINE_MAX_LEN 48U

uint32 format_count = 0;
uint8_t uart_fifo_buf[FIFO_SIZE];
fifo_struct uart_data_fifo;
static uint8_t fifo_read_buf[FIFO_READ_BUF_SIZE] = {0};

PlannerPointV3_BFS obstacles[MAX_OBSTACLES] = {{0}};
PlannerPointV3_BFS boxes[MAX_BOXES] = {{0}};
PlannerPointV3_BFS targets[MAX_TARGETS] = {{0}};
PlannerPointV3_BFS map_bombs[MAX_BOMBS] = {{0}};
PlannerPointV3_BFS cat_turth_path[MAX_CAR_PATH] = {{0}};
PlannerPointV3_BFS car = {1, 2};
CarPosition car_position = {0.0f, 0.0f};
CarPosition car_position_m = {0.0f, 0.0f};
volatile bool car_position_valid = false;
uint8_t image_find_flag = 0;

size_t actual_obstacles_count = 0;
size_t actual_boxes_count = 0;
size_t actual_targets_count = 0;
size_t actual_bombs_count = 0;
size_t actual_car_path_count = 0;

bool data_reception_complete = false;
BlobInfo blob_info = {0};

map_state_t map_state = MAP_STATE_INIT;
bool initial_map_ready = false;
bool uart_data_processing_enabled = false;
int32 dynamic_map_enable = 1;

uint8_t init_map_received_count = 0;
bool current_round_complete = false;

uint8_t get_data_1 = 0;
uint8_t get_data_2 = 0;
uint8_t get_data_3 = 0;
uint8_t get_data_4 = 0;

static uint8_t stream_buf[FRAME_BUF_SIZE] = {0};
static uint16_t stream_len = 0;

static PlannerPointV3_BFS dynamic_obstacles[MAX_OBSTACLES] = {{0}};
static PlannerPointV3_BFS dynamic_boxes[MAX_BOXES] = {{0}};
static PlannerPointV3_BFS dynamic_targets[MAX_TARGETS] = {{0}};
static PlannerPointV3_BFS dynamic_bombs[MAX_BOMBS] = {{0}};
static size_t dynamic_obstacles_count = 0;
static size_t dynamic_boxes_count = 0;
static size_t dynamic_targets_count = 0;
static size_t dynamic_bombs_count = 0;
static PlannerPointV3_BFS dynamic_car = {0, 0};
static bool dynamic_car_received = false;

static int find_pattern(const uint8_t *buf, uint16_t len, const char *pattern, uint16_t pattern_len)
{
    if (buf == NULL || pattern == NULL || len < pattern_len || pattern_len == 0U)
    {
        return -1;
    }

    for (uint16_t i = 0; i <= (uint16_t)(len - pattern_len); i++)
    {
        if (0 == memcmp(buf + i, pattern, pattern_len))
        {
            return (int)i;
        }
    }
    return -1;
}

static int find_byte(const uint8_t *buf, uint16_t len, uint8_t value)
{
    if (buf == NULL)
    {
        return -1;
    }
    for (uint16_t i = 0; i < len; i++)
    {
        if (buf[i] == value)
        {
            return (int)i;
        }
    }
    return -1;
}

static uint16_t skip_line_break_prefix(const uint8_t *buf, uint16_t len, uint16_t idx)
{
    while (idx < len && (buf[idx] == '\r' || buf[idx] == '\n'))
    {
        idx++;
    }
    return idx;
}

static uint16_t trim_line_break_suffix(const uint8_t *buf, uint16_t begin, uint16_t end)
{
    while (end > begin && (buf[end - 1U] == '\r' || buf[end - 1U] == '\n'))
    {
        end--;
    }
    return end;
}

static bool consume_one_line_break(const uint8_t *buf, uint16_t len, uint16_t *idx)
{
    if (buf == NULL || idx == NULL || *idx >= len)
    {
        return false;
    }

    if (buf[*idx] == '\r')
    {
        (*idx)++;
        if (*idx < len && buf[*idx] == '\n')
        {
            (*idx)++;
        }
        return true;
    }

    if (buf[*idx] == '\n')
    {
        (*idx)++;
        return true;
    }

    return false;
}

static void consume_stream_prefix(uint16_t consume_len)
{
    if (consume_len == 0U || stream_len == 0U)
    {
        return;
    }

    if (consume_len >= stream_len)
    {
        stream_len = 0;
        return;
    }

    memmove(stream_buf, stream_buf + consume_len, stream_len - consume_len);
    stream_len = (uint16_t)(stream_len - consume_len);
}

static void remove_stream_range(uint16_t start, uint16_t remove_len)
{
    if (remove_len == 0U || start >= stream_len)
    {
        return;
    }

    if ((uint16_t)(start + remove_len) >= stream_len)
    {
        stream_len = start;
        return;
    }

    memmove(stream_buf + start, stream_buf + start + remove_len, stream_len - start - remove_len);
    stream_len = (uint16_t)(stream_len - remove_len);
}

static void append_stream_bytes(const uint8_t *data, uint32_t len)
{
    if (data == NULL || len == 0U)
    {
        return;
    }

    if (len >= FRAME_BUF_SIZE)
    {
        memcpy(stream_buf, data + len - (FRAME_BUF_SIZE - 1U), FRAME_BUF_SIZE - 1U);
        stream_len = FRAME_BUF_SIZE - 1U;
        return;
    }

    if ((uint32_t)stream_len + len >= FRAME_BUF_SIZE)
    {
        uint16_t need_drop = (uint16_t)((uint32_t)stream_len + len - (FRAME_BUF_SIZE - 1U));
        consume_stream_prefix(need_drop);
    }

    memcpy(stream_buf + stream_len, data, len);
    stream_len = (uint16_t)(stream_len + len);
}

static void reset_dynamic_round(void)
{
    memset(dynamic_obstacles, 0, sizeof(dynamic_obstacles));
    memset(dynamic_boxes, 0, sizeof(dynamic_boxes));
    memset(dynamic_targets, 0, sizeof(dynamic_targets));
    memset(dynamic_bombs, 0, sizeof(dynamic_bombs));
    dynamic_obstacles_count = 0;
    dynamic_boxes_count = 0;
    dynamic_targets_count = 0;
    dynamic_bombs_count = 0;
    dynamic_car_received = false;
}

static void commit_dynamic_round(void)
{
    if (!dynamic_car_received)
    {
        return;
    }

    car = dynamic_car;
    get_data_1 = 1;

    if (dynamic_obstacles_count > 0U)
    {
        memset(obstacles, 0, sizeof(obstacles));
        memcpy(obstacles, dynamic_obstacles, dynamic_obstacles_count * sizeof(PlannerPointV3_BFS));
        actual_obstacles_count = dynamic_obstacles_count;
        get_data_2 = 2;
    }

    if (dynamic_boxes_count > 0U)
    {
        memset(boxes, 0, sizeof(boxes));
        memcpy(boxes, dynamic_boxes, dynamic_boxes_count * sizeof(PlannerPointV3_BFS));
        actual_boxes_count = dynamic_boxes_count;
        get_data_3 = 3;
    }

    if (dynamic_targets_count > 0U)
    {
        memset(targets, 0, sizeof(targets));
        memcpy(targets, dynamic_targets, dynamic_targets_count * sizeof(PlannerPointV3_BFS));
        actual_targets_count = dynamic_targets_count;
        get_data_4 = 4;
    }

    if (dynamic_bombs_count > 0U)
    {
        memset(map_bombs, 0, sizeof(map_bombs));
        memcpy(map_bombs, dynamic_bombs, dynamic_bombs_count * sizeof(PlannerPointV3_BFS));
        actual_bombs_count = dynamic_bombs_count;
    }

    current_round_complete = true;
    format_count++;
    reset_dynamic_round();
}

void pixel_to_grid(int pixel_row, int pixel_col, float *grid_row, float *grid_col,
                   float grid_ratio_row, float grid_ratio_col)
{
    if (grid_row != NULL && grid_col != NULL)
    {
        *grid_row = (float)pixel_row / grid_ratio_row;
        *grid_col = (float)pixel_col / grid_ratio_col;
    }
}

static bool parse_map_payload(const uint8_t *payload, uint16_t payload_len)
{
    if (payload == NULL)
    {
        return false;
    }

    PlannerPointV3_BFS new_obstacles[MAX_OBSTACLES] = {{0}};
    PlannerPointV3_BFS new_boxes[MAX_BOXES] = {{0}};
    PlannerPointV3_BFS new_targets[MAX_TARGETS] = {{0}};
    PlannerPointV3_BFS new_bombs[MAX_BOMBS] = {{0}};
    PlannerPointV3_BFS new_car = {0, 0};
    size_t new_obstacles_count = 0;
    size_t new_boxes_count = 0;
    size_t new_targets_count = 0;
    size_t new_bombs_count = 0;
    bool car_found = false;
    uint16_t idx = 0U;

    for (size_t row = 0; row < MAP_ROWS; row++)
    {
        while (idx < payload_len && (payload[idx] == ' ' || payload[idx] == '\t'))
        {
            idx++;
        }

        if ((uint32_t)idx + MAP_COLS > payload_len)
        {
            return false;
        }

        for (size_t col = 0; col < MAP_COLS; col++)
        {
            char cell = (char)payload[idx++];
            switch (cell)
            {
            case MAP_SYMBOL_OBSTACLE:
                if (new_obstacles_count >= MAX_OBSTACLES)
                {
                    return false;
                }
                new_obstacles[new_obstacles_count].row = (int)row;
                new_obstacles[new_obstacles_count].col = (int)col;
                new_obstacles_count++;
                break;

            case MAP_SYMBOL_BOX:
                if (new_boxes_count >= MAX_BOXES)
                {
                    return false;
                }
                new_boxes[new_boxes_count].row = (int)row;
                new_boxes[new_boxes_count].col = (int)col;
                new_boxes_count++;
                break;

            case MAP_SYMBOL_TARGET:
                if (new_targets_count >= MAX_TARGETS)
                {
                    return false;
                }
                new_targets[new_targets_count].row = (int)row;
                new_targets[new_targets_count].col = (int)col;
                new_targets_count++;
                break;

            case MAP_SYMBOL_BOMB:
                if (new_bombs_count >= MAX_BOMBS)
                {
                    return false;
                }
                new_bombs[new_bombs_count].row = (int)row;
                new_bombs[new_bombs_count].col = (int)col;
                new_bombs_count++;
                break;

            case MAP_SYMBOL_CAR:
                if (car_found)
                {
                    return false;
                }
                new_car.row = (int)row;
                new_car.col = (int)col;
                car_found = true;
                break;

            case MAP_SYMBOL_EMPTY:
                break;

            default:
                return false;
            }
        }

        while (idx < payload_len && (payload[idx] == ' ' || payload[idx] == '\t'))
        {
            idx++;
        }

        if (!consume_one_line_break(payload, payload_len, &idx))
        {
            return false;
        }
    }

    while (idx < payload_len && (payload[idx] == ' ' || payload[idx] == '\t'))
    {
        idx++;
    }
    idx = skip_line_break_prefix(payload, payload_len, idx);

    if (idx != payload_len || !car_found)
    {
        return false;
    }

    memset(obstacles, 0, sizeof(obstacles));
    memset(boxes, 0, sizeof(boxes));
    memset(targets, 0, sizeof(targets));
    memset(map_bombs, 0, sizeof(map_bombs));

    if (new_obstacles_count > 0U)
    {
        memcpy(obstacles, new_obstacles, new_obstacles_count * sizeof(PlannerPointV3_BFS));
    }
    if (new_boxes_count > 0U)
    {
        memcpy(boxes, new_boxes, new_boxes_count * sizeof(PlannerPointV3_BFS));
    }
    if (new_targets_count > 0U)
    {
        memcpy(targets, new_targets, new_targets_count * sizeof(PlannerPointV3_BFS));
    }
    if (new_bombs_count > 0U)
    {
        memcpy(map_bombs, new_bombs, new_bombs_count * sizeof(PlannerPointV3_BFS));
    }

    car = new_car;
    car_position.row = (float)new_car.row;
    car_position.col = (float)new_car.col;
    car_position_m.row = car_position.row * GRID_SIZE_M;
    car_position_m.col = car_position.col * GRID_SIZE_M;
    car_position_valid = true;
    image_find_flag = 0;

    actual_obstacles_count = new_obstacles_count;
    actual_boxes_count = new_boxes_count;
    actual_targets_count = new_targets_count;
    actual_bombs_count = new_bombs_count;

    data_reception_complete = true;
    initial_map_ready = true;
    map_state = (dynamic_map_enable != 0) ? MAP_STATE_DYNAMIC : MAP_STATE_INIT;
    init_map_received_count++;
    current_round_complete = true;
    format_count++;
    get_data_1 = 1;
    get_data_2 = (new_obstacles_count > 0U) ? 2 : 0;
    get_data_3 = (new_boxes_count > 0U) ? 3 : 0;
    get_data_4 = (new_targets_count > 0U) ? 4 : 0;

    return true;
}

static void parse_map_packets(void)
{
    while (stream_len >= (MAP_FRAME_HEADER_TAG_LEN + MAP_FRAME_TAIL_TAG_LEN))
    {
        int start = find_pattern(stream_buf, stream_len, MAP_FRAME_HEADER_TAG, MAP_FRAME_HEADER_TAG_LEN);
        if (start < 0)
        {
            return;
        }

        uint16_t search_from = (uint16_t)start + MAP_FRAME_HEADER_TAG_LEN;
        int end_rel = find_pattern(stream_buf + search_from, (uint16_t)(stream_len - search_from),
                                   MAP_FRAME_TAIL_TAG, MAP_FRAME_TAIL_TAG_LEN);
        if (end_rel < 0)
        {
            return;
        }

        uint16_t end_tag_pos = (uint16_t)(search_from + end_rel);
        uint16_t payload_start = skip_line_break_prefix(stream_buf, stream_len, search_from);
        uint16_t payload_end = end_tag_pos; // 保留$END前的\r\n，让payload里每行都有换行
        uint16_t packet_end = (uint16_t)(end_tag_pos + MAP_FRAME_TAIL_TAG_LEN);
        packet_end = skip_line_break_prefix(stream_buf, stream_len, packet_end);

        if (payload_end > payload_start)
        {
            parse_map_payload(stream_buf + payload_start, (uint16_t)(payload_end - payload_start));
        }

        uint16_t packet_len = (uint16_t)(packet_end - (uint16_t)start);
        remove_stream_range((uint16_t)start, packet_len);
    }
}

static void compact_init_stream_buffer(void)
{
    int start = find_pattern(stream_buf, stream_len, MAP_FRAME_HEADER_TAG, MAP_FRAME_HEADER_TAG_LEN);
    if (start > 0)
    {
        consume_stream_prefix((uint16_t)start);
        return;
    }

    if (start < 0 && stream_len > MAP_FRAME_HEADER_TAG_LEN)
    {
        uint16_t keep = MAP_FRAME_HEADER_TAG_LEN - 1U;
        memmove(stream_buf, stream_buf + stream_len - keep, keep);
        stream_len = keep;
    }
}

static void parse_dynamic_line(const char *line)
{
    char symbol = 0;
    int raw_row = 0;
    int raw_col = 0;

    if (line == NULL)
    {
        return;
    }

    if (3 != sscanf(line, "%c%d,%d", &symbol, &raw_row, &raw_col))
    {
        return;
    }

    switch (symbol)
    {
    case MAP_SYMBOL_CAR:
    {
        /* C%d,%d: raw is grid coordinate * 100. */
        float row_grid = (float)raw_row * 0.01f;
        float col_grid = (float)raw_col * 0.01f;
        int car_grid_row = (int)lroundf(row_grid);
        int car_grid_col = (int)lroundf(col_grid);

        car_position.row = row_grid;
        car_position.col = col_grid;
        car_position_m.row = row_grid * GRID_SIZE_M;
        car_position_m.col = col_grid * GRID_SIZE_M;
        image_find_flag = (car_grid_row >= MAP_ROWS || car_grid_col >= MAP_COLS || car_grid_row < 0 || car_grid_col < 0) ? 1U : 0U;
        car_position_valid = true;

        dynamic_car.row = car_grid_row;
        dynamic_car.col = car_grid_col;
        dynamic_car_received = true;
        if (dynamic_map_enable && map_state == MAP_STATE_DYNAMIC)
        {
            commit_dynamic_round();
        }
        break;
    }

    case MAP_SYMBOL_OBSTACLE:
        if (dynamic_map_enable && map_state == MAP_STATE_DYNAMIC && dynamic_obstacles_count < MAX_OBSTACLES)
        {
            dynamic_obstacles[dynamic_obstacles_count].row = raw_row;
            dynamic_obstacles[dynamic_obstacles_count].col = raw_col;
            dynamic_obstacles_count++;
        }
        break;

    case MAP_SYMBOL_BOX:
        if (dynamic_map_enable && map_state == MAP_STATE_DYNAMIC && dynamic_boxes_count < MAX_BOXES)
        {
            dynamic_boxes[dynamic_boxes_count].row = raw_row;
            dynamic_boxes[dynamic_boxes_count].col = raw_col;
            dynamic_boxes_count++;
        }
        break;

    case MAP_SYMBOL_TARGET:
        if (dynamic_map_enable && map_state == MAP_STATE_DYNAMIC && dynamic_targets_count < MAX_TARGETS)
        {
            dynamic_targets[dynamic_targets_count].row = raw_row;
            dynamic_targets[dynamic_targets_count].col = raw_col;
            dynamic_targets_count++;
        }
        break;

    case MAP_SYMBOL_BOMB:
        if (dynamic_map_enable && map_state == MAP_STATE_DYNAMIC && dynamic_bombs_count < MAX_BOMBS)
        {
            dynamic_bombs[dynamic_bombs_count].row = raw_row;
            dynamic_bombs[dynamic_bombs_count].col = raw_col;
            dynamic_bombs_count++;
        }
        break;

    default:
        break;
    }
}
static void parse_dynamic_lines(void)
{
    while (stream_len > 0U)
    {
        if (stream_len >= MAP_FRAME_HEADER_TAG_LEN &&
            0 == memcmp(stream_buf, MAP_FRAME_HEADER_TAG, MAP_FRAME_HEADER_TAG_LEN))
        {
            break;
        }

        int line_end = find_byte(stream_buf, stream_len, (uint8_t)'\n');
        if (line_end < 0)
        {
            return;
        }

        uint16_t line_len = (uint16_t)(line_end + 1);
        if (line_len >= DYNAMIC_LINE_MAX_LEN)
        {
            consume_stream_prefix(line_len);
            continue;
        }

        char line[DYNAMIC_LINE_MAX_LEN];
        memcpy(line, stream_buf, line_len);
        line[line_len] = '\0';
        parse_dynamic_line(line);
        consume_stream_prefix(line_len);
    }
}

void uart_blob_init(void)
{
    fifo_init(&uart_data_fifo, FIFO_DATA_8BIT, uart_fifo_buf, FIFO_SIZE);
    fifo_clear(&uart_data_fifo);
    uart_init(UART_INDEX, UART_BAUDRATE, UART_TX_PIN, UART_RX_PIN);
    uart_rx_interrupt(UART_INDEX, ZF_ENABLE);

    memset(obstacles, 0, sizeof(obstacles));
    memset(boxes, 0, sizeof(boxes));
    memset(targets, 0, sizeof(targets));
    memset(map_bombs, 0, sizeof(map_bombs));
    memset(cat_turth_path, 0, sizeof(cat_turth_path));
    memset(stream_buf, 0, sizeof(stream_buf));
    stream_len = 0;

    actual_obstacles_count = 0;
    actual_boxes_count = 0;
    actual_targets_count = 0;
    actual_bombs_count = 0;
    actual_car_path_count = 0;
    init_map_received_count = 0;
    current_round_complete = false;

    map_state = MAP_STATE_INIT;
    initial_map_ready = false;
    data_reception_complete = false;
    car_position_valid = false;
    image_find_flag = 0;
    format_count = 0;
    reset_dynamic_round();
}

void process_blob_data(void)
{
    if (!uart_data_processing_enabled)
    {
        return;
    }

    uint32_t read_len = fifo_used(&uart_data_fifo);
    if (read_len == 0U)
    {
        return;
    }
    if (read_len > FIFO_READ_BUF_SIZE)
    {
        read_len = FIFO_READ_BUF_SIZE;
    }

    if (FIFO_SUCCESS != fifo_read_buffer(&uart_data_fifo, fifo_read_buf, &read_len, FIFO_READ_AND_CLEAN))
    {
        return;
    }
    if (read_len == 0U)
    {
        return;
    }

    append_stream_bytes(fifo_read_buf, read_len);
    parse_map_packets();

    if (initial_map_ready)
    {
        map_state = (dynamic_map_enable != 0) ? MAP_STATE_DYNAMIC : MAP_STATE_INIT;
    }

    if (dynamic_map_enable)
    {
        parse_dynamic_lines();
    }

    if (!(dynamic_map_enable && map_state == MAP_STATE_DYNAMIC))
    {
        compact_init_stream_buffer();
    }
}
