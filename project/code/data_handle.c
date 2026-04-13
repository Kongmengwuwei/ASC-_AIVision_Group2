#include "data_handle.h"
#include <stdlib.h>
#include <string.h>

/*
 * 摄像头串口数据处理模块
 * 协议约定：
 * 1) 地图帧：$MAP + 多行地图文本 + $END
 * 2) 车姿帧：$CAR + 三行整数(x_raw/y_raw/yaw_raw) + $END
 *
 * 模块分层：
 * - 串口中断：逐字节写入 FIFO（尽量短小，避免在中断里做重解析）
 * - 主循环：批量从 FIFO 取数据并拼接到 stream_buf
 * - 包解析：在 stream_buf 中查找完整帧并更新 map_data/car_pose
 */
#define MAP_FRAME_HEADER_TAG_LEN   4U
#define CAR_FRAME_HEADER_TAG_LEN   4U
#define FRAME_TAIL_TAG_LEN         4U
#define CAR_LINE_COUNT             3U

typedef enum {
    FRAME_TYPE_NONE = 0,
    FRAME_TYPE_MAP,
    FRAME_TYPE_CAR
} frame_type_t;

static uint8_t uart_fifo_buf[FIFO_SIZE];
static fifo_struct uart_data_fifo;
static uint8_t fifo_read_buf[FIFO_READ_BUF_SIZE] = {0};
static uint8_t stream_buf[FRAME_BUF_SIZE] = {0};
static uint16_t stream_len = 0U;

/* 串口数据总开关：false 时 process_blob_data 直接返回 */
bool uart_data_processing_enabled = false;

/* 地图结果与状态位 */
char map_data[MAP_ROWS][MAP_COLS] = {{0}};
bool map_data_ready = false;
bool map_data_updated = false;

/* 小车位姿结果与状态位 */
CarPose car_pose = {0};
bool car_pose_ready = false;
bool car_pose_updated = false;

/* 统计计数与异常标志 */
uint8_t map_frame_count = 0U;
uint8_t car_frame_count = 0U;
bool uart_rx_overflow_flag = false;

/* 在字节缓冲区内查找 pattern，返回首下标，失败返回 -1 */
static int find_pattern(const uint8_t *buf, uint16_t len, const char *pattern, uint16_t pattern_len)
{
    uint16_t i = 0U;

    if (buf == NULL || pattern == NULL || pattern_len == 0U || len < pattern_len) {
        return -1;
    }

    for (i = 0U; i <= (uint16_t)(len - pattern_len); i++) {
        if (0 == memcmp(buf + i, pattern, pattern_len)) {
            return (int)i;
        }
    }

    return -1;
}

/* 跳过前导换行（兼容 \r/\n/\r\n） */
static uint16_t skip_line_break_prefix(const uint8_t *buf, uint16_t len, uint16_t idx)
{
    while (idx < len && ('\r' == buf[idx] || '\n' == buf[idx])) {
        idx++;
    }
    return idx;
}

/* 去掉尾部换行，返回裁剪后的 end 下标 */
static uint16_t trim_line_break_suffix(const uint8_t *buf, uint16_t begin, uint16_t end)
{
    while (end > begin && ('\r' == buf[end - 1U] || '\n' == buf[end - 1U])) {
        end--;
    }
    return end;
}

/* 消费一个换行符，兼容 \n 或 \r\n */
static bool consume_one_line_break(const uint8_t *buf, uint16_t len, uint16_t *idx)
{
    if (buf == NULL || idx == NULL || *idx >= len) {
        return false;
    }

    if ('\r' == buf[*idx]) {
        (*idx)++;
        if (*idx < len && '\n' == buf[*idx]) {
            (*idx)++;
        }
        return true;
    }

    if ('\n' == buf[*idx]) {
        (*idx)++;
        return true;
    }

    return false;
}

/* 从 stream_buf 头部丢弃已消费字节 */
static void consume_stream_prefix(uint16_t consume_len)
{
    if (0U == consume_len || 0U == stream_len) {
        return;
    }

    if (consume_len >= stream_len) {
        stream_len = 0U;
        return;
    }

    memmove(stream_buf, stream_buf + consume_len, stream_len - consume_len);
    stream_len = (uint16_t)(stream_len - consume_len);
}

/*
 * 将新数据追加到 stream_buf。
 * 若容量不足，优先丢弃最旧数据，保证保留“最近一段”字节流。
 */
static void append_stream_bytes(const uint8_t *data, uint32_t len)
{
    uint16_t need_drop = 0U;

    if (data == NULL || 0U == len) {
        return;
    }

    if (len >= FRAME_BUF_SIZE) {
        memcpy(stream_buf, data + len - (FRAME_BUF_SIZE - 1U), FRAME_BUF_SIZE - 1U);
        stream_len = FRAME_BUF_SIZE - 1U;
        return;
    }

    if ((uint32_t)stream_len + len >= FRAME_BUF_SIZE) {
        need_drop = (uint16_t)((uint32_t)stream_len + len - (FRAME_BUF_SIZE - 1U));
        consume_stream_prefix(need_drop);
    }

    memcpy(stream_buf + stream_len, data, len);
    stream_len = (uint16_t)(stream_len + len);
}

/*
 * 当无法找到完整帧时，压缩流缓冲区：
 * - 若能找到帧头，丢弃帧头之前的噪声
 * - 若完全找不到帧头，仅保留“可能组成帧头前缀”的末尾字节
 */
static void compact_stream_buffer(void)
{
    int map_start = find_pattern(stream_buf, stream_len, MAP_FRAME_HEADER_TAG, MAP_FRAME_HEADER_TAG_LEN);
    int car_start = find_pattern(stream_buf, stream_len, CAR_FRAME_HEADER_TAG, CAR_FRAME_HEADER_TAG_LEN);
    int start = -1;
    uint16_t keep = 0U;

    if (map_start >= 0 && car_start >= 0) {
        start = (map_start < car_start) ? map_start : car_start;
    } else if (map_start >= 0) {
        start = map_start;
    } else {
        start = car_start;
    }

    if (start > 0) {
        consume_stream_prefix((uint16_t)start);
        return;
    }

    if (start < 0 && stream_len > (MAP_FRAME_HEADER_TAG_LEN - 1U)) {
        keep = MAP_FRAME_HEADER_TAG_LEN - 1U;
        memmove(stream_buf, stream_buf + stream_len - keep, keep);
        stream_len = keep;
    }
}

/*
 * 解析地图 payload：
 * - 必须严格包含 MAP_ROWS 行
 * - 每行长度必须为 MAP_COLS
 * - 成功后整体提交到 map_data，并置位 updated 标志
 */
static bool parse_map_payload(const uint8_t *payload, uint16_t payload_len)
{
    char new_map[MAP_ROWS][MAP_COLS] = {{0}};
    Position new_obstacles[MAX_OBSTACLES] = {{0}};
    Position new_boxes[MAX_BOXES] = {{0}};
    Position new_targets[MAX_TARGETS] = {{0}};
    Position new_bombs[MAX_BOMBS] = {{0}};
    Position new_car = {0};
    size_t new_obstacles_count = 0U;
    size_t new_boxes_count = 0U;
    size_t new_targets_count = 0U;
    size_t new_bombs_count = 0U;
    bool car_found = false;
    uint16_t idx = 0U;
    size_t row = 0U;
    size_t col = 0U;
    char cell = 0;

    if (payload == NULL) {
        return false;
    }

    idx = skip_line_break_prefix(payload, payload_len, idx);

    for (row = 0U; row < MAP_ROWS; row++) {
        /* 与旧版一致：允许每行前后带空格/Tab */
        while (idx < payload_len && (' ' == payload[idx] || '\t' == payload[idx])) {
            idx++;
        }

        if (((uint32_t)idx + MAP_COLS) > payload_len) {
            return false;
        }

        for (col = 0U; col < MAP_COLS; col++) {
            cell = (char)payload[idx++];
            switch (cell) {
                case MAP_SYMBOL_OBSTACLE:
                    if (new_obstacles_count >= MAX_OBSTACLES) {
                        return false;
                    }
                    new_obstacles[new_obstacles_count].row = (uint8)row;
                    new_obstacles[new_obstacles_count].col = (uint8)col;
                    new_obstacles[new_obstacles_count].id = 0U;
                    new_obstacles_count++;
                    break;

                case MAP_SYMBOL_BOX:
                    if (new_boxes_count >= MAX_BOXES) {
                        return false;
                    }
                    new_boxes[new_boxes_count].row = (uint8)row;
                    new_boxes[new_boxes_count].col = (uint8)col;
                    new_boxes[new_boxes_count].id = 0U;
                    new_boxes_count++;
                    break;

                case MAP_SYMBOL_TARGET:
                    if (new_targets_count >= MAX_TARGETS) {
                        return false;
                    }
                    new_targets[new_targets_count].row = (uint8)row;
                    new_targets[new_targets_count].col = (uint8)col;
                    new_targets[new_targets_count].id = 0U;
                    new_targets_count++;
                    break;

                case MAP_SYMBOL_BOMB:
                    if (new_bombs_count >= MAX_BOMBS) {
                        return false;
                    }
                    new_bombs[new_bombs_count].row = (uint8)row;
                    new_bombs[new_bombs_count].col = (uint8)col;
                    new_bombs[new_bombs_count].id = 0U;
                    new_bombs_count++;
                    break;

                case MAP_SYMBOL_CAR:
                    if (car_found) {
                        return false;
                    }
                    new_car.row = (uint8)row;
                    new_car.col = (uint8)col;
                    new_car.id = 0U;
                    car_found = true;
                    break;

                case MAP_SYMBOL_EMPTY:
                    break;

                default:
                    return false;
            }

            new_map[row][col] = cell;
        }

        while (idx < payload_len && (' ' == payload[idx] || '\t' == payload[idx])) {
            idx++;
        }

        if (row < (MAP_ROWS - 1U)) {
            if (!consume_one_line_break(payload, payload_len, &idx)) {
                return false;
            }
        } else {
            /* 最后一行换行可有可无 */
            consume_one_line_break(payload, payload_len, &idx);
        }
    }

    while (idx < payload_len && (' ' == payload[idx] || '\t' == payload[idx])) {
        idx++;
    }
    idx = skip_line_break_prefix(payload, payload_len, idx);
    if (idx != payload_len || !car_found) {
        return false;
    }

    memset(obstacles, 0, sizeof(obstacles));
    memset(boxes, 0, sizeof(boxes));
    memset(targets, 0, sizeof(targets));
    memset(bombs, 0, sizeof(bombs));

    if (new_obstacles_count > 0U) {
        memcpy(obstacles, new_obstacles, new_obstacles_count * sizeof(Position));
    }
    if (new_boxes_count > 0U) {
        memcpy(boxes, new_boxes, new_boxes_count * sizeof(Position));
    }
    if (new_targets_count > 0U) {
        memcpy(targets, new_targets, new_targets_count * sizeof(Position));
    }
    if (new_bombs_count > 0U) {
        memcpy(bombs, new_bombs, new_bombs_count * sizeof(Position));
    }
    Obstacles_count = new_obstacles_count;
    Boxes_count = new_boxes_count;
    Targets_count = new_targets_count;
    Bombs_count = new_bombs_count;
    car = new_car;

    memcpy(map_data, new_map, sizeof(map_data));
    map_data_ready = true;
    map_data_updated = true;
    map_frame_count++;
    return true;
}

/*
 * 从直接输入的多行字符串解析地图（等价于 parse_map_payload 语义）：
 * - 支持最后一行不带换行符（会自动补 '\n'）
 * - 解析成功后同样更新 map_data/map_data_ready/map_data_updated
 */
bool parse_map_from_string(const char *map_text)
{
    size_t payload_len = 0U;
    uint8_t temp_payload[FRAME_BUF_SIZE] = {0};

    if (map_text == NULL) {
        return false;
    }

    payload_len = strlen(map_text);
    if (0U == payload_len || payload_len > 0xFFFFU) {
        return false;
    }

    /* 兼容直接输入字符串最后一行没有换行符的情况 */
    if ('\n' != map_text[payload_len - 1U] && '\r' != map_text[payload_len - 1U]) {
        if ((payload_len + 1U) > FRAME_BUF_SIZE) {
            return false;
        }

        memcpy(temp_payload, map_text, payload_len);
        temp_payload[payload_len] = '\n';
        return parse_map_payload(temp_payload, (uint16_t)(payload_len + 1U));
    }

    return parse_map_payload((const uint8_t *)map_text, (uint16_t)payload_len);
}

/* 解析 payload 中的一行 int32（十进制字符串） */
static bool parse_int32_line(const uint8_t *payload, uint16_t payload_len, uint16_t *idx, int32 *value)
{
    char line_buf[24];
    uint16_t line_begin = 0U;
    uint16_t line_end = 0U;
    uint16_t line_len = 0U;
    char *end_ptr = NULL;
    long temp = 0;

    if (payload == NULL || idx == NULL || value == NULL) {
        return false;
    }

    *idx = skip_line_break_prefix(payload, payload_len, *idx);
    line_begin = *idx;

    while (*idx < payload_len && '\r' != payload[*idx] && '\n' != payload[*idx]) {
        (*idx)++;
    }

    line_end = trim_line_break_suffix(payload, line_begin, *idx);
    line_len = (uint16_t)(line_end - line_begin);
    if (0U == line_len || line_len >= sizeof(line_buf)) {
        return false;
    }

    memcpy(line_buf, payload + line_begin, line_len);
    line_buf[line_len] = '\0';

    temp = strtol(line_buf, &end_ptr, 10);
    if ('\0' != *end_ptr) {
        return false;
    }

    *value = (int32)temp;

    if (*idx < payload_len) {
        consume_one_line_break(payload, payload_len, idx);
    }

    return true;
}

/*
 * 解析车姿 payload（固定三行）：
 * x_raw, y_raw, yaw_raw，单位约定为 0.01
 * 同时给出 raw 整数和转换后的浮点值
 */
static bool parse_car_payload(const uint8_t *payload, uint16_t payload_len)
{
    int32 raw_value[CAR_LINE_COUNT] = {0};
    uint16_t idx = 0U;
    uint16_t value_index = 0U;

    if (payload == NULL) {
        return false;
    }

    for (value_index = 0U; value_index < CAR_LINE_COUNT; value_index++) {
        if (!parse_int32_line(payload, payload_len, &idx, &raw_value[value_index])) {
            return false;
        }
    }

    idx = skip_line_break_prefix(payload, payload_len, idx);
    if (idx != payload_len) {
        return false;
    }

    car_pose.x_raw = raw_value[0];
    car_pose.y_raw = raw_value[1];
    car_pose.yaw_raw = raw_value[2];
    car_pose.x = (float)raw_value[0] * 0.01f;
    car_pose.y = (float)raw_value[1] * 0.01f;
    car_pose.yaw = (float)raw_value[2] * 0.01f;

    car_pose_ready = true;
    car_pose_updated = true;
    car_frame_count++;
    return true;
}

/* 在当前 stream_buf 中找“最靠前”的帧头，返回帧类型、起始下标和帧头长度 */
static frame_type_t find_first_frame(uint16_t *start_index, uint16_t *header_len)
{
    int map_start = find_pattern(stream_buf, stream_len, MAP_FRAME_HEADER_TAG, MAP_FRAME_HEADER_TAG_LEN);
    int car_start = find_pattern(stream_buf, stream_len, CAR_FRAME_HEADER_TAG, CAR_FRAME_HEADER_TAG_LEN);

    if (start_index == NULL || header_len == NULL) {
        return FRAME_TYPE_NONE;
    }

    if (map_start < 0 && car_start < 0) {
        return FRAME_TYPE_NONE;
    }

    if (map_start >= 0 && (car_start < 0 || map_start <= car_start)) {
        *start_index = (uint16_t)map_start;
        *header_len = MAP_FRAME_HEADER_TAG_LEN;
        return FRAME_TYPE_MAP;
    }

    *start_index = (uint16_t)car_start;
    *header_len = CAR_FRAME_HEADER_TAG_LEN;
    return FRAME_TYPE_CAR;
}

/*
 * 包解析主循环：
 * 1) 找到最靠前帧头（$MAP/$CAR）
 * 2) 再找对应的 $END
 * 3) 抽取 payload 并分发到 map/car 解析函数
 * 4) 消费已处理字节，继续解析下一帧
 */
static void parse_packets(void)
{
    frame_type_t frame_type = FRAME_TYPE_NONE;
    uint16_t start_index = 0U;
    uint16_t header_len = 0U;
    uint16_t search_from = 0U;
    uint16_t end_tag_pos = 0U;
    uint16_t payload_start = 0U;
    uint16_t payload_end = 0U;
    uint16_t packet_end = 0U;
    int end_rel = 0;

    while (stream_len >= (MAP_FRAME_HEADER_TAG_LEN + FRAME_TAIL_TAG_LEN)) {
        frame_type = find_first_frame(&start_index, &header_len);
        if (FRAME_TYPE_NONE == frame_type) {
            compact_stream_buffer();
            return;
        }

        if (start_index > 0U) {
            consume_stream_prefix(start_index);
            continue;
        }

        search_from = header_len;
        end_rel = find_pattern(stream_buf + search_from,
                               (uint16_t)(stream_len - search_from),
                               FRAME_TAIL_TAG,
                               FRAME_TAIL_TAG_LEN);
        if (end_rel < 0) {
            return;
        }

        end_tag_pos = (uint16_t)(search_from + end_rel);
        payload_start = skip_line_break_prefix(stream_buf, stream_len, search_from);
        payload_end = trim_line_break_suffix(stream_buf, payload_start, end_tag_pos);

        if (FRAME_TYPE_MAP == frame_type) {
            parse_map_payload(stream_buf + payload_start, (uint16_t)(payload_end - payload_start));
        } else if (FRAME_TYPE_CAR == frame_type) {
            parse_car_payload(stream_buf + payload_start, (uint16_t)(payload_end - payload_start));
        }

        packet_end = (uint16_t)(end_tag_pos + FRAME_TAIL_TAG_LEN);
        packet_end = skip_line_break_prefix(stream_buf, stream_len, packet_end);
        consume_stream_prefix(packet_end);
    }
}

/* 初始化串口/FIFO/状态变量，开启串口接收中断 */
void uart_blob_init(void)
{
    fifo_init(&uart_data_fifo, FIFO_DATA_8BIT, uart_fifo_buf, FIFO_SIZE);
    fifo_clear(&uart_data_fifo);
    uart_init(UART_INDEX, UART_BAUDRATE, UART_TX_PIN, UART_RX_PIN);
    uart_rx_interrupt(UART_INDEX, ZF_ENABLE);

    memset(map_data, 0, sizeof(map_data));
    memset(&car_pose, 0, sizeof(car_pose));
    memset(stream_buf, 0, sizeof(stream_buf));
    memset(fifo_read_buf, 0, sizeof(fifo_read_buf));

    stream_len = 0U;
    map_data_ready = false;
    map_data_updated = false;
    car_pose_ready = false;
    car_pose_updated = false;
    map_frame_count = 0U;
    car_frame_count = 0U;
    uart_rx_overflow_flag = false;
    uart_data_processing_enabled = true;
}

/*
 * 主循环调用入口：
 * - 批量读 FIFO
 * - 追加到 stream_buf
 * - 触发协议解析
 */
void process_blob_data(void)
{
    uint32_t read_len = 0U;

    if (!uart_data_processing_enabled) {
        return;
    }

    read_len = fifo_used(&uart_data_fifo);
    if (0U == read_len) {
        return;
    }

    if (read_len > FIFO_READ_BUF_SIZE) {
        read_len = FIFO_READ_BUF_SIZE;
    }

    if (FIFO_SUCCESS != fifo_read_buffer(&uart_data_fifo, fifo_read_buf, &read_len, FIFO_READ_AND_CLEAN)) {
        return;
    }

    if (0U == read_len) {
        return;
    }

    append_stream_bytes(fifo_read_buf, read_len);
    parse_packets();
}

/* 串口接收中断处理：只做“取 1 字节 + 写 FIFO”，失败则置溢出标志 */
void uart_blob_rx_interrupt_handler(void)
{
    uint8_t data = 0U;

    uart_query_byte(UART_INDEX, &data);
    if (FIFO_SUCCESS != fifo_write_buffer(&uart_data_fifo, &data, 1U)) {
        uart_rx_overflow_flag = true;
    }
}

/* 发送地图请求命令（"MAP"） */
void uart_send_map_request(void)
{
    uart_write_string(UART_INDEX, UART_CMD_MAP);
}

/* 发送车姿请求命令（"CAR"） */
void uart_send_car_request(void)
{
    uart_write_string(UART_INDEX, UART_CMD_CAR);
}
