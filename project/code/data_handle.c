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

/*
 * ===================== 图案/数字识别摄像头串口模块 =====================
 *
 * 这一组变量只服务 loadmode.py 的 NUM/IMG 行协议，和上面的地图/车姿
 * $MAP/$CAR/$END 流式协议完全分开：
 * - 使用独立 UART（默认 UART4）；
 * - 使用独立 FIFO；
 * - 使用独立行缓冲；
 * - 使用独立结果结构和 updated 标志。
 *
 * 这样即使视觉识别端返回 "NUM,3,96\n" 这种短行，也不会被地图解析器当成
 * 噪声塞进 stream_buf；反过来地图大包也不会污染视觉识别结果。
 */
static uint8_t vision_uart_fifo_buf[VISION_FIFO_SIZE];
static fifo_struct vision_uart_data_fifo;
static uint8_t vision_fifo_read_buf[VISION_FIFO_READ_BUF_SIZE] = {0};
static char vision_line_buf[VISION_LINE_BUF_SIZE] = {0};
static uint16_t vision_line_len = 0U;
static bool vision_drop_until_line_end = false;

bool vision_data_processing_enabled = false;

VisionRecognitionResult vision_num_result = {0};
bool vision_num_result_ready = false;
bool vision_num_result_updated = false;

VisionRecognitionResult vision_img_result = {0};
bool vision_img_result_ready = false;
bool vision_img_result_updated = false;

uint8_t vision_num_frame_count = 0U;
uint8_t vision_img_frame_count = 0U;
uint8_t vision_bad_line_count = 0U;
bool vision_uart_rx_overflow_flag = false;
bool vision_line_overflow_flag = false;
bool vision_parse_error_flag = false;

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

/*
 * Parse map text using a given grid shape without semantic validation.
 */
static bool parse_map_grid_with_shape(const uint8_t *payload,
                                      uint16_t payload_len,
                                      size_t expected_rows,
                                      size_t expected_cols,
                                      char *out_cells)
{
    uint16_t idx = 0U;
    size_t row = 0U;
    size_t col = 0U;

    if (payload == NULL || out_cells == NULL || 0U == expected_rows || 0U == expected_cols) {
        return false;
    }

    idx = skip_line_break_prefix(payload, payload_len, idx);

    for (row = 0U; row < expected_rows; row++) {
        while (idx < payload_len && (' ' == payload[idx] || '\t' == payload[idx])) {
            idx++;
        }

        if (((uint32_t)idx + expected_cols) > payload_len) {
            return false;
        }

        for (col = 0U; col < expected_cols; col++) {
            out_cells[row * expected_cols + col] = (char)payload[idx++];
        }

        while (idx < payload_len && (' ' == payload[idx] || '\t' == payload[idx])) {
            idx++;
        }

        if (row < (expected_rows - 1U)) {
            if (!consume_one_line_break(payload, payload_len, &idx)) {
                return false;
            }
        } else {
            consume_one_line_break(payload, payload_len, &idx);
        }
    }

    while (idx < payload_len && (' ' == payload[idx] || '\t' == payload[idx])) {
        idx++;
    }
    idx = skip_line_break_prefix(payload, payload_len, idx);
    return (idx == payload_len);
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
    char parsed_cells[MAP_ROWS * MAP_COLS] = {0};
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
    bool input_transposed = false;
    size_t row = 0U;
    size_t col = 0U;
    char cell = 0;

    if (payload == NULL) {
        return false;
    }

    if (!parse_map_grid_with_shape(payload, payload_len, MAP_ROWS, MAP_COLS, parsed_cells)) {
        if (!parse_map_grid_with_shape(payload, payload_len, MAP_COLS, MAP_ROWS, parsed_cells)) {
            return false;
        }
        input_transposed = true;
    }

    for (row = 0U; row < MAP_ROWS; row++) {
        for (col = 0U; col < MAP_COLS; col++) {
            if (!input_transposed) {
                cell = parsed_cells[row * MAP_COLS + col];
            } else {
                /* Input is 14x10, transpose back to canonical 10x14. */
                cell = parsed_cells[col * MAP_ROWS + row];
            }

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
    }

    if (!car_found) {
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
    int32 row_rounded = 0;
    int32 col_rounded = 0;

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

    /* 将车姿浮点坐标(y/x)按四舍五入更新到栅格坐标(row/col) */
    row_rounded = (car_pose.y >= 0.0f) ? (int32)(car_pose.y + 0.5f) : (int32)(car_pose.y - 0.5f);
    col_rounded = (car_pose.x >= 0.0f) ? (int32)(car_pose.x + 0.5f) : (int32)(car_pose.x - 0.5f);

    if (row_rounded < 0) {
        row_rounded = 0;
    } else if (row_rounded > 255) {
        row_rounded = 255;
    }

    if (col_rounded < 0) {
        col_rounded = 0;
    } else if (col_rounded > 255) {
        col_rounded = 255;
    }

    car.row = (uint8)row_rounded;
    car.col = (uint8)col_rounded;

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

/* ASCII 空白判断：视觉协议只允许英文逗号和数字/标签，按 ASCII 处理最稳妥。 */
static bool vision_is_ascii_space(char ch)
{
    return (' ' == ch || '\t' == ch || '\r' == ch || '\n' == ch);
}

/* 原地去掉字符串首尾空白，返回裁剪后的起始地址。 */
static char *vision_trim_ascii_in_place(char *text)
{
    char *end = NULL;

    if (text == NULL) {
        return NULL;
    }

    while ('\0' != *text && vision_is_ascii_space(*text)) {
        text++;
    }

    if ('\0' == *text) {
        return text;
    }

    end = text + strlen(text) - 1U;
    while (end > text && vision_is_ascii_space(*end)) {
        *end = '\0';
        end--;
    }

    return text;
}

/* 将小写英文字母转成大写，避免 OpenMV 端调试时误发小写命令导致解析失败。 */
static char vision_ascii_upper(char ch)
{
    if (ch >= 'a' && ch <= 'z') {
        return (char)(ch - ('a' - 'A'));
    }
    return ch;
}

/* 解析视觉结果行的第一个字段：只接受 NUM 或 IMG。 */
static VisionRecognitionType vision_parse_type_field(const char *cmd)
{
    if (cmd == NULL || '\0' == cmd[0] || '\0' == cmd[1] || '\0' == cmd[2] || '\0' != cmd[3]) {
        return VISION_RECOGNITION_NONE;
    }

    if ('N' == vision_ascii_upper(cmd[0]) &&
        'U' == vision_ascii_upper(cmd[1]) &&
        'M' == vision_ascii_upper(cmd[2])) {
        return VISION_RECOGNITION_NUM;
    }

    if ('I' == vision_ascii_upper(cmd[0]) &&
        'M' == vision_ascii_upper(cmd[1]) &&
        'G' == vision_ascii_upper(cmd[2])) {
        return VISION_RECOGNITION_IMG;
    }

    return VISION_RECOGNITION_NONE;
}

/* 严格解析十进制整数：字段里除了首尾空白，不允许混入其它字符。 */
static bool vision_parse_int32_text(const char *text, int32 *value)
{
    char *end_ptr = NULL;
    long temp = 0;

    if (text == NULL || value == NULL || '\0' == *text) {
        return false;
    }

    temp = strtol(text, &end_ptr, 10);
    if (end_ptr == text) {
        return false;
    }

    while ('\0' != *end_ptr && vision_is_ascii_space(*end_ptr)) {
        end_ptr++;
    }

    if ('\0' != *end_ptr) {
        return false;
    }

    *value = (int32)temp;
    return true;
}

/* 记录一行视觉识别坏包，计数溢出不影响功能，只作为调试线索。 */
static void vision_note_bad_line(void)
{
    vision_parse_error_flag = true;
    vision_bad_line_count++;
}

/* 初始化单个视觉识别结果结构，label_value 预置为 -1 便于上层判断“非数字标签”。 */
static void vision_reset_result(VisionRecognitionResult *result, VisionRecognitionType type)
{
    if (result == NULL) {
        return;
    }

    memset(result, 0, sizeof(*result));
    result->type = type;
    result->label_value = -1;
    result->score = -1;
    result->mode_marker = false;
    result->success = false;
}

/* 提交一条已经校验过的 NUM/IMG 结果，更新对应的 ready/updated 标志。 */
static void vision_store_result(VisionRecognitionType type,
                                const char *label,
                                int32 label_value,
                                bool label_is_number,
                                int16 score,
                                bool success,
                                bool mode_marker)
{
    VisionRecognitionResult *dst = NULL;

    if (VISION_RECOGNITION_NUM == type) {
        dst = &vision_num_result;
    } else if (VISION_RECOGNITION_IMG == type) {
        dst = &vision_img_result;
    } else {
        return;
    }

    vision_reset_result(dst, type);
    if (label != NULL) {
        strncpy(dst->label, label, VISION_LABEL_MAX_LEN - 1U);
        dst->label[VISION_LABEL_MAX_LEN - 1U] = '\0';
    }
    dst->label_value = label_value;
    dst->label_is_number = label_is_number;
    dst->score = score;
    dst->success = success;
    dst->mode_marker = mode_marker;

    if (VISION_RECOGNITION_NUM == type) {
        vision_num_result_ready = true;
        vision_num_result_updated = true;
        vision_num_frame_count++;
    } else {
        vision_img_result_ready = true;
        vision_img_result_updated = true;
        vision_img_frame_count++;
    }
}

/*
 * 解析 loadmode.py 返回的一整行：
 *
 * 正常格式：
 *   NUM,标签,置信度
 *   IMG,标签,置信度
 *
 * 失败格式：
 *   NUM,-1,-1
 *   IMG,-1,-1
 *
 * 注意：这里不解析地图帧，也不查找 $MAP/$END。视觉识别是行协议，
 * 地图/车姿是包协议，两者保持独立，避免互相误判。
 */
static void vision_parse_line(char *line)
{
    char *cmd = NULL;
    char *label = NULL;
    char *score_text = NULL;
    char *comma1 = NULL;
    char *comma2 = NULL;
    VisionRecognitionType type = VISION_RECOGNITION_NONE;
    int32 score_value = 0;
    int32 label_value = -1;
    bool label_is_number = false;
    bool success = true;
    bool mode_marker = false;

    if (line == NULL) {
        return;
    }

    cmd = vision_trim_ascii_in_place(line);
    if (cmd == NULL || '\0' == *cmd) {
        return;
    }

    comma1 = strchr(cmd, ',');
    if (comma1 == NULL) {
        vision_note_bad_line();
        return;
    }
    *comma1 = '\0';

    label = comma1 + 1U;
    comma2 = strchr(label, ',');
    if (comma2 == NULL) {
        vision_note_bad_line();
        return;
    }
    *comma2 = '\0';
    score_text = comma2 + 1U;

    if (strchr(score_text, ',') != NULL) {
        vision_note_bad_line();
        return;
    }

    cmd = vision_trim_ascii_in_place(cmd);
    label = vision_trim_ascii_in_place(label);
    score_text = vision_trim_ascii_in_place(score_text);
    if (cmd == NULL || label == NULL || score_text == NULL || '\0' == *label) {
        vision_note_bad_line();
        return;
    }

    type = vision_parse_type_field(cmd);
    if (VISION_RECOGNITION_NONE == type) {
        vision_note_bad_line();
        return;
    }

    if (!vision_parse_int32_text(score_text, &score_value)) {
        vision_note_bad_line();
        return;
    }

    label_is_number = vision_parse_int32_text(label, &label_value);
    if (!label_is_number) {
        label_value = -1;
    }

    if (0 == strcmp(label, "-1")) {
        if (-1 == score_value) {
            success = false;
        } else if (0 == score_value) {
            /*
             * 新版摄像头在第一次接收到识别命令时，会先判断关卡模式色块。
             * 命中对应纯色块时返回 IMG/NUM,-1,0：这不是普通识别失败，
             * 而是告诉主控“已确认模式，但本帧没有实际图案/数字标签”。
             */
            success = true;
            mode_marker = true;
        } else {
            vision_note_bad_line();
            return;
        }
    } else if (score_value < 0 || score_value > 100) {
        vision_note_bad_line();
        return;
    }

    vision_store_result(type, label, label_value, label_is_number, (int16)score_value, success, mode_marker);
}

/*
 * 逐字节拼接视觉识别行。
 *
 * 串口中断只负责把字节放入 FIFO，真正的行拼接在主循环完成：
 * - 收到 '\n' 或 '\r' 时认为一行结束；
 * - 空行直接忽略，兼容 "\r\n"；
 * - 行太长时丢弃到下一次换行，防止异常数据撑爆缓冲区。
 */
static void vision_feed_rx_byte(uint8_t data)
{
    if ('\r' == data || '\n' == data) {
        if (vision_drop_until_line_end) {
            vision_drop_until_line_end = false;
            vision_line_len = 0U;
            return;
        }

        if (vision_line_len > 0U) {
            vision_line_buf[vision_line_len] = '\0';
            vision_parse_line(vision_line_buf);
            memset(vision_line_buf, 0, sizeof(vision_line_buf));
            vision_line_len = 0U;
        }
        return;
    }

    if (vision_drop_until_line_end || '\0' == data) {
        return;
    }

    if (vision_line_len >= (VISION_LINE_BUF_SIZE - 1U)) {
        vision_line_len = 0U;
        vision_drop_until_line_end = true;
        vision_line_overflow_flag = true;
        vision_note_bad_line();
        return;
    }

    vision_line_buf[vision_line_len++] = (char)data;
}

/* 视觉识别串口/FIFO/状态初始化。由 uart_blob_init 统一调用，外部不需要再单独初始化。 */
static void vision_uart_init_internal(void)
{
    fifo_init(&vision_uart_data_fifo, FIFO_DATA_8BIT, vision_uart_fifo_buf, VISION_FIFO_SIZE);
    fifo_clear(&vision_uart_data_fifo);
    uart_init(VISION_UART_INDEX, VISION_UART_BAUDRATE, VISION_UART_TX_PIN, VISION_UART_RX_PIN);
    uart_rx_interrupt(VISION_UART_INDEX, ZF_ENABLE);

    memset(vision_fifo_read_buf, 0, sizeof(vision_fifo_read_buf));
    memset(vision_line_buf, 0, sizeof(vision_line_buf));
    vision_line_len = 0U;
    vision_drop_until_line_end = false;

    vision_reset_result(&vision_num_result, VISION_RECOGNITION_NUM);
    vision_num_result_ready = false;
    vision_num_result_updated = false;

    vision_reset_result(&vision_img_result, VISION_RECOGNITION_IMG);
    vision_img_result_ready = false;
    vision_img_result_updated = false;

    vision_num_frame_count = 0U;
    vision_img_frame_count = 0U;
    vision_bad_line_count = 0U;
    vision_uart_rx_overflow_flag = false;
    vision_line_overflow_flag = false;
    vision_parse_error_flag = false;
    vision_data_processing_enabled = true;
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

    /* 视觉识别串口独立初始化：不复用地图/车姿 FIFO，也不复用 stream_buf。 */
    vision_uart_init_internal();
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

/*
 * 视觉识别数据处理入口：
 * - 从视觉识别专用 FIFO 取出 UART4 中断收到的字节；
 * - 按行拼接 NUM/IMG 返回结果；
 * - 解析成功后只更新 vision_num_result 或 vision_img_result；
 * - 不访问 map_data/car_pose，也不调用 parse_packets。
 */
void process_vision_data(void)
{
    uint32_t read_len = 0U;
    uint32_t i = 0U;

    if (!vision_data_processing_enabled) {
        return;
    }

    read_len = fifo_used(&vision_uart_data_fifo);
    if (0U == read_len) {
        return;
    }

    if (read_len > VISION_FIFO_READ_BUF_SIZE) {
        read_len = VISION_FIFO_READ_BUF_SIZE;
    }

    if (FIFO_SUCCESS != fifo_read_buffer(&vision_uart_data_fifo,
                                         vision_fifo_read_buf,
                                         &read_len,
                                         FIFO_READ_AND_CLEAN)) {
        return;
    }

    for (i = 0U; i < read_len; i++) {
        vision_feed_rx_byte(vision_fifo_read_buf[i]);
    }
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

/* 视觉识别串口接收中断：只收字节入视觉 FIFO，所有解析都留给 process_vision_data。 */
void vision_uart_rx_interrupt_handler(void)
{
    uint8_t data = 0U;

    if (!uart_query_byte(VISION_UART_INDEX, &data)) {
        return;
    }

    if (FIFO_SUCCESS != fifo_write_buffer(&vision_uart_data_fifo, &data, 1U)) {
        vision_uart_rx_overflow_flag = true;
    }
}

/*
 * 根据识别类型和识别距离选择主控要发送给摄像头的命令。
 *
 * 新协议里第 1 个字符表示距离：
 * - '1'：两格距离模型；
 * - '2'：一格距离模型。
 *
 * 第 2 个字符表示模型类型：
 * - 'I'：图像识别，摄像头返回 IMG,...；
 * - 'N'：数字识别，摄像头返回 NUM,...。
 */
static const char *vision_get_request_command(VisionRecognitionType type,
                                              VisionRecognitionDistance distance)
{
    if (VISION_RECOGNITION_IMG == type) {
        if (VISION_RECOGNITION_DISTANCE_TWO_GRID == distance) {
            return UART_CMD_VISION_IMG_TWO_GRID;
        }
        if (VISION_RECOGNITION_DISTANCE_ONE_GRID == distance) {
            return UART_CMD_VISION_IMG_ONE_GRID;
        }
    } else if (VISION_RECOGNITION_NUM == type) {
        if (VISION_RECOGNITION_DISTANCE_TWO_GRID == distance) {
            return UART_CMD_VISION_NUM_TWO_GRID;
        }
        if (VISION_RECOGNITION_DISTANCE_ONE_GRID == distance) {
            return UART_CMD_VISION_NUM_ONE_GRID;
        }
    }

    return NULL;
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

/* 向视觉识别端请求一次数字识别。发送前清掉半包数据，避免旧残留误触发本次结果。 */
bool uart_send_vision_request(VisionRecognitionType type, VisionRecognitionDistance distance)
{
    const char *cmd = vision_get_request_command(type, distance);

    if (cmd == NULL) {
        return false;
    }

    /*
     * 发送新请求前只清空视觉识别串口的 FIFO、行缓冲和 updated 标志。
     * 地图/车姿 UART1 使用的是另一套 FIFO 和 stream_buf，这里不会碰它们。
     */
    vision_clear_pending_data();
    uart_write_string(VISION_UART_INDEX, cmd);
    return true;
}

bool uart_send_vision_num_request_by_distance(VisionRecognitionDistance distance)
{
    return uart_send_vision_request(VISION_RECOGNITION_NUM, distance);
}

bool uart_send_vision_img_request_by_distance(VisionRecognitionDistance distance)
{
    return uart_send_vision_request(VISION_RECOGNITION_IMG, distance);
}

/* 兼容旧调用：默认使用一格距离数字识别模型，即发送 "2N\n"。 */
void uart_send_vision_num_request(void)
{
    (void)uart_send_vision_num_request_by_distance(VISION_RECOGNITION_DISTANCE_ONE_GRID);
}

/* 向视觉识别端请求一次图案识别。发送前清掉半包数据，避免旧残留误触发本次结果。 */
/* 兼容旧调用：默认使用一格距离图像识别模型，即发送 "2I\n"。 */
void uart_send_vision_img_request(void)
{
    (void)uart_send_vision_img_request_by_distance(VISION_RECOGNITION_DISTANCE_ONE_GRID);
}

/* 清空视觉识别接收缓存和“有新结果”标志，但保留最近一次结果内容供调试查看。 */
void vision_clear_pending_data(void)
{
    fifo_clear(&vision_uart_data_fifo);
    memset(vision_fifo_read_buf, 0, sizeof(vision_fifo_read_buf));
    memset(vision_line_buf, 0, sizeof(vision_line_buf));
    vision_line_len = 0U;
    vision_drop_until_line_end = false;
    vision_num_result_updated = false;
    vision_img_result_updated = false;
}

/* 仅读取最近一次数字识别响应；没有收到过响应时返回 false。 */
bool vision_get_latest_num_result(VisionRecognitionResult *out_result)
{
    if (out_result == NULL || !vision_num_result_ready) {
        return false;
    }

    *out_result = vision_num_result;
    return true;
}

/* 仅读取最近一次图案识别响应；没有收到过响应时返回 false。 */
bool vision_get_latest_img_result(VisionRecognitionResult *out_result)
{
    if (out_result == NULL || !vision_img_result_ready) {
        return false;
    }

    *out_result = vision_img_result;
    return true;
}

/* 读取最新数字识别响应，并清除 updated，适合“一次请求等待一次结果”的流程。 */
bool vision_take_num_result(VisionRecognitionResult *out_result)
{
    if (out_result == NULL || !vision_num_result_updated) {
        return false;
    }

    *out_result = vision_num_result;
    vision_num_result_updated = false;
    return true;
}

/* 读取最新图案识别响应，并清除 updated，适合“一次请求等待一次结果”的流程。 */
bool vision_take_img_result(VisionRecognitionResult *out_result)
{
    if (out_result == NULL || !vision_img_result_updated) {
        return false;
    }

    *out_result = vision_img_result;
    vision_img_result_updated = false;
    return true;
}
