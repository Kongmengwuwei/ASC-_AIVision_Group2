#include "Camera_handler.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

// 初始地图帧协议：$MAP + 地图正文 + $END
#define MAP_FRAME_HEADER_TAG "$MAP"
#define MAP_FRAME_TAIL_TAG "$END"
#define MAP_FRAME_HEADER_TAG_LEN 4U
#define MAP_FRAME_TAIL_TAG_LEN 4U
// 动态更新行的最大长度（如 C123,456）
#define DYNAMIC_LINE_MAX_LEN 48U

// 串口FIFO底层缓存与一次处理读缓冲
uint32 format_count = 0;                              // 完成有效解析（初始帧/动态轮）的计数器
uint8_t uart_fifo_buf[FIFO_SIZE];                     // FIFO底层存储数组
fifo_struct uart_data_fifo;                           // 串口接收FIFO对象
static uint8_t fifo_read_buf[FIFO_READ_BUF_SIZE] = {0}; // 每次从FIFO读出的临时缓冲

// 当前生效地图对象（供外部模块读取）
Point obstacles[MAX_OBSTACLES] = {{0}};                 // 当前障碍物坐标列表
Point boxes[MAX_BOXES] = {{0}};                         // 当前箱子坐标列表
Point targets[MAX_TARGETS] = {{0}};                     // 当前目标点坐标列表
Point map_bombs[MAX_BOMBS] = {{0}};                     // 当前炸弹坐标列表
Point cat_turth_path[MAX_CAR_PATH] = {{0}};             // 预留路径坐标列表
Point car = {1, 2};                                     // 车辆整数栅格位置
CarPosition car_position = {0.0f, 0.0f};             // 车辆浮点栅格位置
CarPosition car_position_m = {0.0f, 0.0f};           // 车辆米制坐标
volatile bool car_position_valid = false;            // 车辆位置是否有效
uint8_t image_find_flag = 0;                         // 车辆位置是否越界（1越界）

size_t actual_obstacles_count = 0;                   // 当前障碍物数量
size_t actual_boxes_count = 0;                       // 当前箱子数量
size_t actual_targets_count = 0;                     // 当前目标点数量
size_t actual_bombs_count = 0;                       // 当前炸弹数量
size_t actual_car_path_count = 0;                    // 当前路径点数量

bool data_reception_complete = false;                // 初始地图数据是否接收完成
BlobInfo blob_info = {0};                            // 色块识别结果缓存

map_state_t map_state = MAP_STATE_INIT;              // 当前解析状态机状态
bool initial_map_ready = false;                      // 初始地图是否已可用
bool uart_data_processing_enabled = false;           // 串口解析总开关
int32 dynamic_map_enable = 1;                        // 是否允许动态地图更新

uint8_t init_map_received_count = 0;                 // 初始地图成功接收次数
bool current_round_complete = false;                 // 当前一轮解析是否完成

uint8_t get_data_1 = 0;                              // 车辆数据到位标记
uint8_t get_data_2 = 0;                              // 障碍物数据到位标记
uint8_t get_data_3 = 0;                              // 箱子数据到位标记
uint8_t get_data_4 = 0;                              // 目标点数据到位标记

// 字节流拼接缓冲：用于拼接串口碎片数据
static uint8_t stream_buf[FRAME_BUF_SIZE] = {0};     // 拼包缓冲区
static uint16_t stream_len = 0;                      // 当前拼包缓冲有效长度

// 动态地图一轮临时缓存：遇到车辆行后统一提交
static Point dynamic_obstacles[MAX_OBSTACLES] = {{0}};              // 本轮动态障碍物缓存
static Point dynamic_boxes[MAX_BOXES] = {{0}};                      // 本轮动态箱子缓存
static Point dynamic_targets[MAX_TARGETS] = {{0}};                  // 本轮动态目标点缓存
static Point dynamic_bombs[MAX_BOMBS] = {{0}};                      // 本轮动态炸弹缓存
static size_t dynamic_obstacles_count = 0;             // 本轮动态障碍物数量
static size_t dynamic_boxes_count = 0;                 // 本轮动态箱子数量
static size_t dynamic_targets_count = 0;               // 本轮动态目标点数量
static size_t dynamic_bombs_count = 0;                 // 本轮动态炸弹数量
static Point dynamic_car = {0, 0};                     // 本轮动态车辆位置
static bool dynamic_car_received = false;              // 本轮是否收到车辆行（提交触发条件）

// 在缓冲区中查找指定子串，返回首位置或-1
static int find_pattern(const uint8_t *buf, uint16_t len, const char *pattern, uint16_t pattern_len)
{
    if (buf == NULL || pattern == NULL || len < pattern_len || pattern_len == 0U)
    {
        return -1;
    }

    for (uint16_t i = 0; i <= (uint16_t)(len - pattern_len); i++) // i: 当前匹配起始下标
    {
        if (0 == memcmp(buf + i, pattern, pattern_len))
        {
            return (int)i;
        }
    }
    return -1;
}
// 在缓冲区中查找单个字节，返回首位置或-1
static int find_byte(const uint8_t *buf, uint16_t len, uint8_t value)
{
    if (buf == NULL)
    {
        return -1;
    }
    for (uint16_t i = 0; i < len; i++) // i: 当前检查的字节下标
    {
        if (buf[i] == value)
        {
            return (int)i;
        }
    }
    return -1;
}
// 跳过前导换行符（\r/\n）
static uint16_t skip_line_break_prefix(const uint8_t *buf, uint16_t len, uint16_t idx)
{
    while (idx < len && (buf[idx] == '\r' || buf[idx] == '\n'))
    {
        idx++;
    }
    return idx;
}
// 去除末尾换行符，返回裁剪后的end
static uint16_t trim_line_break_suffix(const uint8_t *buf, uint16_t begin, uint16_t end)
{
    while (end > begin && (buf[end - 1U] == '\r' || buf[end - 1U] == '\n'))
    {
        end--;
    }
    return end;
}
// 消费一个换行：兼容 \n / \r\n
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
// 丢弃流缓冲前缀字节
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
// 删除流缓冲指定区间 [start, start + remove_len)
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
// 追加新收到字节，必要时丢弃最旧数据避免溢出
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
        uint16_t need_drop = (uint16_t)((uint32_t)stream_len + len - (FRAME_BUF_SIZE - 1U)); // need_drop: 为避免溢出需要丢弃的旧字节数
        consume_stream_prefix(need_drop);
    }

    memcpy(stream_buf + stream_len, data, len);
    stream_len = (uint16_t)(stream_len + len);
}

// 清空一轮动态更新的临时缓存
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
// 将一轮动态缓存提交为当前生效地图
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
        memcpy(obstacles, dynamic_obstacles, dynamic_obstacles_count * sizeof(Point));
        actual_obstacles_count = dynamic_obstacles_count;
        get_data_2 = 2;
    }

    if (dynamic_boxes_count > 0U)
    {
        memset(boxes, 0, sizeof(boxes));
        memcpy(boxes, dynamic_boxes, dynamic_boxes_count * sizeof(Point));
        actual_boxes_count = dynamic_boxes_count;
        get_data_3 = 3;
    }

    if (dynamic_targets_count > 0U)
    {
        memset(targets, 0, sizeof(targets));
        memcpy(targets, dynamic_targets, dynamic_targets_count * sizeof(Point));
        actual_targets_count = dynamic_targets_count;
        get_data_4 = 4;
    }

    if (dynamic_bombs_count > 0U)
    {
        memset(map_bombs, 0, sizeof(map_bombs));
        memcpy(map_bombs, dynamic_bombs, dynamic_bombs_count * sizeof(Point));
        actual_bombs_count = dynamic_bombs_count;
    }

    current_round_complete = true;
    format_count++;
    reset_dynamic_round();
}

// 像素坐标按给定比例换算为栅格坐标
void pixel_to_grid(int pixel_row, int pixel_col, float *grid_row, float *grid_col,
                   float grid_ratio_row, float grid_ratio_col)
{
    if (grid_row != NULL && grid_col != NULL)
    {
        *grid_row = (float)pixel_row / grid_ratio_row;
        *grid_col = (float)pixel_col / grid_ratio_col;
    }
}

// 解析初始地图正文（固定12x16字符矩阵）
static bool parse_map_payload(const uint8_t *payload, uint16_t payload_len)
{
    if (payload == NULL)
    {
        return false;
    }

    Point new_obstacles[MAX_OBSTACLES] = {{0}};              // 新解析出的障碍物列表（临时）
    Point new_boxes[MAX_BOXES] = {{0}};         // 新解析出的箱子列表（临时）
    Point new_targets[MAX_TARGETS] = {{0}};     // 新解析出的目标点列表（临时）
    Point new_bombs[MAX_BOMBS] = {{0}};         // 新解析出的炸弹列表（临时）
    Point new_car = {0, 0};                     // 新解析出的车辆位置（临时）
    size_t new_obstacles_count = 0;             // 新障碍物数量
    size_t new_boxes_count = 0;                 // 新箱子数量
    size_t new_targets_count = 0;               // 新目标点数量
    size_t new_bombs_count = 0;                 // 新炸弹数量
    bool car_found = false;                     // 本帧是否找到且仅找到一个车辆标记
    uint16_t idx = 0U;                          // payload遍历游标

    for (size_t row = 0; row < MAP_ROWS; row++) // row: 当前解析的地图行
    {
        while (idx < payload_len && (payload[idx] == ' ' || payload[idx] == '\t'))
        {
            idx++;
        }

        if ((uint32_t)idx + MAP_COLS > payload_len)
        {
            return false;
        }

        for (size_t col = 0; col < MAP_COLS; col++) // col: 当前解析的地图列
        {
            char cell = (char)payload[idx++]; // cell: 当前格子的符号字符
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
        memcpy(obstacles, new_obstacles, new_obstacles_count * sizeof(Point));
    }
    if (new_boxes_count > 0U)
    {
        memcpy(boxes, new_boxes, new_boxes_count * sizeof(Point));
    }
    if (new_targets_count > 0U)
    {
        memcpy(targets, new_targets, new_targets_count * sizeof(Point));
    }
    if (new_bombs_count > 0U)
    {
        memcpy(map_bombs, new_bombs, new_bombs_count * sizeof(Point
        ));
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

// 从直接输入的多行字符串解析地图（等价于 parse_map_payload 的语义）
bool parse_map_from_string(const char *map_text)
{
    if (map_text == NULL)
    {
        return false;
    }

    size_t payload_len = strlen(map_text);
    if (payload_len == 0U || payload_len > 0xFFFFU)
    {
        return false;
    }

    // 兼容直接输入字符串最后一行没有换行符的情况
    if (map_text[payload_len - 1U] != '\n' && map_text[payload_len - 1U] != '\r')
    {
        if (payload_len + 1U > FRAME_BUF_SIZE)
        {
            return false;
        }

        uint8_t temp_payload[FRAME_BUF_SIZE] = {0};
        memcpy(temp_payload, map_text, payload_len);
        temp_payload[payload_len] = '\n';
        return parse_map_payload(temp_payload, (uint16_t)(payload_len + 1U));
    }

    return parse_map_payload((const uint8_t *)map_text, (uint16_t)payload_len);
}

// 从流缓冲提取并解析 $MAP...$END 完整包
static void parse_map_packets(void)
{
    while (stream_len >= (MAP_FRAME_HEADER_TAG_LEN + MAP_FRAME_TAIL_TAG_LEN))
    {
        int start = find_pattern(stream_buf, stream_len, MAP_FRAME_HEADER_TAG, MAP_FRAME_HEADER_TAG_LEN); // start: $MAP起始位置
        if (start < 0)
        {
            return;
        }

        uint16_t search_from = (uint16_t)start + MAP_FRAME_HEADER_TAG_LEN; // search_from: 开始寻找$END的位置
        int end_rel = find_pattern(stream_buf + search_from, (uint16_t)(stream_len - search_from), // end_rel: 相对search_from的$END偏移
                                   MAP_FRAME_TAIL_TAG, MAP_FRAME_TAIL_TAG_LEN);
        if (end_rel < 0)
        {
            return;
        }

        uint16_t end_tag_pos = (uint16_t)(search_from + end_rel); // end_tag_pos: $END标签起点
        uint16_t payload_start = skip_line_break_prefix(stream_buf, stream_len, search_from); // payload起始（跳过首部换行）
        uint16_t payload_end = end_tag_pos; // payload结束（不含$END）
        uint16_t packet_end = (uint16_t)(end_tag_pos + MAP_FRAME_TAIL_TAG_LEN); // packet_end: 完整包结束位置
        packet_end = skip_line_break_prefix(stream_buf, stream_len, packet_end);

        if (payload_end > payload_start)
        {
            parse_map_payload(stream_buf + payload_start, (uint16_t)(payload_end - payload_start));
        }

        uint16_t packet_len = (uint16_t)(packet_end - (uint16_t)start); // packet_len: 本次要移除的包长度
        remove_stream_range((uint16_t)start, packet_len);
    }
}

// 初始帧模式下压缩缓冲，尽量保留可能的包头前缀
static void compact_init_stream_buffer(void)
{
    int start = find_pattern(stream_buf, stream_len, MAP_FRAME_HEADER_TAG, MAP_FRAME_HEADER_TAG_LEN); // start: 当前缓冲中$MAP位置
    if (start > 0)
    {
        consume_stream_prefix((uint16_t)start);
        return;
    }

    if (start < 0 && stream_len > MAP_FRAME_HEADER_TAG_LEN)
    {
        uint16_t keep = MAP_FRAME_HEADER_TAG_LEN - 1U; // keep: 保留的后缀长度，用于跨包头匹配
        memmove(stream_buf, stream_buf + stream_len - keep, keep);
        stream_len = keep;
    }
}

// 解析动态行：C/O/B/T/D + row,col
static void parse_dynamic_line(const char *line)
{
    char symbol = 0; // symbol: 行类型标识（C/#/B/T/D）
    int raw_row = 0; // raw_row: 串口上传的原始行坐标
    int raw_col = 0; // raw_col: 串口上传的原始列坐标

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
        float row_grid = (float)raw_row * 0.01f;   // row_grid: 车辆浮点行坐标（raw/100）
        float col_grid = (float)raw_col * 0.01f;   // col_grid: 车辆浮点列坐标（raw/100）
        int car_grid_row = (int)lroundf(row_grid); // car_grid_row: 四舍五入后的整型行坐标
        int car_grid_col = (int)lroundf(col_grid); // car_grid_col: 四舍五入后的整型列坐标

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
// 连续解析动态行，遇到$MAP头则交回包解析流程
static void parse_dynamic_lines(void)
{
    while (stream_len > 0U)
    {
        if (stream_len >= MAP_FRAME_HEADER_TAG_LEN &&
            0 == memcmp(stream_buf, MAP_FRAME_HEADER_TAG, MAP_FRAME_HEADER_TAG_LEN))
        {
            break;
        }

        int line_end = find_byte(stream_buf, stream_len, (uint8_t)'\n'); // line_end: 当前行换行符位置
        if (line_end < 0)
        {
            return;
        }

        uint16_t line_len = (uint16_t)(line_end + 1); // line_len: 当前行总长度（含换行）
        if (line_len >= DYNAMIC_LINE_MAX_LEN)
        {
            consume_stream_prefix(line_len);
            continue;
        }

        char line[DYNAMIC_LINE_MAX_LEN]; // line: 单行动态数据字符串缓存
        memcpy(line, stream_buf, line_len);
        line[line_len] = '\0';
        parse_dynamic_line(line);
        consume_stream_prefix(line_len);
    }
}

// 初始化串口接收、FIFO与模块状态
void uart_blob_init(void)
{
    uart_init(UART_2, 115200, UART2_TX_B18, UART2_RX_B19);

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

// 主入口：从FIFO取数据并驱动两类协议解析
void process_blob_data(void)
{
    if (!uart_data_processing_enabled)
    {
        return;
    }

    uint32_t read_len = fifo_used(&uart_data_fifo); // read_len: 本轮准备从FIFO读取的字节数
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
