#ifndef _WIFI_H_
#define _WIFI_H_

#include "zf_common_typedef.h"
#include "zf_device_wifi_spi.h"

#define WIFI_PRINTF_BUFFER_SIZE (256)
#define WIFI_RECEIVE_BUFFER_SIZE (512)
#define WIFI_SSID_BUFFER_SIZE (33)
#define WIFI_PASSWORD_BUFFER_SIZE (65)
#define WIFI_IP_BUFFER_SIZE (16)
#define WIFI_PORT_BUFFER_SIZE (6)

// wifi信息结构体，方便获取wifi的各种信息
typedef struct
{
    uint8 connect_state;                      // 1:  连接, 0: 未连接
    char ssid[WIFI_SSID_BUFFER_SIZE];         // wifi名字
    char password[WIFI_PASSWORD_BUFFER_SIZE]; // 密码
    char ip[WIFI_IP_BUFFER_SIZE];             // ip
    char target_port[WIFI_PORT_BUFFER_SIZE];  // 目标端口
    char local_port[WIFI_PORT_BUFFER_SIZE];   // 本地端口
} wifi_info_struct;

extern wifi_info_struct wifi_info;

uint8 wifi_init(char *wifi_ssid, char *pass_word, const char *target_ip);
void wifi_test(char *wifi_ssid, char *pass_word);
int wifi_printf(const char *format, ...);

uint32 wifi_receive_update(void);
const uint8 *wifi_get_receive_buffer(void);
uint32 wifi_get_receive_length(void);
uint8 wifi_get_receive_overflow(void);
void wifi_clear_receive_buffer(void);

#endif
