#ifndef _WIFI_H_
#define _WIFI_H_

#include "zf_common_typedef.h"
#include "zf_device_wifi_spi.h"

#define WIFI_PRINTF_BUFFER_SIZE      (256)
#define WIFI_RECEIVE_BUFFER_SIZE     (512)

uint8 wifi_init(char *wifi_ssid, char *pass_word, const char *target_ip);
void wifi_test(char *wifi_ssid, char *pass_word);

int wifi_printf(const char *format, ...);

uint32 wifi_receive_update(void);
const uint8 *wifi_get_receive_buffer(void);
uint32 wifi_get_receive_length(void);
uint8 wifi_get_receive_overflow(void);
void wifi_clear_receive_buffer(void);

#endif
