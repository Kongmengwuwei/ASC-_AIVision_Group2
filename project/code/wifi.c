#include "wifi.h"
#include "zf_common_headfile.h"

static uint8 wifi_receive_buffer[WIFI_RECEIVE_BUFFER_SIZE];
static uint32 wifi_receive_length = 0;
static uint8 wifi_receive_overflow = 0;


//wifi初始化，参数分别是wifi名字，密码和ip(可有可无)
uint8 wifi_init(char *wifi_ssid, char *pass_word, const char *target_ip)
{
    const char *connect_ip = target_ip;
    uint8 return_state;

    if((NULL == connect_ip) || ('\0' == connect_ip[0]))
    {
        connect_ip = WIFI_SPI_TARGET_IP;
    }

    return_state = wifi_spi_init(wifi_ssid, pass_word);
    if(return_state)
    {
        return return_state;
    }

    return wifi_spi_socket_connect("UDP",
                                   (char *)connect_ip,
                                   WIFI_SPI_TARGET_PORT,
                                   WIFI_SPI_LOCAL_PORT);
}

// wifi 测试函数用于获取相关ip信息，最好单独使用
void wifi_test(char *wifi_ssid, char *pass_word)
{
    while(wifi_spi_init(wifi_ssid, pass_word))
    {
        printf("\r\n unconnected");
        system_delay_ms(100);
    }

    printf("\r\n module version:%s", wifi_spi_version);
    printf("\r\n module mac    :%s", wifi_spi_mac_addr);
    printf("\r\n module ip     :%s", wifi_spi_ip_addr_port);
}

//像prinft一样用
int wifi_printf(const char *format, ...)
{
    static char wifi_printf_buffer[WIFI_PRINTF_BUFFER_SIZE];
    va_list args;
    int write_length;
    uint32 send_length;
    uint32 remain_length;

    if(NULL == format)
    {
        return -1;
    }

    va_start(args, format);
    write_length = vsnprintf(wifi_printf_buffer, sizeof(wifi_printf_buffer), format, args);
    va_end(args);

    if(write_length < 0)
    {
        return -1;
    }

    send_length = (uint32)write_length;
    if(send_length >= sizeof(wifi_printf_buffer))
    {
        send_length = sizeof(wifi_printf_buffer) - 1;
    }

    remain_length = wifi_spi_send_buffer((const uint8 *)wifi_printf_buffer, send_length);
    wifi_spi_udp_send_now();

    if(0 != remain_length)
    {
        return -1;
    }

    return (int)send_length;
}


//一下是数据处理现在还没用
uint32 wifi_receive_update(void)
{
    uint32 read_length;
    uint32 free_length;

    if(wifi_receive_length >= WIFI_RECEIVE_BUFFER_SIZE)
    {
        wifi_receive_overflow = 1;
        return 0;
    }

    free_length = WIFI_RECEIVE_BUFFER_SIZE - wifi_receive_length;
    read_length = wifi_spi_read_buffer(&wifi_receive_buffer[wifi_receive_length], free_length);
    wifi_receive_length += read_length;

    if((0 != read_length) && (wifi_receive_length >= WIFI_RECEIVE_BUFFER_SIZE))
    {
        wifi_receive_overflow = 1;
    }

    return read_length;
}

const uint8 *wifi_get_receive_buffer(void)
{
    return wifi_receive_buffer;
}

uint32 wifi_get_receive_length(void)
{
    return wifi_receive_length;
}

uint8 wifi_get_receive_overflow(void)
{
    return wifi_receive_overflow;
}

void wifi_clear_receive_buffer(void)
{
    memset(wifi_receive_buffer, 0, sizeof(wifi_receive_buffer));
    wifi_receive_length = 0;
    wifi_receive_overflow = 0;
}
