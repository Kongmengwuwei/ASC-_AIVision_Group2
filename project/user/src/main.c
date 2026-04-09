#include "zf_common_headfile.h"
#include "Mymenu.h"
#include "Attitude.h"
#include "data_handle.h"

 
int main(void)
{
    clock_init(SYSTEM_CLOCK_600M);  
    //debug_init();                  
	
	//陀螺仪及解算中断初始化
	imu963ra_init();
	Attitude_Init();
	pit_ms_init(PIT_CH1, 2);
	interrupt_set_priority(PIT_IRQn, 0);
	
	//串口即数据处理初始化
	uart_blob_init();
	

    // 此处编写用户代码 例如外设初始化代码等
	pit_ms_init(PIT_CH0, 20);                    // 初始化 PIT_CH0 为周期中断 20ms 周期
    interrupt_set_priority(PIT_IRQn, 2);         // 设置 PIT0 对周期中断的中断优先级为 2
    Menu_Init();
	interrupt_global_enable(0);
	
	printf("1");
	
    while(1)
    {	
	    Menu_Switch();
        Menu_Show();
		
		
		//地图获取调试
		static uint8_t map_req_sent = 0;
		uint8_t row = 0;

		if (!map_req_sent)
		{
			uart_send_map_request();   // 向上位机发送 "MAP"
			map_req_sent = 1;
		}

		process_blob_data();

		if (map_data_updated)
		{
			uart_write_string(UART_INDEX, "$MAP\r\n");

			for (row = 0; row < MAP_ROWS; row++)
			{
				uart_write_buffer(UART_INDEX, (const uint8_t *)map_data[row], MAP_COLS);
				uart_write_string(UART_INDEX, "\r\n");
			}

			uart_write_string(UART_INDEX, "$END\r\n");

			map_data_updated = false;
		}
			

	}
}

void pit_0_handler (void)
{
	key_scanner();                                  // 周期中断触发标志位置位
}


void pit_1_handler(void)
{
	Attitude_Calculate();
}



