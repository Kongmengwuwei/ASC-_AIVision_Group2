#include "zf_common_headfile.h"
#include "Serial.h"

uint8_t UART1_ReceiveFlag=0;	//接收标志位读取后要清0
uint8_t UART2_ReceiveFlag=0;


//UART1使用FIFO环形缓冲区
#define UART_BUFFER_SIZE  256
uint8_t uart_rx_buffer[UART_BUFFER_SIZE];
fifo_struct uart_rx_fifo;



//串口初始化，现在只初始化UART1图像摄像头通信
//调试初始化最好取消
//波特率115200
void Serial_Init(void)
{
	//UART1以及FIFO初始化
	uart_init(UART_1, 115200, UART1_TX_B12, UART1_RX_B13);                          // 初始化UART1也是默认的DebugUart
    uart_rx_interrupt(UART_1, ZF_ENABLE);                                   					// 开启 UART_INDEX 的接收中断
    interrupt_set_priority(LPUART1_IRQn, 0);                                   							// 设置中断优先级为 0
	fifo_init(&uart_rx_fifo, FIFO_DATA_8BIT, uart_rx_buffer, UART_BUFFER_SIZE);							// 8位数据模式，角度和小车位置要做特殊处理

	//串口二数据也放在FIFO缓冲区里
	uart_init(UART_4, 115200, UART4_TX_C16, UART4_RX_C17);                                              // 初始化UART4
    uart_rx_interrupt(UART_4, ZF_ENABLE);                                   					        // 开启 UART_INDEX 的接收中断
    interrupt_set_priority(LPUART4_IRQn, 0);                                   							// 设置中断优先级为 0
}


void SendCommd(Order commd)
{
	switch (commd)
	{
		case Get_map:
		{
			uart_write_string(UART_1, "MAP\n");
			break;
		}
		case Get_car:
		{
			uart_write_string(UART_1, "CAR\n");
			break;
		}
		case Get_number:
		{
			uart_write_string(UART_4, "NUM\n");
			break;
		}
		default :
		{
			break;
		}
		
	}
}






//-------------------------------------------------------------------------------------------------------------------
// 函数简介     串口数据处理函数,使用状态机思路处理 FIFO缓冲区里的数据
// 返回说明     没有返回值，但会置数据处理标志位，有空闲、处理中、处理完毕（处理完毕在读取后要置为空闲）
// 使用方法     在主循环中调用即可
//-------------------------------------------------------------------------------------------------------------------
uint8_t DataHandleFlag=0; 

void DataHandle (void)
{
	//包头包尾初值
	char Title[6]={0};
	char End[6]={0};
	
	
	if(UART1_ReceiveFlag==1)
	{
		UART1_ReceiveFlag=0;
		{
			
		}
	}
}















//-------------------------------------------------------------------------------------------------------------------
// 函数简介     UART_INDEX 的接收中断处理函数 这个函数将在 UART_INDEX 对应的中断调用 详见 isr.c
// 参数说明     void
// 返回参数     void
// 使用示例     uart_rx_interrupt_handler();
//-------------------------------------------------------------------------------------------------------------------
void uart1_rx_interrupt_handler (void)
{
	UART1_ReceiveFlag=1;
	
	uint8_t data;
	uart_query_byte(DEBUG_UART_INDEX, &data);                             // 接收数据 查询式 有数据会返回 TRUE 没有数据会返回 FALSE
    fifo_write_buffer(&uart_rx_fifo, &data, 1);                           // 将数据写入 fifo 中
	
}



void uart4_rx_interrupt_handler (void)
{
	UART2_ReceiveFlag=1;
	
	uint8_t data;
	uart_query_byte(UART_4, &data);                             // 接收数据 查询式 有数据会返回 TRUE 没有数据会返回 FALSE
    fifo_write_buffer(&uart_rx_fifo, &data,1);                           // 将数据写入 fifo 中
	
}




