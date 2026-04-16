/*
蓝牙串口调试模块，配合江协的小程序可以比较方便的调试，但是蓝牙程序可能会对主程序造成比较大的延迟
*/

#include "zf_common_headfile.h"
#include "zf_driver_uart.h"


//初始化，受硬件限制用的是USRT4会导致一个摄像头用不了
void Blue_Serial_Init(void)
{
	uart_init(UART_4, 115200,UART4_TX_C16, UART4_RX_C17);
}



void BlueSerial_SendByte(uint8_t Byte)
{
	uart_write_byte (UART_4, Byte);
}

void BlueSerial_SendArray(uint8_t *Array, uint16_t Length)
{
	uint16_t i;
	for (i = 0; i < Length; i ++)
	{
		BlueSerial_SendByte(Array[i]);
	}
}

void BlueSerial_SendString(char *String)
{
	uint8_t i;
	for (i = 0; String[i] != '\0'; i ++)
	{
		BlueSerial_SendByte(String[i]);
	}
}

uint32_t BlueSerial_Pow(uint32_t X, uint32_t Y)
{
	uint32_t Result = 1;
	while (Y --)
	{
		Result *= X;
	}
	return Result;
}

void BlueSerial_SendNumber(uint32_t Number, uint8_t Length)
{
	uint8_t i;
	for (i = 0; i < Length; i ++)
	{
		BlueSerial_SendByte(Number / BlueSerial_Pow(10, Length - i - 1) % 10 + '0');
	}
}


//和printf一样用就好了
void BlueSerial_Printf(char *format, ...)
{
	char String[100];
	va_list arg;
	va_start(arg, format);
	vsprintf(String, format, arg);
	va_end(arg);
	BlueSerial_SendString(String);
}

