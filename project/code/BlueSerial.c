/*
蓝牙串口调试模块，配合江协的小程序可以比较方便的调试，但是蓝牙程序可能会对主程序造成比较大的延迟
*/

#include "zf_common_headfile.h"
#include "zf_driver_uart.h"
#include "path_follow.h"
#include "Motor.h"
#include "Attitude.h"
#include <stdarg.h>
#include <stdio.h>

#define BLUESERIAL_PATH_REPORT_PERIOD_TICKS 5U

static volatile uint8 g_blueserial_path_report_pending = 0U;


// BlueSerial uses UART4.
void Blue_Serial_Init(void)
{
	uart_init(UART_4, 115200, UART4_TX_C16, UART4_RX_C17);
}



void BlueSerial_SendByte(uint8_t Byte)
{
	uart_write_byte(UART_4, Byte);
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


//和printf一样用就好�?
void BlueSerial_Printf(char *format, ...)
{
	char String[220];
	va_list arg;
	va_start(arg, format);
	vsprintf(String, format, arg);
	va_end(arg);
	BlueSerial_SendString(String);
}

void BlueSerial_PathDebugTick10ms(void)
{
	static uint8 tick_div = 0U;

	if (++tick_div >= BLUESERIAL_PATH_REPORT_PERIOD_TICKS)
	{
		tick_div = 0U;
		g_blueserial_path_report_pending = 1U;
	}
}
static void BlueSerial_GetActualBodySpeed(float *vx_cmps, float *vy_cmps, float *omega_radps)
{
	float count_to_mps;
	float w_ul;
	float w_ur;
	float w_dl;
	float w_dr;

	if (vx_cmps == NULL || vy_cmps == NULL || omega_radps == NULL)
	{
		return;
	}

	*vx_cmps = 0.0f;
	*vy_cmps = 0.0f;
	*omega_radps = 0.0f;

	if (pulse_per_meter <= 0.0)
	{
		return;
	}

	count_to_mps = ((float)PID_RATE) / (float)pulse_per_meter;
	w_ul = (float)up_L_all * count_to_mps;
	w_ur = (float)up_R_all * count_to_mps;
	w_dl = (float)down_L_all * count_to_mps;
	w_dr = (float)down_R_all * count_to_mps;

	*vx_cmps = 0.25f * (w_ul + w_ur + w_dl + w_dr) * 100.0f;
	*vy_cmps = 0.25f * (-w_ul + w_ur + w_dl - w_dr) * 100.0f;
	*omega_radps = (-w_ul + w_ur - w_dl + w_dr) / (2.0f * D_X + 2.0f * D_Y);
}

void BlueSerial_PathDebugReport(void)
{
	path_follow_status_t st = {0};
	float actual_vx_cmps = 0.0f;
	float actual_vy_cmps = 0.0f;
	float actual_omega_radps = 0.0f;

	if (g_blueserial_path_report_pending == 0U)
	{
		return;
	}
	g_blueserial_path_report_pending = 0U;

	path_follow_get_status(&st);
	BlueSerial_GetActualBodySpeed(&actual_vx_cmps, &actual_vy_cmps, &actual_omega_radps);

	BlueSerial_Printf("TPOS %.3f %.3f APOS %.3f %.3f TVEL %.1f %.1f %.2f AVEL %.1f %.1f %.2f TYAW %.2f\r\n",
					  st.target_x_m,
					  st.target_y_m,
					  st.x_m,
					  st.y_m,
					  speed_three_array[0],
					  speed_three_array[1],
					  speed_three_array[2],
					  actual_vx_cmps,
					  actual_vy_cmps,
					  actual_omega_radps,
					  st.target_yaw_deg);
}
