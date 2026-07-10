#ifndef __BLUESERIAL_H
#define __BLUESERIAL_H

/* UART8 蓝牙模块初始化和发送辅助函数。 */
void Blue_Serial_Init(void);
void BlueSerial_Printf(char *format, ...);
void BlueSerial_GetLastRxFrame(char *buffer, uint16 length);

/* 由 LPUART8_IRQHandler 调用：只收字节、拼帧、入队，不在中断里解析命令。 */
void BlueSerial_RxIrqHandler(void);

/* 主循环调用：解析已经入队的 [command] 帧。PathDebugReport() 内部也会调用它。 */
void BlueSerial_CommandTask(void);

/*
 * 查询蓝牙是否正在接管底盘输出。PIT1 用它决定是否短路原有控制分支。
 * 返回 1：蓝牙正在原始 PWM、路径移动或航向保持；返回 0：交还原工程控制逻辑。
 */
uint8 BlueSerial_IsControlActive(void);

/*
 * 蓝牙接管时的 10ms 控制 tick，在 encoder_get() 之后调用。
 * 根据当前蓝牙模式选择原始四轮 PWM、路径运动、航向保持或停车输出。
 */
void BlueSerial_ControlTick10ms(void);

/* 路径调试接口：主循环调用 Report 服务蓝牙命令；周期遥测由 BlueSerial.c 宏控制。 */
void BlueSerial_PathDebugTick10ms(void);
void BlueSerial_PathDebugReport(void);

#endif
