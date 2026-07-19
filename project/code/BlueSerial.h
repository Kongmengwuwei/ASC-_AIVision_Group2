#ifndef __BLUESERIAL_H
#define __BLUESERIAL_H

/* UART8 蓝牙模块初始化和发送辅助函数。 */
void Blue_Serial_Init(void);
void BlueSerial_SetEnabled(uint8 enabled);
uint8 BlueSerial_GetEnabled(void);
void BlueSerial_Printf(char *format, ...);
void BlueSerial_GetLastRxFrame(char *buffer, uint16 length);
/* Send the existing POSITION vision/odometry report without requesting a new camera frame. */
void BlueSerial_ReportPosition(void);

/* 由 LPUART8_IRQHandler 调用：只收字节、拼帧、入队，不在中断里解析命令。 */
void BlueSerial_RxIrqHandler(void);

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

/* 10 ms timebase for the non-blocking 100 ms Bluetooth telemetry report. */
void BlueSerial_TelemetryTick10ms(void);

/* 主循环调用：处理蓝牙命令及异步动作完成状态。 */
void BlueSerial_Task(void);

#endif
