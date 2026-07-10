#ifndef __BLUESERIAL_H
#define __BLUESERIAL_H

/* UART4 Bluetooth module initialization and transmit helpers. */
void Blue_Serial_Init(void);
void BlueSerial_Printf(char *format, ...);

/* Called by LPUART4_IRQHandler. Only receives and queues bytes. */
void BlueSerial_RxIrqHandler(void);

/* Called in the main loop. Parses queued [command] frames. */
void BlueSerial_CommandTask(void);

/*
 * Stand-alone tuning control tick. Call once every 10 ms after encoder_get().
 * It selects raw four-wheel PWM, path motion, or stopped output.
 */
void BlueSerial_ControlTick10ms(void);

/* Existing path debug report API. It also services Bluetooth commands. */
void BlueSerial_PathDebugTick10ms(void);
void BlueSerial_PathDebugReport(void);

#endif
