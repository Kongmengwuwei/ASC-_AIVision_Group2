#ifndef __BLUESERIAL_H
#define __BLUESERIAL_H
void Blue_Serial_Init(void);
void BlueSerial_Printf(char *format, ...);
void BlueSerial_PathDebugTick10ms(void);
void BlueSerial_PathDebugReport(void);
#endif
