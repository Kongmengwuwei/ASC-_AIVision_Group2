#ifndef __CONFIG_H_
#define __CONFIG_H_

#include "pid.h"


//#define Basic_Speed 2200

extern tagPID_T ULpid;
extern tagPID_T URpid;
extern tagPID_T DLpid;
extern tagPID_T DRpid;
extern tagPID_T Yawpid;
extern tagPID_T Camera_x_pid;
extern tagPID_T Camera_y_pid;
extern tagPID_T Gyro_rotate_pid;

extern PIDInitStruct ULPidInitStruct;
extern PIDInitStruct URPidInitStruct;
extern PIDInitStruct DLPidInitStruct;
extern PIDInitStruct DRPidInitStruct;
extern PIDInitStruct YawPidInitStruct;
extern PIDInitStruct Camera_x_PidInitStruct;
extern PIDInitStruct Camera_y_PidInitStruct;
extern PIDInitStruct Gyro_Rotate_PidInitStruct;

#endif
