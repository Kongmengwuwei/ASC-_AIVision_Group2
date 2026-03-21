#include "config.h"


tagPID_T ULpid;
tagPID_T URpid;
tagPID_T DLpid;
tagPID_T DRpid;
tagPID_T Yawpid;
tagPID_T Camera_x_pid;
tagPID_T Camera_y_pid;
tagPID_T Gyro_rotate_pid;

PIDInitStruct ULPidInitStruct = 
{
	.fKp       = 9.7,     //.fKp  9.7
	.fKi       = 2.2,     //.fKi 2.05
	.fKd       = 0,     //.fKd
	.fMax_Iout = 4000,      //.fMax_Iout
	.fMax_Out  = 6000,       //.fMax_Out
	.alpha     = 0.9
};

PIDInitStruct URPidInitStruct = 
{
	.fKp       = 7.7,     //.fKp 8.6
	.fKi       = 2.1,     //.fKi 2.0
	.fKd       = 0,     //.fKd
	.fMax_Iout = 4000,      //.fMax_Iout
	.fMax_Out  = 6000,       //.fMax_Out
	.alpha     = 0.9
};

PIDInitStruct DLPidInitStruct = 
{
	.fKp       = 7.5,     //.fKp  9.7
	.fKi       = 2,     //.fKi 2.05
	.fKd       = 0,     //.fKd
	.fMax_Iout = 4000,      //.fMax_Iout
	.fMax_Out  = 6000,       //.fMax_Out
	.alpha     = 0.9
};

PIDInitStruct DRPidInitStruct = 
{
	.fKp       = 9.8,     //.fKp 8.6
	.fKi       = 2.1,     //.fKi 2.0
	.fKd       = 0,     //.fKd
	.fMax_Iout = 4000,      //.fMax_Iout
	.fMax_Out  = 6000,       //.fMax_Out
	.alpha     = 0.9
};
PIDInitStruct YawPidInitStruct = 
{
	.fKp       = 1.2,     //.fKp 8.6
	.fKi       = 0,     //.fKi 2.0
	.fKd       = 2.1,     //.fKd
	.fMax_Iout = 100,      //.fMax_Iout
	.fMax_Out  = 150,       //.fMax_Out
	.alpha     = 0.9
};

PIDInitStruct Camera_x_PidInitStruct = 
{
	.fKp       = 0.4,     //.fKp 8.6
	.fKi       = 0,     //.fKi 2.0
	.fKd       = 4,     //.fKd
	.fMax_Iout = 100,      //.fMax_Iout
	.fMax_Out  = 150,       //.fMax_Out
	.alpha     = 0.9
};

PIDInitStruct Camera_y_PidInitStruct = 
{
	.fKp       = 1.4,     //.fKp 8.6
	.fKi       = 0,     //.fKi 2.0
	.fKd       = 8,     //.fKd
	.fMax_Iout = 100,      //.fMax_Iout
	.fMax_Out  = 150,       //.fMax_Out
	.alpha     = 0.9
};

PIDInitStruct Gyro_Rotate_PidInitStruct = 
{
	.fKp       = 1.1,     //.fKp 8.6
	.fKi       = 0,     //.fKi 2.0
	.fKd       = 2.5,     //.fKd
	.fMax_Iout = 100,      //.fMax_Iout
	.fMax_Out  = 150,       //.fMax_Out
	.alpha     = 0.9
};
