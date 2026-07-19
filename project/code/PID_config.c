#include "PID_config.h"
#include "Motor.h"


tagPID_T ULpid;
tagPID_T URpid;
tagPID_T DLpid;
tagPID_T DRpid;
tagPID_T Yawpid;
tagPID_T Camera_x_pid;
tagPID_T Camera_y_pid;
tagPID_T Gyro_rotate_pid;

#if MOTOR_BOARD_USE_NEW
PIDInitStruct ULPidInitStruct = 
{
	.fKp       = 10.0,    //.fKp
	.fKi       = 0.8,     //.fKi
	.fKd       = 25.0,    //.fKd
	.fMax_Iout = 4000,      //.fMax_Iout
	.fMax_Out  = 6000,       //.fMax_Out
	.alpha     = 0.9
};

PIDInitStruct URPidInitStruct = 
{
	.fKp       = 14.0,    //.fKp
	.fKi       = 1.1,     //.fKi
	.fKd       = 20.0,    //.fKd
	.fMax_Iout = 4000,      //.fMax_Iout
	.fMax_Out  = 6000,       //.fMax_Out
	.alpha     = 0.9
};

PIDInitStruct DLPidInitStruct = 
{
	.fKp       = 8.0,     //.fKp
	.fKi       = 1.1,     //.fKi
	.fKd       = 20.0,    //.fKd
	.fMax_Iout = 4000,      //.fMax_Iout
	.fMax_Out  = 6000,       //.fMax_Out
	.alpha     = 0.8
};

PIDInitStruct DRPidInitStruct = 
{
	.fKp       = 15.0,    //.fKp
	.fKi       = 1.4,     //.fKi
	.fKd       = 25.0,    //.fKd
	.fMax_Iout = 4000,      //.fMax_Iout
	.fMax_Out  = 6000,       //.fMax_Out
	.alpha     = 0.9
};
#else
/* Old-board values calibrated with independent wheel steps at 40/80/120 counts. */
PIDInitStruct ULPidInitStruct =
{
	.fKp       = 12.0,
	.fKi       = 0.6,
	.fKd       = 5.0,
	.fMax_Iout = 4000,
	.fMax_Out  = 6000,
	.alpha     = 0.9
};

PIDInitStruct URPidInitStruct =
{
	.fKp       = 12.0,
	.fKi       = 0.6,
	.fKd       = 5.0,
	.fMax_Iout = 4000,
	.fMax_Out  = 6000,
	.alpha     = 0.9
};

PIDInitStruct DLPidInitStruct =
{
	.fKp       = 12.0,
	.fKi       = 0.8,
	.fKd       = 5.0,
	.fMax_Iout = 4000,
	.fMax_Out  = 6000,
	.alpha     = 0.9
};

PIDInitStruct DRPidInitStruct =
{
	.fKp       = 12.0,
	.fKi       = 0.6,
	.fKd       = 5.0,
	.fMax_Iout = 4000,
	.fMax_Out  = 6000,
	.alpha     = 0.9
};
#endif
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
