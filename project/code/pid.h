#ifndef _pid_h_
#define _pid_h_

typedef struct
{
    float fCurr_Value;  /* 当前值 */
    float fExp_Value;   /* 设定的期望值 */
	
    float fKp;          /* 比例系数 */
    float fKi;          /* 微分系数 */
    float fKd;          /* 积分系数 */
    
    float fPout;        /* 比例项输出 */
    float fIout;        /* 积分项输出 */
    float fDout;        /* 微分项输出 */

    float fCtrl_Out;    /* 本次输出值 */
    float fPre_Out;     /* 前一次输出值 */

    float fMax_Iout;    /* PID最大积分输出 */

    float fDbuf[3];     /* 微分项最近三个值 0为最新 1上一次 2上上次 */
    float fError[3];    /* 误差项最近三个值 0为最新 1上一次 2上上次 */
	
    int fMax_Out;     /* PID最大输出 */
	
	float alpha;
	
}tagPID_T;

typedef struct
{
    float fKp;          /* 比例系数 */
    float fKi;          /* 微分系数 */
    float fKd;          /* 积分系数 */

    float fMax_Iout;    /* PID最大积分输出 */
    float fMax_Out;     /* PID最大输出 */
	
	float alpha;

}PIDInitStruct;

void PID_Init(tagPID_T *_tPid,PIDInitStruct *_tPidInit);
void PID_Update(tagPID_T *_tPid,PIDInitStruct *_tPidInit);
int PID_Add_Calculate(tagPID_T *_tPid,float _fCurrValue,float _fExpValue);
int PID_Location_Calculate(tagPID_T *_tPid,float _fCurrValue,float _fExpValue);
void PID_Clear(tagPID_T *_tPid);

#endif
