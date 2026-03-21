#include "pid.h"

/**
 * @brief PID初始函数
 * @param _tPid-PID结构体指针
 * @param _tPidInit-PID初始化结构体
 * @retval Null
*/
void PID_Init(tagPID_T *_tPid,PIDInitStruct *_tPidInit)
{
	
	/* 历史数据置0 */
    _tPid->fDbuf[0] = _tPid->fDbuf[1] = _tPid->fDbuf[2] = _tPid->fError[0] = _tPid->fError[1] = _tPid->fError[2] = _tPid->fPout = _tPid->fIout = _tPid->fDout = _tPid->fCtrl_Out = _tPid->fPre_Out = 0.0f;
	
	/* 参数赋值 */
		_tPid->fKp = _tPidInit->fKp;
		_tPid->fKi = _tPidInit->fKi;
		_tPid->fKd = _tPidInit->fKd;
		
		_tPid->fMax_Iout = _tPidInit->fMax_Iout;
		_tPid->fMax_Out = _tPidInit->fMax_Out;
	
		_tPid->alpha = _tPidInit->alpha;

}

/**
 * @brief 更新PID系数函数
 * @param _tPid-PID结构体指针
 * @param _tPidInit-PID初始化结构体
 * @retval Null
*/
void PID_Update(tagPID_T *_tPid,PIDInitStruct *_tPidInit)
{
    /* 更新PID系数 */

		_tPid->fKp = _tPidInit->fKp;
		_tPid->fKi = _tPidInit->fKi;
		_tPid->fKd = _tPidInit->fKd;
		
		_tPid->fMax_Iout = _tPidInit->fMax_Iout;
		_tPid->fMax_Out = _tPidInit->fMax_Out;
	
		_tPid->alpha = _tPidInit->alpha;

}


///**
// * @brief PID计算函数
// * @param _tPid-PID结构体指针
// * @param _fCurrValue-当前值
// * @param _fExpValue-期望值
// * @param dt-时间微分，接近即可
// * @retval float 经过PID计算后的输出值
//*/
//int PID_Calculate(tagPID_T *_tPid,float _fCurrValue,float _fExpValue,float dt)
//{
//    /* 设定期望值和当前值 */
//    _tPid->fExp_Value  = _fExpValue;
//    _tPid->fCurr_Value = _fCurrValue;
//    
//	_tPid->fError[2] = _tPid->fError[1];
//	_tPid->fError[1] = _tPid->fError[0];
//    _tPid->fError[0] = _fExpValue - _fCurrValue;
//	
//    /* 迭代微分项的数组 */
//    _tPid->fDbuf[2] = _tPid->fDbuf[1];
//    _tPid->fDbuf[1] = _tPid->fDbuf[0];

//    /* 两次微分求平均*/
//    _tPid->fDbuf[0] = (_tPid->fError[0] - 2.0f * _tPid->fError[1] + _tPid->fError[2]) / 2.0f;
//	
//    _tPid->fPout = _tPid->fKp * _tPid->fError[0];
//    _tPid->fIout += _tPid->fKi * _tPid->fError[0] * dt;
//	_tPid->fDout = _tPid->fKd * _tPid->fDbuf[0] / dt;
//	//限幅
//	if(_tPid->fIout > _tPid->fMax_Iout)
//		_tPid->fIout = _tPid->fMax_Iout;
//    
//    /* 结果处理 */
//    _tPid->fPre_Out = _tPid->fCtrl_Out;
//    _tPid->fCtrl_Out = (int)(_tPid->fPout + _tPid->fIout + _tPid->fDout);
//	//限幅
//	if(_tPid->fPre_Out > _tPid->fMax_Out)
//		_tPid->fPre_Out = _tPid->fMax_Out;
//	
//    return _tPid->fCtrl_Out;
//}

/**
 * @brief PID增量计算函数
 * @param _tPid-PID结构体指针
 * @param _fCurrValue-当前值
 * @param _fExpValue-期望值
 * @retval int 经过PID计算后的输出值
*/
int PID_Add_Calculate(tagPID_T *_tPid,float _fCurrValue,float _fExpValue)
{
    int Delt_Out; /* PID输出增量值 */
	float Pre_I;

    /* 设定期望值和当前值 */
    _tPid->fExp_Value  = _fExpValue;
    _tPid->fCurr_Value = _fCurrValue;
    
	/* 存放过去两次误差值 */
    _tPid->fError[2] = _tPid->fError[1];
    _tPid->fError[1] = _tPid->fError[0];
    _tPid->fError[0] = _fExpValue - _fCurrValue;
	
	/* 记录过去的Iout项的值，用于低通滤波 */
	Pre_I = _tPid->fIout;

    /* 采用增量式PID */

    /* 以本次误差与上次误差的差值作为比例项的输入带入计算 */
    _tPid->fPout = _tPid->fKp * (_tPid->fError[0] - _tPid->fError[1]);

    /* 以本次误差作为积分项带入计算 */
    _tPid->fIout = _tPid->fKi * _tPid->fError[0];
	/* 对积分项采用低通滤波 */
	_tPid->fIout = _tPid->alpha * _tPid->fIout + (1 - _tPid->alpha) * Pre_I;

	//限幅
	if(_tPid->fIout > _tPid->fMax_Iout)
		_tPid->fIout = _tPid->fMax_Iout;
	if(_tPid->fIout < -_tPid->fMax_Iout)
		_tPid->fIout = -_tPid->fMax_Iout;

    /* 迭代微分项的数组 */
    _tPid->fDbuf[2] = _tPid->fDbuf[1];
    _tPid->fDbuf[1] = _tPid->fDbuf[0];

    /* 以本次误差与上次误差的差值减去上次误差与上上次误差的差值作为微分项的输入带入计算 */
    _tPid->fDbuf[0] = (_tPid->fError[0] - 2.0f * _tPid->fError[1] + _tPid->fError[2]);
    _tPid->fDout = _tPid->fKd * _tPid->fDbuf[0];
    
    /* 结果处理 */
    Delt_Out = (int)(_tPid->fPout + _tPid->fIout + _tPid->fDout);    /* 计算增量输出 */
	
	//限幅
	if(Delt_Out > _tPid->fMax_Out)
		Delt_Out = _tPid->fMax_Out;
	if(Delt_Out < -_tPid->fMax_Out)
		Delt_Out = -_tPid->fMax_Out;
	
    _tPid->fCtrl_Out = _tPid->fPre_Out + Delt_Out;                 /* 与前值累加 */
    _tPid->fPre_Out = _tPid->fCtrl_Out;                            /* 记录数值，为下次PID做准备 */


    return (int)_tPid->fCtrl_Out;
}

/**
 * @brief PID位置计算函数
 * @param _tPid-PID结构体指针
 * @param _fCurrValue-当前值
 * @param _fExpValue-期望值
 * @retval int 经过PID计算后的输出值
*/
int PID_Location_Calculate(tagPID_T *_tPid, float _fCurrValue, float _fExpValue)
{
    /* 设定期望值和当前值 */
    _tPid->fExp_Value  = _fExpValue;
    _tPid->fCurr_Value = _fCurrValue;
    
    /* 存放过去两次误差值 */
    _tPid->fError[2] = _tPid->fError[1];
    _tPid->fError[1] = _tPid->fError[0];
    _tPid->fError[0] = _fExpValue - _fCurrValue;
    
    /* 采用位置式PID */
    
    /* 计算P项的值 */
    _tPid->fPout = _tPid->fKp * _tPid->fError[0];
    
    /* 计算D项的值（添加低通滤波） */
    float raw_dout = _tPid->fKd * (_tPid->fError[0] - _tPid->fError[1]);  // 原始D项
    _tPid->fDout = _tPid->alpha * raw_dout + (1.0f - _tPid->alpha) * _tPid->fDout;  // 低通滤波
    
    /* 计算I项的值 */
    _tPid->fIout += _tPid->fKi * _tPid->fError[0];
    
    // 积分限幅
    if(_tPid->fIout > _tPid->fMax_Iout)
        _tPid->fIout = _tPid->fMax_Iout;
    if(_tPid->fIout < -_tPid->fMax_Iout)  // 补充负向限幅
        _tPid->fIout = -_tPid->fMax_Iout;
    
    // 计算输出
    _tPid->fCtrl_Out = _tPid->fPout + _tPid->fIout + _tPid->fDout;
    
    // 输出限幅
    if(_tPid->fCtrl_Out > _tPid->fMax_Out)
        _tPid->fCtrl_Out = _tPid->fMax_Out;
    if(_tPid->fCtrl_Out < -_tPid->fMax_Out)  // 补充负向限幅
        _tPid->fCtrl_Out = -_tPid->fMax_Out;
    
    return (int)_tPid->fCtrl_Out;
}








/**
 * @brief PID历史储存数据清空函数
 * @param _tPid-PID结构体指针
 * @retval Null
*/
void PID_Clear(tagPID_T *_tPid)
{
	/* 当前误差清零 */
    _tPid->fError[0] = _tPid->fError[1] = _tPid->fError[2] = 0.0f;

    /* 微分项清零 */
    _tPid->fDbuf[0] = _tPid->fDbuf[1] = _tPid->fDbuf[2] = 0.0f;

    /* 输出清零 */
    _tPid->fCtrl_Out = _tPid->fPout = _tPid->fIout = _tPid->fDout = 0.0f;

    /* 目标值和当前值清零 */
    _tPid->fCurr_Value = _tPid->fExp_Value = _tPid->fPre_Out =0.0f;
	
}

