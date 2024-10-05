#ifndef __PID_H
#define __PID_H

typedef struct PID {
		float  Kp;         //  Proportional Const  P系数
		float  Ki;           //  Integral Const      I系数
		float  Kd;         //  Derivative Const    D系数
		
		float  PrevError ;          //  Error[-2]  
		float  LastError;          //  Error[-1]  
		float  Error;              //  Error[0 ]  
		float  DError;            //pid->Error - pid->LastError	
		float  SumError;           //  Sums of Errors  
		
		float  output;
		
		float  Integralmax;      //积分项的最大值
		float  outputmax;        //输出项的最大值
} PID;

float abs_limit(float value, float ABS_MAX);
float PID_Position_Calc(PID *pid, float Target_val, float Actual_val);
float PID_Incremental_Calc(PID *pid, float Target_val, float Actual_val);
void PID_Init(PID *pid, float Kp , float Ki , float Kd , float Limit_value);

#endif
