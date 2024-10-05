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

float abs_limit(float value, float ABS_MAX)   //积分限幅，设置最大值。
{
	if(value > ABS_MAX)
		value = ABS_MAX;

	else if(value< -ABS_MAX)
		value = -ABS_MAX;
	return value;
}

float PID_Position_Calc(PID *pid, float Target_val, float Actual_val)  //位置式PID
{   

	pid->Error = Target_val - Actual_val;      //与pid P系数相乘。比例误差值 当前差值=目标值-实际值
	pid->SumError += pid->Error;                 //与pid I系数相乘。稳态误差值 误差相加作为误差总和，给积分项
	pid->DError = pid->Error - pid->LastError;   //与pid D系数相乘。 微分项-消除震荡

	pid->output =   pid->Kp* pid->Error +        
					abs_limit( pid->Ki* pid->SumError, pid->Integralmax ) +   
					pid->Kd* pid->DError ;  
								
   pid->LastError = pid->Error; //更新误差
   
  //限制输出最大值，防止出现突发意外。输出outputmax的最大值
	if(pid->output > pid->outputmax )  pid->output = pid->outputmax; 
	if(pid->output < - pid->outputmax )  pid->output = -pid->outputmax;
	
	return pid->output ;   //输出为pwm值
}


float PID_Incremental_Calc(PID *pid, float Target_val, float Actual_val)  
{  
	pid->Error = Target_val- Actual_val;                        

	pid->output  +=  pid->Kp* ( pid->Error - pid->LastError )+   
					 pid->Ki* pid->Error +   
					 pid->Kd* ( pid->Error +  pid->PrevError - 2*pid->LastError);  
								 
	pid->PrevError = pid->LastError;  
	pid->LastError = pid->Error;

	if(pid->output > pid->outputmax )    pid->output = pid->outputmax;
	else if(pid->output < - pid->outputmax )  pid->output = -pid->outputmax;
	return pid->output ;   //输出为pwm值 
}

//PID初始化
void PID_Init(PID *pid, float Kp , float Ki , float Kd , float Limit_value)  
{  
	pid->Kp= Kp;
	pid->Ki= Ki;
	pid->Kd= Kd;
	
	pid->PrevError =pid->LastError = pid->Error =pid->SumError= pid->output =  0; 
	pid->Integralmax = pid->outputmax  = Limit_value;
}  

