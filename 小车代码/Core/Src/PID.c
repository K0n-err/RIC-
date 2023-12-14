
#include "PID.h"

void PID_param_init(PID *pid)
{
    /* 初始化参数 */
    pid->err=0.0;
    pid->err_last=0.0;
    pid->integral=0.0;		

    pid->Kp=1.0;	
    pid->Ki=0.9;		
    pid->Kd=0.02;				  
	
	
	pid->kp_out=0;
	pid->ki_out=0;
	pid->kd_out=0;
}

float PID_realize(PID *pid,float actual_val,float target_val)
{
			

	/*计算目标值与实际值的误差*/
	pid->err = target_val - actual_val;
	
	/*积分项*/
	pid->integral += pid->err;
	
	/*积分项求和输出*/
	pid->ki_out=pid->Ki * pid->integral;
	
	/*积分项求和限幅度*/
	if(pid->ki_out>1000)
		pid->ki_out=1000;
	if(pid->ki_out<-1000)
		pid->ki_out=-1000;
	
	/*比例项输出*/
	pid->kp_out=pid->Kp * pid->err;
	
	/*微分项输出*/
	pid->kd_out=pid->Kd * (pid->err - pid->err_last);
	
	//PID输出
	pid->output_val = pid->kp_out+ 
				      pid->ki_out+ 
				      pid->kd_out;
	/*误差传递*/
	pid->err_last = pid->err;
	
	//输出限幅
	if(pid->output_val <(-1000))
		pid->output_val =(-1000);
	if(pid->output_val >1000)
		pid->output_val =1000;
	if(target_val==0)
		pid->output_val =0;
	/*返回当前实际值*/
	return pid->output_val;
}
