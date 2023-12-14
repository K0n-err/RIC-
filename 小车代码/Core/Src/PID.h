
#ifndef __PID_H
#define __PID_H
typedef struct
{
	float err;          //偏差值
	float err_last;     //上一个偏差值
	float Kp,Ki,Kd;     //比例、积分、微分系数
	float integral;     //积分值
	float output_val;   //输出值
	float kp_out;		//比例项输出
	float ki_out;		//积分项输出
	float kd_out;		//微分项输出
	
}PID;

extern PID pid[3];

void PID_param_init(PID *pid);
float PID_realize(PID *pid,float actual_val,float target_val);

#endif
