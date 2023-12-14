#include "robot_arm.h"
#include "tim.h"
#include "controller.h"

servo_data servo[3];//三个舵机对象


void robot_arm_init()//机械臂的舵机初始化
{
	HAL_TIM_Base_Start(&htim2);//舵机PWM开启
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);//舵机1
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_2);//舵机2
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_3);//舵机3
}



extern int stop,start,stop_1,start_1;//防复位的标志位


void servo_1_set(servo_data *servo)//底座舵机控制
{
	static int last_temp,last_pwmVal;
	int temp;
	temp = servo->angle;
	
	float pwmVal;
	
	pwmVal=184.0f-(servo->angle/11.0f);
	
	last_pwmVal = pwmVal;
	
	if(temp<last_temp)
		pwmVal = last_pwmVal;

	
	if(stop == 0)
		__HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_1,pwmVal);//防复位
	if(start == 1)
		__HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_1,195);
	last_temp = temp;

}


void servo_2_set(servo_data *servo)//中间舵机控制
{
	static int last_temp,last_pwmVal;
	int temp;
	temp = servo->angle;
	
	float pwmVal;
	
	pwmVal=188.0f-(servo->angle/11.0f);
	
	last_pwmVal = pwmVal;
	if(temp<last_temp)
		pwmVal = last_pwmVal;

	
	if(stop_1 == 0)
		__HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_2,pwmVal);
	if(start_1 == 1)
		__HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_2,190);
	last_temp = temp;

}


void servo_3_set(servo_data *servo)//爪子舵机控制
{
	float pwmVal;
	pwmVal=servo->angle;
	__HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_3,pwmVal);

}
