#include "robot_arm.h"
#include "tim.h"
#include "controller.h"

servo_data servo[3];//�����������


void robot_arm_init()//��е�۵Ķ����ʼ��
{
	HAL_TIM_Base_Start(&htim2);//���PWM����
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);//���1
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_2);//���2
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_3);//���3
}



extern int stop,start,stop_1,start_1;//����λ�ı�־λ


void servo_1_set(servo_data *servo)//�����������
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
		__HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_1,pwmVal);//����λ
	if(start == 1)
		__HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_1,195);
	last_temp = temp;

}


void servo_2_set(servo_data *servo)//�м�������
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


void servo_3_set(servo_data *servo)//צ�Ӷ������
{
	float pwmVal;
	pwmVal=servo->angle;
	__HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_3,pwmVal);

}
