
#include "tim.h"
#include "gpio.h"
#include "motor.h"
#include "controller.h"
#include "usart.h"
#include "math.h"
#include "string.h"
#include "PID.h"
#include "ws2812b.h"
#include "robot_arm.h"

handle_data handle_d ;//手柄数据对象
motor_data motor[4];//四个电机对象

PID pid[3];//各个电机的pid

void motor_set(motor_data *motor,int j)//控制电机速度，方向
{
	
	switch (j)
	{
		case 0 :	
		{	
			if(motor->target>0)//正转
			{
				HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_SET);
				
				__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_1,motor->target*4.34f);
			}
			if(motor->target<0)//反转
			{
				HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,GPIO_PIN_SET);
				
				__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_1,-motor->target*4.34f);
			}
			if(motor->target==0)//停转
			{
				HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_RESET);
				
				__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_1,0);
			}
		}
		case 1 :	
		{	
			if(motor->target>0)//正转
			{
					HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,GPIO_PIN_RESET);
					HAL_GPIO_WritePin(GPIOC,GPIO_PIN_14,GPIO_PIN_SET);
				
				__HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_2,motor->target*4.34f);
			}
			if(motor->target<0)//反转
			{
					HAL_GPIO_WritePin(GPIOC,GPIO_PIN_14,GPIO_PIN_RESET);
					HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,GPIO_PIN_SET);
				
				__HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_2,-motor->target*4.34f);
			}
			if(motor->target==0)//停转
			{
				HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,GPIO_PIN_RESET);
					HAL_GPIO_WritePin(GPIOC,GPIO_PIN_14,GPIO_PIN_RESET);
				
				__HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_2,0);
			}
		}
		case 2 :	
		{	
			if(motor->target>0)//正转
			{
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_13,GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_SET);
				
				__HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_1,motor->target*4.34f);
			}
			if(motor->target<0)//反转
			{
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_13,GPIO_PIN_SET);
				
				__HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_1,-motor->target*4.34f);
			}
			if(motor->target==0)//停转
			{
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_13,GPIO_PIN_RESET);
				
				__HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_1,0);
			}
		}
		case 3 :	
		{	
			if(motor->target>0)//正转
			{
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,GPIO_PIN_SET);
				
				__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_4,motor->target*4.34f);
			}
			if(motor->target<0)//反转
			{
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,GPIO_PIN_SET);
				
				__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_4,-motor->target*4.34f);
			}
			if(motor->target==0)//停转
			{
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,GPIO_PIN_RESET);
				
				__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_4,0);
			}
		}
	}
}




 

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)//10ms中断
{
	if(htim->Instance == TIM3)
	{	//数据接收
		if (recv_end_flag == 1 && rx_len == DATA_REAL_LENGTH)
      {
        Data_Resolve(&xbox_t);

        rx_len = 0;
       
        recv_end_flag = 0;

      }
      
      HAL_UART_Receive_DMA(&c_huart, rx_buffer, BUF_SIZE);
		
	  Rc_To_chassis();//底盘控制
	  Rc_To_robotarm();//机械臂控制

  }
	
}



void motor_init(void)//电机初始化
{
	
	
	HAL_TIM_Base_Start(&htim1);//电机PWM开启
	HAL_TIM_Base_Start(&htim4);//电机PWM开启
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_4);
	__HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_1,0);
	__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_4,0);
	__HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_2,0);
	__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_1,0);
		for(int i = 0;i<=3;i++)
		{
			PID_param_init(&pid[i]);
			
		}

}


void move_speedresol(float Vx,float Vy,float Vz)//麦轮速度逆解算
{
		
		
		motor[0].target = (Vx+Vy-Vz*(Car_H/2.0f+Car_W/2.0f));
		motor[1].target = (Vx-Vy-Vz*(Car_H/2.0f+Car_W/2.0f));
		motor[2].target = (Vx+Vy+Vz*(Car_H/2.0f+Car_W/2.0f));
		motor[3].target = (Vx-Vy+Vz*(Car_H/2.0f+Car_W/2.0f));
		
		for(int i  = 0;i<=3;i++)
		{
			
			motor_set(&motor[i],i);//控制电机
		}

}
