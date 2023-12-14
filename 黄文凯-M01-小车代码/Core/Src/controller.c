#include "controller.h"
#include "stm32f1xx_it.h"
#include "stdio.h"
#include "motor.h"
#include "PID.h"
#include "robot_arm.h"

handle_data xbox_t = {0};

extern servo_data servo[3];
extern motor_data motor[4];

// Error flag
uint8_t err;
// The length of one frame of data received
volatile uint8_t rx_len = 0;
// A frame of data reception completion flag
volatile uint8_t recv_end_flag = 0;

// Define the serial port receiving buffer
uint8_t rx_buffer[BUF_SIZE] = {0};


void Data_Resolve(handle_data *ptr);
void Print_Controller_Data(handle_data *ptr);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);


// An enumeration that defines the state of the button
enum ButtonState getButtonState(uint8_t data, uint8_t mask)
{
	return (data & mask) ? BUTTON_PRESSED : BUTTON_NOT_PRESSED;
}

void Data_Resolve(handle_data *ptr)
{
	// Count
	uint8_t temp0 = 0;

	// Takes the cached array address
	uint8_t *rx_p = rx_buffer;

	// Find header 0xA5 with loop optimization
	while (*rx_p != 0xA5 && temp0 <= 19)
	{
		temp0++;
		rx_p++;
	}

	// Error flag
	err = 0;

	// Determine the frame header and end of the frame
	if (rx_p[0] == 0xA5 && rx_p[17] == 0xA6)
	{
		// Horizontal stroke of the left stick
		ptr->L_Joystick_Horizon = rx_p[1] | (rx_p[2] << 8);
		// The vertical travel of the left stick
		ptr->L_Joystick_Vertical = rx_p[3] | (rx_p[4] << 8);
		// Horizontal stroke of the right stick
		ptr->R_Joystick_Horizon = rx_p[5] | (rx_p[6] << 8);
		// The vertical stroke of the right stick
		ptr->R_Joystick_Vertical = rx_p[7] | (rx_p[8] << 8);
		// Left trigger travel
		ptr->L_Trigger = rx_p[9] | (rx_p[10] << 8);
		// Right trigger stroke
		ptr->R_Trigger = rx_p[11] | (rx_p[12] << 8);
		// Joystick input combination
		ptr->combination = rx_p[13];
		// Buttons
		ptr->A = getButtonState(rx_p[14], 0x01);  // 设置按钮 A 的状态
		ptr->B = getButtonState(rx_p[14], 0x02);  // 设置按钮 B 的状态
		ptr->X = getButtonState(rx_p[14], 0x08);  // 设置按钮 X 的状态
		ptr->Y = getButtonState(rx_p[14], 0x10);  // 设置按钮 Y 的状态
		ptr->LB = getButtonState(rx_p[14], 0x40);  // 设置按钮 LB 的状态
		ptr->RB = getButtonState(rx_p[14], 0x80);  // 设置按钮 RB 的状态
		ptr->View = getButtonState(rx_p[15], 0x04);  // 设置按钮 View 的状态
		ptr->Menu = getButtonState(rx_p[15], 0x08);  // 设置按钮 Menu 的状态
		ptr->Xbox = getButtonState(rx_p[15], 0x10);  // 设置按钮 Xbox 的状态
		ptr->press_L = getButtonState(rx_p[15], 0x20);  // 设置按钮 press_L 的状态
		ptr->press_R = getButtonState(rx_p[15], 0x40); // 设置按钮 press_R 的状态
		ptr->Share = getButtonState(rx_p[16], 0x01); // 设置按钮 Share 的状态
	}
	else
	{
		err = 1;
		// Modify to error handling
	}
}


int a = 0,see_b = 0,see_a = 0,flag_3;
void Rc_To_chassis()	//遥控器数据传入到底盘控制函数中
{
	float L_Joystick_shuiping = 0,L_Joystick_chuizhi = 0,R_Joystick_zixuan = 0;
	
	//对遥控器数据进行转化，0 到 655535 映射为 -220 到 220
	L_Joystick_chuizhi = handle_verticaldata_trans(xbox_t.L_Joystick_Vertical);
	R_Joystick_zixuan = handle_spindata_trans(xbox_t.R_Joystick_Horizon);
	L_Joystick_shuiping = handle_horizondata_trans(xbox_t.L_Joystick_Horizon);
	
	//控制前进后退，左右平移
	if(L_Joystick_chuizhi != 0|| L_Joystick_shuiping != 0)
	{
		
		see_b =handle_verticaldata_trans(xbox_t.L_Joystick_Vertical);
		see_a  = handle_horizondata_trans(xbox_t.L_Joystick_Horizon);
		move_speedresol(see_b,see_a,0);
	
	}

	//控制自旋
	else if(R_Joystick_zixuan  )
	{
		
		
		a = R_Joystick_zixuan;

		move_speedresol(0,0,a);
		
	}
	
	//进入摇杆死区时，速度为0
	else if(L_Joystick_chuizhi==0&&R_Joystick_zixuan==0&&L_Joystick_shuiping==0)
	{
		for(int i = 0;i<=3;i++)
			motor[i].target = 0;
		
		move_speedresol(0,0,0);
	}
}

float stop,start,stop_1,start_1;//防复位的标志位

void Rc_To_robotarm()//遥控器数据传入机械臂控制函数
{
	float servo_1,servo_2,servo_3;
	
	servo_1 = handle_robotarm_trans(xbox_t.R_Trigger);
	servo_2 = handle_robotarm_trans(xbox_t.L_Trigger);
	
	
	if(servo_1)//底座舵机
	{
		servo[0].angle = servo_1;
		servo_1_set(&servo[0]);
	}
	if(servo_2)//第二个舵机
	{
		servo[1].angle = servo_2;
		servo_2_set(&servo[1]);
	}
	
	if(xbox_t.X)//防止第二个舵机复位
	{
		stop = 1;
		start = 0;
	}
	if(xbox_t.Y)//解锁第二个舵机，可以复位
	{
		start = 1;
		stop = 0;
	}
	
	switch(xbox_t.combination)
	{
		case 7:stop_1 = 1;start_1 = 0;break;//对第二个舵机进行锁定
		
		case 3:stop_1 = 0;start_1 = 1;break;//对第二个舵机进行解锁
		

	}
	
	if(xbox_t.B)//夹取
	{
		servo[2].angle = 195.0f;
		
		servo_3_set(&servo[2]);
	}
	if(xbox_t.A)//松开
	{
		servo[2].angle = 185.0f;
		
		servo_3_set(&servo[2]);
	}
}




/*手柄数据转化，即0-65535 映射到 0-220*/


int see_L;//方便debug观察数值
float handle_horizondata_trans(uint16_t L_joystick_in)//Vy轴
{
	float L_shuiping_data,temp;
	if(32100 < L_joystick_in && L_joystick_in<34000)
		temp = 0;
	else
		temp = (float)(L_joystick_in - 32768);
	L_shuiping_data = (temp/32768.0f)*220.0f;
	see_L = L_shuiping_data;
	return L_shuiping_data;
}

int see_H;
float handle_verticaldata_trans(uint16_t L_joystick_in)//Vx轴
{
	float L_chuizhi_data,temp;
	if(32100 < L_joystick_in && L_joystick_in<34000)//摇杆死区
		temp = 0;
	else
		temp = (float)(L_joystick_in - 32768);
	L_chuizhi_data = (temp/32768.0f)*220.0f;
	see_H = L_chuizhi_data;
	return see_H;
}

int see_zi;
float handle_spindata_trans(uint16_t L_joystick_in)//Vw轴
{
	float R_shuiping_data,temp;
	if(32100 < L_joystick_in && L_joystick_in<34000)
		temp = 0;
	else
		temp = (float)(L_joystick_in - 32768);
	R_shuiping_data = (temp/32768.0f)*220.0f;
	see_zi = R_shuiping_data;
	return see_zi;
}

/*手柄数据转化到舵机控制范围，即0-65535到0°- 180°*/

int see_arm;
float handle_robotarm_trans(uint16_t L_joystick_in)
{
	float RB_data,temp;
	if(L_joystick_in==0)
		temp = 0;
	else
		temp = (float)(L_joystick_in /1023.0f)*180.0f;
	RB_data = temp;
	see_arm = temp;
	return RB_data;
}