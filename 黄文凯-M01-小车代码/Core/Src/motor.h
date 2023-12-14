#ifndef __MOTOR_H
#define __MOTOR_H

#include "stdint.h"
#include "PID.h"

#define Car_H 0.5f//Ç°ºóÂóÂÖµÄ¾àÀë
#define Car_W 0.45f//×óÓÒÂóÂÖµÄ¾àÀë

typedef struct
{
	int32_t loop,count;
	float V_in,out_speed,pwmval;
	float target ;
	int16_t Get_data;
	
}motor_data;


void chasis_move_resol(float Vx,float Vy,float Vz);

void move_speedresol(float Vx,float Vy,float Vz);

extern float PID_realize(PID *pid,float actual_val,float target_val);

void motor_set(motor_data *motor,int j);

 void motor_init(void);

void motor_data_resol(motor_data *motor,int i);

#endif
