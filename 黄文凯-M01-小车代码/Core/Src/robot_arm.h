#ifndef __ROBOT_ARM_H
#define __ROBOT_ARM_H

void robot_arm_init(void);



typedef struct
{
	int angle;
	

}servo_data;

void servo_1_set(servo_data *servo);
void servo_2_set(servo_data *servo);
void servo_3_set(servo_data *servo);


#endif
