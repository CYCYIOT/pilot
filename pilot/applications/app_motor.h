#ifndef _APP_MOTOR_H_
#define _APP_MOTOR_H_

#include "lib_math.h"

#define MAX_NUM_MOTORS 8

#define MOTORS_MOT_0  		0
#define MOTORS_MOT_1  		1
#define MOTORS_MOT_2  		2
#define MOTORS_MOT_3  		3
#define MOTORS_MOT_4  		4
#define MOTORS_MOT_5  		5
#define MOTORS_MOT_6  		6
#define MOTORS_MOT_7  		7

#define MOTORS_CW   -1
#define MOTORS_CCW   1

#define MOTORS_AXIS_4	4
#define MOTORS_AXIS_6	6
#define MOTORS_AXIS_8	8

#define MOTORS_AXIS_4_PLUS	0
#define MOTORS_AXIS_4_X		1

#define MOTORS_AXIS_6_PLUS	0
#define MOTORS_AXIS_6_X		1
#define MOTORS_AXIS_6_Y		2

#define MOTORS_AXIS_8_PLUS		0
#define MOTORS_AXIS_8_X			1
#define MOTORS_AXIS_8_CIRCLE	2

void motor_param_init(void);
void motor_init(void);
void motor_update(float dt);
void motor_control_get(float * control);
void motor_output_get(float * output);
void motor_control_set_roll(float roll);
void motor_control_set_pitch(float pitch);
void motor_control_set_yaw(float yaw);
void motor_control_set_thr(float thr);
void motor_set_test(bool need_disarmed,float time,float val[MAX_NUM_MOTORS]);
void motor_test_takeoff();
float motor_control_get_thr();

#endif
