#include <string.h>

#include "hal.h"

#include "app_param.h"
#include "app_system.h"
#include "app_motor.h"
#include "app_debug.h"

#define DEBUG_ID DEBUG_ID_MOTOR

float motor_control[4];
float motor_map[MAX_NUM_MOTORS];
float r_f[MAX_NUM_MOTORS]; 
float p_f[MAX_NUM_MOTORS]; 
float y_f[MAX_NUM_MOTORS];
float motor_out[MAX_NUM_MOTORS];
uint8_t motor_num;

float motor_rp_max;
float motor_yaw_max;
float motor_thr_max;
float motor_out_max;
float motor_out_min;

float motor_test_time;
float motor_test_timeout;
float motor_test[MAX_NUM_MOTORS];
//int mo_flag=0;
void add_motor_normal(uint8_t motor_num,float angle_degrees,float yaw_factor)
{
	float roll_fac,pitch_fac,yaw_fac;
	roll_fac = cosf(radians(angle_degrees + 90));
	pitch_fac = cosf(radians(angle_degrees));
	yaw_fac = yaw_factor;
	r_f[motor_num] = roll_fac;
	p_f[motor_num] = pitch_fac;
	y_f[motor_num] = yaw_fac;
}

void set_motors(uint8_t motor_calss,uint8_t frame_type)
{	
	switch(motor_calss){
		case MOTORS_AXIS_4:
			motor_num = 4;
			switch(frame_type){
				case MOTORS_AXIS_4_PLUS:					
                    add_motor_normal(motor_map[0],   0, MOTORS_CW);
                    add_motor_normal(motor_map[1],  90, MOTORS_CCW);
                   	add_motor_normal(motor_map[2], 180, MOTORS_CW);
                    add_motor_normal(motor_map[3], -90, MOTORS_CCW);
					break;
				case MOTORS_AXIS_4_X:
					add_motor_normal(motor_map[0], -45,  MOTORS_CW);
		            add_motor_normal(motor_map[1],  45,  MOTORS_CCW);
		            add_motor_normal(motor_map[2],  135, MOTORS_CW);
		            add_motor_normal(motor_map[3], -135, MOTORS_CCW);
                   	break;
				default:
					break;
			}
			break;
		case MOTORS_AXIS_6:
			motor_num = 6;
			switch(frame_type){
				case MOTORS_AXIS_6_PLUS:					
					add_motor_normal(motor_map[0], 0,  MOTORS_CW);
					add_motor_normal(motor_map[1], 180,MOTORS_CCW);
					add_motor_normal(motor_map[2],-120,MOTORS_CW);
					add_motor_normal(motor_map[3], 60, MOTORS_CCW);
					add_motor_normal(motor_map[4], -60,MOTORS_CCW);
					add_motor_normal(motor_map[5], 120,MOTORS_CW);
					break;				
                case MOTORS_AXIS_6_X:
                    add_motor_normal(motor_map[0],  90, MOTORS_CW);
                    add_motor_normal(motor_map[1], -90, MOTORS_CCW);
                    add_motor_normal(motor_map[2], -30, MOTORS_CW);
                    add_motor_normal(motor_map[3], 150, MOTORS_CCW);
                    add_motor_normal(motor_map[4],  30, MOTORS_CCW);
                    add_motor_normal(motor_map[5],-150, MOTORS_CW);
                    break;
				default:
					break;
			}
		case MOTORS_AXIS_8:
			motor_num = 8;
			switch(frame_type){
				case MOTORS_AXIS_8_PLUS:						
					add_motor_normal(motor_map[0],	  0,  MOTORS_CW);
					add_motor_normal(motor_map[1],	180,  MOTORS_CW);
					add_motor_normal(motor_map[2],	 45,  MOTORS_CCW);
					add_motor_normal(motor_map[3],	135,  MOTORS_CCW);
					add_motor_normal(motor_map[4],	-45,  MOTORS_CCW);
					add_motor_normal(motor_map[5],  -135,  MOTORS_CCW);
					add_motor_normal(motor_map[6],	-90,  MOTORS_CW);
					add_motor_normal(motor_map[7],	 90,  MOTORS_CW);
					break;
				case MOTORS_AXIS_8_X:						
					add_motor_normal(motor_map[0],	22.5f, MOTORS_CW);
					add_motor_normal(motor_map[1], -157.5f, MOTORS_CW);
					add_motor_normal(motor_map[2],   67.5f, MOTORS_CCW);
					add_motor_normal(motor_map[3],  157.5f, MOTORS_CCW);
					add_motor_normal(motor_map[4],  -22.5f, MOTORS_CCW);
					add_motor_normal(motor_map[5], -112.5f, MOTORS_CCW);
					add_motor_normal(motor_map[6],  -67.5f, MOTORS_CW);
					add_motor_normal(motor_map[7],  112.5f, MOTORS_CW);
					break;
				default:
					break;
			}	
			break;			
		default:
			break;
	}
	
}

void motor_set_test(bool need_disarmed,float time,float val[MAX_NUM_MOTORS])
{
	uint8_t count;

	if(need_disarmed == true && system_get_armed() == true){
		INFO(DEBUG_ID,"motor test failed,cann't start when armed");
	}else{
		motor_test_timeout = time;
		motor_test_time = 0.0f;	
		
		for(count = 0 ; count < MAX_NUM_MOTORS ; count++){
			motor_test[(uint8_t)motor_map[count]] = val[count];
		}
	}
}

bool motor_disarmed(void)
{
	float motor_out[MAX_NUM_MOTORS] = {0};

	motor_control[0] = 0.0f;
	motor_control[1] = 0.0f;
	motor_control[2] = 0.0f;
	motor_control[3] = 0.0f;

	return hal_set_motor(motor_out,motor_num);
}

void motor_param_init(void)
{
	param_set_var(MOTOR__MAP_0_NAME		,&motor_map[0]);
	param_set_var(MOTOR__MAP_1_NAME		,&motor_map[1]);
	param_set_var(MOTOR__MAP_2_NAME		,&motor_map[2]);
	param_set_var(MOTOR__MAP_3_NAME		,&motor_map[3]);
	param_set_var(MOTOR__MAP_4_NAME		,&motor_map[4]);
	param_set_var(MOTOR__MAP_5_NAME		,&motor_map[5]);
	param_set_var(MOTOR__MAP_6_NAME		,&motor_map[6]);
	param_set_var(MOTOR__MAP_7_NAME		,&motor_map[7]);

	param_set_var(MOTOR__RP_MAX_NAME	,&motor_rp_max);
	param_set_var(MOTOR__YAW_MAX_NAME	,&motor_yaw_max);
	param_set_var(MOTOR__THR_MAX_NAME	,&motor_thr_max);
	
	param_set_var(MOTOR__OUT_MAX_NAME	,&motor_out_max);
	param_set_var(MOTOR__OUT_MIN_NAME	,&motor_out_min);
}

void motor_init(void)
{
	uint8_t i;
	
	motor_test_timeout = 0.0f;
	
	for(i = 0;i < MAX_NUM_MOTORS;i++){
		if(motor_map[i] < 0 || motor_map[i] >= MAX_NUM_MOTORS){
			ERR(DEBUG_ID,"motor map check failed");
		}else{
			DEBUG(DEBUG_ID,"motor map[%d]:%d",i,(uint8_t)motor_map[i]);
		}
	}
	
	set_motors(MOTORS_AXIS_4,MOTORS_AXIS_4_X);
	motor_disarmed();                                 //µç»ú½âËø

	INFO(DEBUG_ID,"init");
}

void motor_control_get(float * control)
{
	control[0] = motor_control[0];
	control[1] = motor_control[1];
	control[2] = motor_control[2];
	control[3] = motor_control[3];
}

void motor_output_get(float * output)
{
	uint8_t count;
	for(count = 0 ; count < MAX_NUM_MOTORS; count++){
		output[count] = motor_out[count];
	}
}

void motor_control_set_roll(float roll)
{
	motor_control[0] = constrain_float(roll,-motor_rp_max,motor_rp_max);
}

void motor_control_set_pitch(float pitch)
{
	motor_control[1] = constrain_float(pitch,-motor_rp_max,motor_rp_max);
}

void motor_control_set_yaw(float yaw)
{
	motor_control[2] = constrain_float(yaw,-motor_yaw_max,motor_yaw_max);
}

void motor_control_set_thr(float thr)
{
	motor_control[3] = constrain_float(thr,0.0f,motor_thr_max);
}

float motor_control_get_thr()
{
	return motor_control[3];
}

void motor_test_takeoff()
{
   float motor[8];
   int count;
   for(count = 0 ; count < 8 ;count++)
		motor[count] = 0.20;
		
   motor_set_test(false,0.01f,motor);
   hal_set_motor(motor_test,motor_num);
	  	
}
/******  Defualt Order of motors    ******

                  0      1
                    \   /
                      ^
                    /   \
                  3      2

**************************************/
void motor_update(float dt)
{
	uint8_t i;
	float motor_max = 0.0f;
	
	//float motor[8];
	//int count;
	
	for(i = 0 ; i < motor_num ; i++){
		motor_out[i] = motor_control[0] * r_f[i] + motor_control[1] * p_f[i] + motor_control[2] * y_f[i] + motor_control[3];
		if(motor_out[i] < 0.0f){
			motor_out[i] = 0.0f;
		}
		if(motor_out[i] > motor_max){
			motor_max = motor_out[i];
		}
	}
	
	if(motor_max > 1.0f){
		for(i = 0;i < motor_num;i++){
			motor_out[i] /= motor_max;
		}	
	}
	
	for(i = 0;i < motor_num;i++){
		motor_out[i] = constrain_float(motor_out[i],motor_out_min,motor_out_max);
	}
#if 0
	  if(mo_flag < 5000)
	  	{
		for(count = 0 ; count < 8 ;count++){
			motor[count] = 0.05;
		}
		motor_set_test(false,0.01f,motor);
		hal_set_motor(motor_test,motor_num);
		mo_flag++;
	  	}
#endif
	if(motor_test_timeout > 0.0f){
		motor_test_time += dt;
		if(motor_test_time >= motor_test_timeout){
			motor_test_timeout = 0.0f;
		}		
		hal_set_motor(motor_test,motor_num);
	}else{
	     
		if(system_get_armed() == true){
	       hal_set_motor(motor_out,motor_num);
        //   for(i = 0;i<motor_num;i++)
		 //  debug_t("motor[%d] = %f\n",i,motor_out[i]);
			
		}else{
			motor_disarmed();
		}
	}
	
	DEBUG_HZ(DEBUG_ID,5,dt,"control(%3.3f,%3.3f,%3.3f,%3.3f) out(%3.3f,%3.3f,%3.3f,%3.3f)",motor_control[0],motor_control[1],motor_control[2],motor_control[3],motor_out[0],motor_out[1],motor_out[2],motor_out[3]);
}


