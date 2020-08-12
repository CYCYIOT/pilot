#include "app_param.h"
#include "app_rc.h"
#include "app_nav.h"
#include "app_system.h"
#include "app_debug.h"
#include "app_control.h"
#include "app_motor.h"
#include "app_control_common.h"
#include "app_att_control.h"
#include "app_pos_control.h"
#include "app_attitude.h"
#include "app_failsafe.h"
#include "app_rangefinder.h"
#include "awlink_item_control.h"

#include "lib_inav_rangefinder.h"
#include "lib_inav_baro.h"
#include "lib_inav_flow.h"
#include "lib_math.h"

#include "ahrs.h"

#define DEBUG_ID DEBUG_ID_CONTROL

#define FLIP_MODE_RISE		0
#define FLIP_MODE_BURST		1
#define FLIP_MODE_BREAK		2
#define FLIP_MODE_STABLE	3
#define FLIP_MODE_HOLD		4

#define FLIP_FRONT	0
#define FLIP_BACK	1
#define FLIP_LEFT	2
#define FLIP_RIGHT	3

#define FLIP_RISE_ALT_MAX 0.35f

float flip_alt_init;

uint8_t flip_mode;
uint8_t flip_direction = FLIP_FRONT;

float flip_hold_time;
float flip_hold_timeout;

float flip_stable_time;
float flip_stable_timeout;

float flip_rise_speed;
float flip_burst_rate_limit;
float flip_burst_att_limit;
float flip_burst_rate_check;
float flip_burst_att_check;
float flip_break_att;
float flip_hold_att;
float flip_offset_att[2];

float flip_break_gain;
float flip_stab_gain;
float flip_yaw_gain;

float flip_att_roll;
float flip_att_pitch;
float flip_att_yaw;

float flip_rate_limit;

float flip_pos[3];
float flip_rf_alt;

float flip_stable_att_target[2];

uint8_t control_flip_get_mode()
{
	return flip_mode;
}

float control_flip_stable_get_roll_target()
{
	return flip_stable_att_target[0];
}

float control_flip_stable_get_pitch_target()
{
	return flip_stable_att_target[1];
}

float control_flip_get_roll()
{
	return flip_att_roll;
}

float control_flip_get_pitch()
{
	return flip_att_pitch;
}

float control_flip_get_yaw()
{
	return flip_att_yaw;
}

bool control_flip_check()
{
	uint8_t check = CONTROL_CHECK_ATT | CONTROL_CHECK_ALT | CONTROL_CHECK_ARM;

	return control_check(check);
}

float control_flip_burst_get_rate(float rate[3])
{
	if(flip_direction == FLIP_FRONT){
		
		rate[1] = -flip_rate_limit;
		return (flip_burst_att_check + flip_att_pitch);
		
	}else if(flip_direction == FLIP_BACK){
	
		rate[1] = flip_rate_limit;
		return (flip_burst_att_check - flip_att_pitch);
		
	}else if(flip_direction == FLIP_LEFT){
	
		rate[0] = -flip_rate_limit;
		return (flip_burst_att_check + flip_att_roll);
		
	}else if(flip_direction == FLIP_RIGHT){
	
		rate[0] = flip_rate_limit;
		return (flip_burst_att_check - flip_att_roll);
	}

	return 0;
}

float control_flip_break_get_rate(float rate[3])
{
	if(flip_direction == FLIP_FRONT){
		
		rate[0] = -attitude_get_rate_roll() * flip_stab_gain;
		rate[1] = (-flip_break_att - flip_att_pitch) * flip_break_gain;
		rate[2] = -flip_att_yaw * flip_yaw_gain;
		return (-flip_break_att - flip_att_pitch);
		
	}else if(flip_direction == FLIP_BACK){
	
		rate[0] = -attitude_get_rate_roll() * flip_stab_gain;
		rate[1] = (flip_break_att - flip_att_pitch) * flip_break_gain;
		rate[2] = -flip_att_yaw * flip_yaw_gain;
		return (flip_break_att - flip_att_pitch);
		
	}else if(flip_direction == FLIP_LEFT){
	
		rate[0] = (-flip_break_att - flip_att_roll) * flip_break_gain;
		rate[1] = -attitude_get_rate_pitch() * flip_stab_gain;
		rate[2] = -flip_att_yaw * flip_yaw_gain;
		return (-flip_break_att - flip_att_roll);
		
	}else if(flip_direction == FLIP_RIGHT){
	
		rate[0] = (flip_break_att - flip_att_roll) * flip_break_gain;
		rate[1] = -attitude_get_rate_pitch() * flip_stab_gain;
		rate[2] = -flip_att_yaw * flip_yaw_gain;
		return (flip_break_att - flip_att_roll);
		
	}

	return 0;
}

float control_flip_stable_get_rate(float rate[3])
{
	if(flip_direction == FLIP_FRONT){

		flip_stable_att_target[0] = flip_offset_att[0] - attitude_get_att_roll();
		flip_stable_att_target[1] = flip_offset_att[1] - flip_hold_att - attitude_get_att_pitch();
		
		rate[0] = flip_stable_att_target[0] * flip_stab_gain;
		rate[1] = flip_stable_att_target[1] * flip_break_gain;
		
		return flip_stable_att_target[1];
               
	}else if(flip_direction == FLIP_BACK){

		flip_stable_att_target[0] = flip_offset_att[0] - attitude_get_att_roll();
		flip_stable_att_target[1] = flip_offset_att[1] + flip_hold_att - attitude_get_att_pitch();
	
		rate[0] = flip_stable_att_target[0] * flip_stab_gain;
		rate[1] = flip_stable_att_target[1] * flip_break_gain;
		
		return flip_stable_att_target[1];
               
	}else if(flip_direction == FLIP_LEFT){

		flip_stable_att_target[0] = flip_offset_att[0] - flip_hold_att - attitude_get_att_roll();
		flip_stable_att_target[1] = flip_offset_att[1] - attitude_get_att_pitch();
		
		rate[0] = flip_stable_att_target[0] * flip_break_gain;
		rate[1] = flip_stable_att_target[1] * flip_stab_gain;
		
		return flip_stable_att_target[0];
               
	}else if(flip_direction == FLIP_RIGHT){

		flip_stable_att_target[0] = flip_offset_att[0] + flip_hold_att - attitude_get_att_roll();
		flip_stable_att_target[1] = flip_offset_att[1] - attitude_get_att_pitch();
		
		rate[0] = flip_stable_att_target[0] * flip_break_gain;
		rate[1] = flip_stable_att_target[1] * flip_stab_gain;
		
		return flip_stable_att_target[0];
               
	}

	return 0;
}

void control_flip_param_init()
{
	param_set_var(CON__FLIP__RISE_SPEED_NAME		,&flip_rise_speed);
	
	param_set_var(CON__FLIP__BURST_ATT_LIMIT_NAME	,&flip_burst_att_limit);
	param_set_var(CON__FLIP__BURST_RATE_LIMIT_NAME	,&flip_burst_rate_limit);
	param_set_var(CON__FLIP__BURST_ATT_CHECK_NAME	,&flip_burst_att_check);
	param_set_var(CON__FLIP__BURST_RATE_CHECK_NAME	,&flip_burst_rate_check);
	
	param_set_var(CON__FLIP__BREAK_ATT_NAME			,&flip_break_att);
	param_set_var(CON__FLIP__HOLD_ATT_NAME			,&flip_hold_att);
	
	param_set_var(CON__FLIP__BREAK_GAIN_NAME		,&flip_break_gain);
	param_set_var(CON__FLIP__STAB_GAIN_NAME			,&flip_stab_gain);
	param_set_var(CON__FLIP__YAW_GAIN_NAME			,&flip_yaw_gain);
	
	param_set_var(CON__FLIP__HOLD_TIMEOUT_NAME		,&flip_hold_timeout);
	param_set_var(CON__FLIP__STABLE_TIMEOUT_NAME	,&flip_stable_timeout);
}

void control_flip_exit()
{
	failsafe_set_att_limit_enable(true);
	failsafe_set_collision_enable(true);
    failsafe_set_collision_file_enable(false);

	inav_flow_set_normal_mode();
	inav_baro_set_noise_enable(true);
	inav_baro_set_normal_mode();
	inav_rf_set_normal_mode();
	
	ahrs_set_flip_mode(false);
	
	att_control_set_att_pid_normal_mode();
	pos_control_set_alt_pid_normal_mode();
	pos_control_set_vel_pid_normal_mode();
	set_file_flag();
	INFO(DEBUG_ID,"flip exit");
}

void control_flip_init(float param1,float param2)
{
	flip_mode = FLIP_MODE_RISE;

	flip_direction = (uint8_t)param1;

	flip_att_roll = 0.0f;
	flip_att_pitch = 0.0f;
	flip_att_yaw = 0.0f;

	flip_hold_time = 0.0f;
	flip_stable_time = 0.0f;
	
	nav_get_att_offset(flip_offset_att);
	flip_alt_init = nav_get_pos_ned_z();

	pos_control_set_alt_pid_flip_mode();
	failsafe_set_att_limit_enable(false);
	failsafe_set_collision_enable(false);

	INFO(DEBUG_ID,"flip init:%d",flip_direction);
}

void control_flip_rise_update(float dt,rc_s * rc)
{
	float vel[3] = {0};
	
	control_vel_ned_xy_common(dt,vel,0.5f,0.1f);
	control_alt_vel_common(dt,2.0f,2.0f);
	control_yaw_common_rc(dt,rc);
	if((nav_get_pos_ned_z() - flip_alt_init) >= FLIP_RISE_ALT_MAX){
		flip_mode = FLIP_MODE_HOLD;
	}

	if(nav_get_vel_ned_z() >= flip_rise_speed){
		motor_control_set_roll(0.0f);
		motor_control_set_pitch(0.0f);
		motor_control_set_yaw(0.0f);
		motor_control_set_thr(0.0f);
		inav_baro_set_noise_enable(false);
		inav_baro_set_flip_mode();
		inav_rf_set_flip_mode();
		inav_flow_set_flip1_mode();
		ahrs_set_flip_mode(true);
		att_control_set_att_pid_flip_mode();
		flip_mode = FLIP_MODE_BURST;
		INFO(DEBUG_ID,"flip burst");
	}
}

void control_flip_burst_update(float dt,rc_s * rc)
{
	float rate_target[3] = {0};
	float att_diff;

	att_diff = control_flip_burst_get_rate(rate_target);
	control_att_rate_common(dt,rate_target[0],rate_target[1],flip_rate_limit);

	if(flip_direction == FLIP_RIGHT || flip_direction == FLIP_LEFT){
		if(fabs(attitude_get_rate_roll()) > flip_burst_rate_limit){
			motor_control_set_roll(0.0f);
		}else{
			if(flip_direction == FLIP_RIGHT){
				motor_control_set_roll(2.0f);
			}else{
				motor_control_set_roll(-2.0f);
			}
		}
		motor_control_set_pitch(0.0f);
		
		if(att_diff < 0.0f || fabs(attitude_get_rate_roll()) >= flip_burst_rate_check){
			att_control_set_att_pid_error_clear();
			flip_rate_limit = fabs(attitude_get_rate_roll());
			flip_mode = FLIP_MODE_BREAK;
			INFO(DEBUG_ID,"flip break:(%3.3f,%3.3f)",flip_att_roll,flip_att_pitch);
		}
	}else{
		if(fabs(attitude_get_rate_pitch()) > flip_burst_rate_limit){
			motor_control_set_pitch(0.0f);
		}else{
			if(flip_direction == FLIP_BACK){
				motor_control_set_pitch(2.0f);
			}else{
				motor_control_set_pitch(-2.0f);
			}
		}
		motor_control_set_roll(0.0f);
		
		if(att_diff < 0.0f || fabs(attitude_get_rate_pitch()) >= flip_burst_rate_check){
			att_control_set_att_pid_error_clear();
			flip_rate_limit = fabs(attitude_get_rate_pitch());
			flip_mode = FLIP_MODE_BREAK;
			INFO(DEBUG_ID,"flip break:(%3.3f,%3.3f)",flip_att_roll,flip_att_pitch);
		}
	}
}

void control_flip_break_update(float dt,rc_s * rc)
{
	float rate_target[3] = {0};
	float att_diff;

	att_diff = control_flip_break_get_rate(rate_target);
	control_att_rate_common(dt,rate_target[0],rate_target[1],flip_rate_limit);
	control_yaw_rate_common(dt,rate_target[2],0.0f);
	
	if(fabs(att_diff) < 40.0f){
		flip_rf_alt = rangefinder_get_range_f();
		flip_pos[2] = nav_get_pos_ned_z();
		flip_mode = FLIP_MODE_STABLE;
		INFO(DEBUG_ID,"flip stable:(%3.3f,%3.3f)",flip_att_roll,flip_att_pitch);
	}
}

void control_flip_stable_update(float dt,rc_s * rc)
{
	float rate_target[3] = {0};
	float att_diff;

	att_diff = control_flip_stable_get_rate(rate_target);
	control_att_rate_common(dt,rate_target[0],rate_target[1],flip_rate_limit);

	flip_pos[2] = complementary_filter(flip_pos[2],nav_get_pos_ned_z(),0.996f);
	flip_rf_alt = complementary_filter(flip_rf_alt,rangefinder_get_range_f(),0.994f);
	control_set_rf_alt_target(flip_rf_alt);
		
	control_alt_pos_common(dt,flip_pos[2],0.2f);
	//control_alt_vel_common(dt,0.0f,0.1f);
	
	control_yaw_common_rc(dt,rc);
	
	if(fabs(att_diff) < 5.0f){
		flip_stable_time += dt;
		if(flip_stable_time >= flip_stable_timeout){
			failsafe_set_att_limit_enable(true);
			failsafe_set_collision_enable(true);
			failsafe_set_collision_file_enable(false);
			
			att_control_set_att_pid_error_clear();
			att_control_set_att_pid_normal_mode();
			pos_control_set_vel_pid_flip_mode();
			
			inav_flow_set_flip2_mode();
			
			ahrs_set_flip_mode(false);
			
			flip_mode = FLIP_MODE_HOLD;
			flip_pos[0] = nav_get_pos_ned_x();
			flip_pos[1] = nav_get_pos_ned_y();
			INFO(DEBUG_ID,"flip hold:(%3.3f,%3.3f)",flip_att_roll,flip_att_pitch);
		}
	}else{
		flip_stable_time = 0.0f;
	}
}

void control_flip_hold_update(float dt,rc_s * rc)
{
	float vel_target[3] = {0};
	
	//control_pos_ned_xy_common(dt,flip_pos,0,0.2f);
	control_vel_ned_xy_common(dt,vel_target,0,0.2f);

	flip_pos[2] = complementary_filter(flip_pos[2],nav_get_pos_ned_z(),0.996f);
	flip_rf_alt = complementary_filter(flip_rf_alt,rangefinder_get_range_f(),0.994f);
	control_set_rf_alt_target(flip_rf_alt);

	control_alt_pos_common(dt,flip_pos[2],0.2f);
	//control_alt_vel_common(dt,0.0f,0.1f);
	
	control_yaw_common_rc(dt,rc);

	if(fabs(rc->thr_raw) > 0.2f || fabs(rc->roll_raw) > 0.2f || fabs(rc->pitch_raw) > 0.2f){
		control_set_normal_mode();
		INFO(DEBUG_ID,"flip hold break");
	}else{
		if(fabs(nav_get_vel_grd_x()) < 0.1f && fabs(nav_get_vel_grd_y()) < 0.1f){
			flip_hold_time += dt;
			if(flip_hold_time >= flip_hold_timeout){
				pos_control_set_pos_ned_z_target(nav_get_pos_ned_z());
				control_set_normal_mode();
				INFO(DEBUG_ID,"flip done");
			}
		}else{		
			flip_hold_time = 0.0f;
		}
	}
}

void control_flip_update(float dt,rc_s * rc)
{
	flip_att_roll += degrees(attitude_get_rate_roll() * dt);
	flip_att_pitch += degrees(attitude_get_rate_pitch() * dt);
	flip_att_yaw += degrees(attitude_get_rate_yaw() * dt);

	if(flip_mode == FLIP_MODE_RISE){
		control_flip_rise_update(dt,rc);
	}else if(flip_mode == FLIP_MODE_BURST){
		control_flip_burst_update(dt,rc);
	}else if(flip_mode == FLIP_MODE_BREAK){
		control_flip_break_update(dt,rc);
	}else if(flip_mode == FLIP_MODE_STABLE){
		control_flip_stable_update(dt,rc);
	}else if(flip_mode == FLIP_MODE_HOLD){
		control_flip_hold_update(dt,rc);
	}
}

