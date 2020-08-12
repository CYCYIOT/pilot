#include "app_control_common.h"
#include "app_system.h"
#include "app_att_control.h"
#include "app_attitude.h"
#include "app_motor.h"
#include "app_rc.h"
#include "app_pos_control.h"
#include "app_attitude.h"
#include "app_nav.h"
#include "app_param.h"
#include "app_rangefinder.h"
#include "app_sensors.h"

#include "lib_math.h"

bool rf_alt_init = false;
float rf_alt_target;

float yaw_smooth = 0.9f;
float yaw_smooth_time = 1.0f;
float yaw_smooth_time_count = 0;

float alt_smooth = 0.95f;
float alt_smooth_time = 0.8f;
float alt_smooth_time_count = 0;

float rf_smooth_timeout = 0.5f;
float rf_smooth_time = 0.0f;

float pos_z_start = 0.0f;

float control_att_rp_limit = 0.0f;
float control_rate_yaw_limit = 0.0f;
float control_rate_rp_limit = 0.0f;
float control_rate_rp_stab_limit = 0.0f;
float control_pos_z_limit = 0.0f;
float control_vel_z_limit = 0.0f;
float control_vel_z_stab_limit = 0.0f;
float control_vel_xy_limit = 0.0f;

float control_rf_gain = 0.0f;
float control_rf_vel_check = 0.0f;

float control_att_rp_gain;
float control_rate_yaw_gain;
float control_vel_z_gain;

float control_get_rf_alt_target()
{
	return rf_alt_target;
}

void control_set_rf_alt_target(float alt)
{
	rf_alt_target = alt;
}

void control_yaw_common(float dt,float yaw_target,float rate_limit)
{
	float rate_taret_yaw;

	att_control_set_target_att_yaw(yaw_target);
	rate_taret_yaw = att_control_att_yaw_update(dt);	
	control_yaw_rate_common(dt,rate_taret_yaw,rate_limit);
}

void control_yaw_rate_common(float dt,float yaw_rate_target,float rate_limit)
{
	float yaw_output;

	if(rate_limit == 0.0f){
		rate_limit = fabs(control_rate_yaw_limit);
	}else{
		rate_limit = fabs(rate_limit);
	}
	
	yaw_rate_target = constrain_float(yaw_rate_target,-rate_limit,rate_limit);
	att_control_set_target_rate_yaw(yaw_rate_target);
	yaw_output = att_control_rate_yaw_update(dt);
	
	motor_control_set_yaw(yaw_output);
}

void control_pos_ned_xy_common(float dt, float pos_ned_target[3],float vel_limit,float rate_limit)
{
	float vel_xy_target[2];

	pos_control_set_pos_ned_x_target(pos_ned_target[0]);
	pos_control_set_pos_ned_y_target(pos_ned_target[1]);

	vel_xy_target[0] = pos_control_pos_ned_x_update(dt);
	vel_xy_target[1] = pos_control_pos_ned_y_update(dt);

	control_vel_ned_xy_common(dt,vel_xy_target,vel_limit,rate_limit);
}

void control_vel_grd_xy_common(float dt, float vel_grd_target[2],float vel_limit,float rate_limit)
{
	float att_target[2];
	float vel_target_length;

	if(float_is_zero(vel_limit) == true){
		vel_limit = control_vel_xy_limit;
	}else{
		vel_limit = fabs(vel_limit);
	}

	vel_target_length = pythagorous2(vel_grd_target[0],vel_grd_target[1]);
	if(vel_target_length > fabs(vel_limit)){
		vel_grd_target[0] = vel_grd_target[0] / (vel_target_length / fabs(vel_limit));
		vel_grd_target[1] = vel_grd_target[1] / (vel_target_length / fabs(vel_limit));
	}
	
	pos_control_set_vel_grd_x_target(vel_grd_target[0]);
	pos_control_set_vel_grd_y_target(vel_grd_target[1]);
	
	att_target[0] = pos_control_vel_grd_y_update(dt);
	att_target[1] = pos_control_vel_grd_x_update(dt);
	
	control_att_common(dt,degrees(att_target[0]),-degrees(att_target[1]),rate_limit);
}


void control_vel_ned_xy_common(float dt, float vel_ned_target[3],float vel_limit,float rate_limit)
{
	float yaw_cos = 0.0f;
	float yaw_sin = 0.0f;
	float att_target[2];
	float vel_grd_target[2];
	float vel_target_length;
	
	yaw_cos = cosf(radians(attitude_get_att_yaw()));
	yaw_sin = sinf(radians(attitude_get_att_yaw()));

	if(float_is_zero(vel_limit) == true){
		vel_limit = control_vel_xy_limit;
	}else{
		vel_limit = fabs(vel_limit);
	}

	vel_target_length = pythagorous2(vel_ned_target[0],vel_ned_target[1]);
	if(vel_target_length > fabs(vel_limit)){
		vel_ned_target[0] = vel_ned_target[0] / (vel_target_length / fabs(vel_limit));
		vel_ned_target[1] = vel_ned_target[1] / (vel_target_length / fabs(vel_limit));
	}

	vel_grd_target[0] =  vel_ned_target[0] * yaw_cos + vel_ned_target[1] * yaw_sin;
	vel_grd_target[1] = -vel_ned_target[0] * yaw_sin + vel_ned_target[1] * yaw_cos;

	pos_control_set_vel_ned_x_target(vel_ned_target[0]);
	pos_control_set_vel_ned_y_target(vel_ned_target[1]);

	pos_control_set_vel_grd_x_target(vel_grd_target[0]);
	pos_control_set_vel_grd_y_target(vel_grd_target[1]);
	
	att_target[0] = pos_control_vel_grd_y_update(dt);
	att_target[1] = pos_control_vel_grd_x_update(dt);
	
	control_att_common(dt,degrees(att_target[0]),-degrees(att_target[1]),rate_limit);
}

void control_att_rate_common(float dt,float roll_target,float pitch_target,float rate_limit)
{
	float roll_output;
	float pitch_output;

	if(float_is_zero(rate_limit) == true){
		rate_limit = control_rate_rp_stab_limit;
	}else{
		rate_limit = fabs(rate_limit);
	}

	roll_target = constrain_float(roll_target,-rate_limit,rate_limit);
	pitch_target = constrain_float(pitch_target,-rate_limit,rate_limit);

	att_control_set_target_rate_roll(roll_target);
	att_control_set_target_rate_pitch(pitch_target);

	roll_output = att_control_rate_roll_update(dt);
	pitch_output = att_control_rate_pitch_update(dt);

	motor_control_set_roll(roll_output);
	motor_control_set_pitch(pitch_output);
}

void control_att_common(float dt,float roll_target,float pitch_target,float rate_limit)
{
	float rate_taret_roll;
	float rate_taret_pitch;

	roll_target = constrain_float(roll_target,-control_att_rp_limit,control_att_rp_limit);
	pitch_target = constrain_float(pitch_target,-control_att_rp_limit,control_att_rp_limit);

	att_control_set_target_att_roll(roll_target);
	att_control_set_target_att_pitch(pitch_target);
	
	rate_taret_roll = att_control_att_roll_update(dt);
	rate_taret_pitch = att_control_att_pitch_update(dt);

	control_att_rate_common(dt,rate_taret_roll,rate_taret_pitch,rate_limit);
}

void control_set_alt_start(float alt)
{
	pos_z_start = alt;
	rf_alt_init = false;
}

void control_alt_vel_common(float dt,float vel_z_target,float vel_limit)
{
	float thr_target;

	if(float_is_zero(vel_limit) == true){
		vel_limit = fabs(control_vel_z_limit);
	}else{
		vel_limit = fabs(vel_limit);
	}

	if(nav_get_pos_ned_z() > (pos_z_start + control_pos_z_limit) && vel_z_target > 0){
		vel_z_target = 0;
	}
	
	vel_z_target = constrain_float(vel_z_target,-vel_limit,vel_limit);	
	pos_control_set_vel_ned_z_target(vel_z_target);
	thr_target = pos_control_vel_ned_z_update(dt);
	
	motor_control_set_thr(thr_target);
}

void control_alt_pos_common(float dt,float pos_z_target,float vel_limit)
{
	float vel_z_target;

	if(pos_z_target > (pos_z_start + control_pos_z_limit)){
		pos_z_target = pos_z_start + control_pos_z_limit;
	}

	pos_control_set_pos_ned_z_target(pos_z_target);

	if(rangefinder_get_status() == SENSOR_STATUS_OK){
		float rf_alt = rangefinder_get_range_f();
		vel_z_target = (rf_alt_target - rf_alt) * control_rf_gain;
	}else{
		vel_z_target = pos_control_pos_ned_z_update(dt);
	}
	
	control_alt_vel_common(dt,vel_z_target,vel_limit);
}

void control_alt_common_rc(float dt,rc_s * rc)
{
	float vel_z_target = 0.0f;
	float pos_z_curr = 0.0f;
	float vel_limit = 0.0f;

	pos_z_curr = nav_get_pos_ned_z();
#if 1
	if(rangefinder_get_status() == SENSOR_STATUS_OK){
		float rf_alt = rangefinder_get_range_f();
		
		vel_limit = control_vel_z_limit;
		if(fabs(rc->thr) > 0.0f){
			vel_z_target = rc->thr * control_vel_z_gain;
			rf_smooth_time = rf_smooth_timeout;
		}else{
			if(rf_alt_init == false || fabs(rangefinder_get_vel()) >= control_rf_vel_check){
				rf_alt_init = true;
				rf_smooth_time = rf_smooth_timeout;
			}

			if(rf_smooth_time > 0){
				rf_smooth_time -= dt;
				rf_alt_target = rf_alt;
			}

			vel_z_target = (rf_alt_target - rf_alt) * control_rf_gain;
		}

		pos_control_set_pos_ned_z_target(pos_z_curr);
	}else{
#endif
		rf_alt_init = false;
		if(fabs(rc->thr) > 0.0f && !(pos_z_curr > (pos_z_start + control_pos_z_limit) && rc->thr > 0)){
			vel_z_target = rc->thr * control_vel_z_gain;
			pos_control_set_pos_ned_z_target(pos_z_curr);
			alt_smooth_time_count = alt_smooth_time;
			vel_limit = control_vel_z_limit;
		}else{
			float alt_tmp;
			if(alt_smooth_time_count > 0){
				alt_smooth_time_count -= dt;
				alt_tmp = complementary_filter(pos_control_get_pos_ned_z_target(),pos_z_curr,alt_smooth);
				pos_control_set_pos_ned_z_target(alt_tmp);
				vel_limit = control_vel_z_limit;
			}else{
				if(fabs(rc->roll) > 0 || fabs(rc->pitch) > 0){
					vel_limit = control_vel_z_limit;
				}else{
					vel_limit = control_vel_z_stab_limit;
				}
			}
			vel_z_target = pos_control_pos_ned_z_update(dt);
		}
	}

	control_alt_vel_common(dt,vel_z_target,vel_limit);
}

void control_yaw_common_rc(float dt,rc_s * rc)
{
	float yaw_tmp;

	if(fabs(rc->yaw) > 0.0f){
		control_yaw_rate_common(dt,rc->yaw * control_rate_yaw_gain,0.0f);
		att_control_set_target_att_yaw(attitude_get_att_yaw());
		yaw_smooth_time_count = yaw_smooth_time;
	}else{
		if(yaw_smooth_time_count > 0){
			yaw_smooth_time_count -= dt;
			yaw_tmp = complementary_filter(att_control_get_target_att_yaw(),attitude_get_att_yaw(),yaw_smooth);
			att_control_set_target_att_yaw(yaw_tmp);
		}
		
		control_yaw_common(dt,att_control_get_target_att_yaw(),0.0f);
	}
}

void control_att_common_rc(float dt,rc_s * rc)
{
	float rate_limit = 0.0f;
	float roll	= rc->roll;
	float pitch = rc->pitch;

	if(fabs(roll) > 0 || fabs(pitch) > 0){
		rate_limit = control_rate_rp_limit;
	}

	control_att_common(dt,roll * control_att_rp_gain,pitch * control_att_rp_gain,rate_limit);
}

void control_att_common_limit(float dt,float roll_target,float pitch_target,float att_limit,float rate_limit)
{
	float rate_taret_roll;
	float rate_taret_pitch;

	roll_target = constrain_float(roll_target,-att_limit,att_limit);
	pitch_target = constrain_float(pitch_target,-att_limit,att_limit);

	att_control_set_target_att_roll(roll_target);
	att_control_set_target_att_pitch(pitch_target);
	
	rate_taret_roll = att_control_att_roll_update(dt);
	rate_taret_pitch = att_control_att_pitch_update(dt);

	control_att_rate_common(dt,rate_taret_roll,rate_taret_pitch,rate_limit);
}

void control_common_param_init()
{
	param_set_var(CON__COMM__RF_GAIN_NAME				,&control_rf_gain);
	param_set_var(CON__COMM__RF_VEL_CHECK_NAME			,&control_rf_vel_check);

	param_set_var(CON__COMM__ATT_RP_GAIN_NAME			,&control_att_rp_gain);
	param_set_var(CON__COMM__RATE_YAW_GAIN_NAME			,&control_rate_yaw_gain);
	param_set_var(CON__COMM__VEL_Z_GAIN_NAME			,&control_vel_z_gain);

	param_set_var(CON__COMM__ATT_RP_LIMIT_NAME			,&control_att_rp_limit);
	param_set_var(CON__COMM__RATE_YAW_LIMIT_NAME		,&control_rate_yaw_limit);
	param_set_var(CON__COMM__RATE_RP_LIMIT_NAME			,&control_rate_rp_limit);
	param_set_var(CON__COMM__RATE_RP_STAB_LIMIT_NAME	,&control_rate_rp_stab_limit);
	param_set_var(CON__COMM__VEL_Z_LIMIT_NAME			,&control_vel_z_limit);
	param_set_var(CON__COMM__VEL_Z_STAB_LIMIT_NAME		,&control_vel_z_stab_limit);
	param_set_var(CON__COMM__VEL_XY_LIMIT_NAME			,&control_vel_xy_limit);
	param_set_var(CON__COMM__POS_Z_LIMIT_NAME			,&control_pos_z_limit);
}

void control_common_init()
{
	
}

