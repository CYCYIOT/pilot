#include "app_rc.h"
#include "app_debug.h"
#include "app_nav.h"
#include "app_attitude.h"
#include "app_att_control.h"

#include "app_control.h"
#include "app_control_followme.h"
#include "app_control_common.h"

#include "geo.h"

#define DEBUG_ID DEBUG_ID_CONTROL

control_followme_s target_pos;
nav_s fm_poshold;

static float target_roll;
static float target_pitch;
static float target_yaw;
static float target_theta;
static float target_x;
static float target_y;
static float target_distance;
static float followme_distance = 4.0f;

static bool fm_target_printf = false;
static bool fm_target_check = false;

float fm_pos_sp[3] = {0.0f,0.0f,0.0f};
float fm_vel_sp[3] = {0.0f,0.0f,0.0f};


float fm_now[2] = {0};
float fm_mode = 0.0f;
float fm_time = 0.0f;

float fm_target_accuracy = 0.0f;

float fm_att_limited = 8.0f;

bool control_followme_check()
{
	uint8_t check = CONTROL_CHECK_ATT | CONTROL_CHECK_ALT | CONTROL_CHECK_REL_POS | CONTROL_CHECK_ARM;

	return control_check(check);
}

bool control_followme_mode_init(float param1,float param2)
{
	INFO(DEBUG_ID,"control_followme_mode_init \r\n");
	fm_target_printf = true;
	fm_target_check = false;
	target_pos.accuracy = 0.0f;
	nav_get_pos(&fm_poshold);
	if(fm_poshold.pos_valid == true){
		return true;
	}else{
		return false;
	}
}

void control_followme_param_init()
{
	fm_target_accuracy = 1.0f;
}

void control_followme_update(float dt,rc_s * rc)
{
	nav_s pos;
	float pos_att_sp[3] = {0.0f,0.0f,0.0f};

	fm_mode = 0.0f;
	fm_time += dt;
	
	nav_get_pos(&pos);
	if(pos.pos_valid == true){
		fm_mode = 1.0f;
		if(fm_target_check == true){
			float pos_diff[2];

			nav_map_gps(target_pos.lat,target_pos.lon,&fm_now[0],&fm_now[1]);

			pos_diff[0] = fm_now[0] - pos.pos_ned[0];
			pos_diff[1] = fm_now[1] - pos.pos_ned[1];
		
			target_distance = pythagorous2(pos_diff[0],pos_diff[1]);
			if(target_distance <= 20.0f){
				fm_poshold = pos;
				fm_mode = 2.0f;
				
				target_yaw = degrees(atan2f(-pos_diff[0],pos_diff[1])) + 90.0f;
				target_yaw = wrap_180_cd_float(target_yaw);
				target_theta = attitude_get_att_yaw() - target_yaw;
				
				target_distance = target_distance - followme_distance;
				target_x = target_distance * cosf(radians(target_theta));
				target_y = target_distance * sinf(radians(target_theta));
				
				fm_pos_sp[0] = target_distance * cosf(radians(target_yaw)) + pos.pos_ned[0];
				fm_pos_sp[1] = target_distance * sinf(radians(target_yaw)) + pos.pos_ned[1];
				
				control_pos_ned_xy_common(dt,fm_pos_sp,0,0.5f);
				control_yaw_common(dt,target_yaw,0.0f);
			}else{
				fm_mode = 3.0f;

				control_pos_ned_xy_common(dt,fm_poshold.pos_ned,0,0.5f);
				control_yaw_common_rc(dt,rc);
			}
			
			{
				static float fm_debug_time = 0;
				fm_debug_time += dt;
				if(fm_debug_time >= 0.5f){
					fm_debug_time = 0;
					DEBUG(DEBUG_ID,"%3.3f(%3.3f,%3.3f,%3.3f)(%3.3f,%3.3f)->(%lf,%lf)(%3.3f)%3.3f %3.3f\r\n",attitude_get_att_yaw(),target_yaw,pos_att_sp[0],pos_att_sp[1],target_x,target_y,target_pos.lat,target_pos.lon,target_pos.accuracy,target_distance,((float)target_pos.count/fm_time));
				}
			}
		}else{
			fm_mode = 4.0f;
			fm_time = 0.0f;
			target_pos.count = 0;
			fm_target_printf = false;

			control_pos_ned_xy_common(dt,fm_poshold.pos_ned,0,0.5f);
			control_yaw_common_rc(dt,rc);
		}
	}else{
		fm_mode = 5.0f;
		pos_att_sp[0] = 0.0f;
		pos_att_sp[1] = 0.0f;
		control_att_common_rc(dt,rc);
		control_yaw_common_rc(dt,rc);
	}

	control_alt_common_rc(dt,rc);
}

void control_followme_get_debug(float * mode,float * distance,float * tx,float * ty,float * tr,float * tp,float * tyaw)
{
	*mode = fm_mode;
	*distance = target_distance;
	*tx = fm_pos_sp[0];
	*ty = fm_pos_sp[1];
	*tr = target_roll;
	*tp = target_pitch;
	*ty = target_theta;
}

void control_followme_get_diff_xy(float * x,float * y)
{
	*x = target_x;
	*y = target_y;
}

void control_followme_get_target(double * lat,double * lon,float * x,float * y,float * accuracy,float * yaw,float * vel,uint32_t * count)
{
	*lat = target_pos.lat;
	*lon = target_pos.lon;
	*x = fm_now[0];
	*y = fm_now[1];
	*accuracy = target_pos.accuracy;
	*yaw = (float)target_pos.yaw;
	*count = target_pos.count;
	*vel = target_pos.vel;
}

void control_followme_set_target_accuracy(float accuracy,float distance)
{
	INFO(DEBUG_ID,"control_followme_set_target_accuracy:%3.3f,%3.3f\r\n",accuracy,distance);
	fm_target_accuracy = accuracy;
}

void control_followme_set_target(double lat,double lon,float alt,float accuracy,int16_t yaw,float vel)
{
	if(fm_target_printf == true){
		INFO(DEBUG_ID,"control_followme_set_target:%lf,%lf %3.3f %3.3f %d\r\n",lat,lon,alt,accuracy,yaw);
	}

	target_pos.lat = lat;
	target_pos.lon = lon;
	target_pos.alt = alt;
	target_pos.yaw = yaw;
	target_pos.vel = vel;
	target_pos.accuracy = accuracy;
	target_pos.count++;

	fm_target_check = true;
}

