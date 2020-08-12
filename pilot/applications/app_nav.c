#include "app_debug.h"
#include "app_nav.h"
#include "app_system.h"
#include "app_rc.h"
#include "app_imu.h"
#include "app_gps.h"
#include "app_flow.h"
#include "app_attitude.h"
#include "app_pos_control.h"

#define DEBUG_ID DEBUG_ID_NAV

nav_s nav_pos;
nav_s arm_pos;
float nav_dis_to_ground;
att_s att;
bool arm_status_last;

inav_estimator inav_est;

float acc_bias_backup[3]={0.0f};
float acc_bias_backup_flag = false;

float nav_att_offset[2] = {0};

void nav_param_init()
{
	inav_param_init(&inav_est);

	nav_pos.pos_valid = false;
	arm_pos.pos_valid = false;
}

void nav_init()                     //º½Ïò
{
	INFO(DEBUG_ID,"init");	
	arm_status_last = system_get_armed();
	inav_init(&inav_est);
}

void nav_update(float dt)
{
	bool arm_status;
	float yaw_cos = 0.0f;
	float yaw_sin = 0.0f;
	gps_info_s  gps_in;

	if(gps_get_info(&gps_in) == true){
		inav_apply_gps(&inav_est,&gps_in);
	}
	
	inav_update(dt,&inav_est,system_get_armed());
	
	if((inav_est.ref.init_done == true) && (inav_est.pos_valid == false)){
		INFO(DEBUG_ID,"get position failed");
	}else{
		yaw_cos = cosf(radians(attitude_get_att_yaw()));
		yaw_sin = sinf(radians(attitude_get_att_yaw()));

		nav_pos.pos_ned[0] = inav_est.pos_ned[0];
		nav_pos.pos_ned[1] = inav_est.pos_ned[1];
		nav_pos.pos_ned[2] = -inav_est.pos_ned[2];
		
		nav_pos.vel_ned[0] = inav_est.vel_ned[0];
		nav_pos.vel_ned[1] = inav_est.vel_ned[1];
		nav_pos.vel_ned[2] = -inav_est.vel_ned[2];
		
		v3f_set(nav_pos.acc_ned,inav_est.acc_ned);
		v3f_set(nav_pos.acc_body,inav_est.acc_body);
		v3f_set(nav_pos.acc_bias_body,inav_est.acc_bias_body);

		nav_pos.acc_body_sum = pythagorous_v3f(nav_pos.acc_body);
		
		nav_pos.vel_grd[0] =  nav_pos.vel_ned[0] * yaw_cos + nav_pos.vel_ned[1] * yaw_sin;
		nav_pos.vel_grd[1] = -nav_pos.vel_ned[0] * yaw_sin + nav_pos.vel_ned[1] * yaw_cos;
		nav_pos.vel_grd[2] = nav_pos.vel_ned[2];
		
		nav_pos.pos_valid  = inav_est.pos_valid;
		nav_pos.vel_valid  = inav_est.vel_valid;
		nav_pos.alt_valid  = inav_est.alt_valid;
	}
	
	arm_status = system_get_armed();
	if(arm_status != arm_status_last){
		if(arm_status == true){
			memcpy(&arm_pos,&nav_pos,sizeof(nav_s));
			INFO(DEBUG_ID,"arm pos(%3.3f,%3.3f,%3.3f)",arm_pos.pos_ned[0],arm_pos.pos_ned[1],arm_pos.pos_ned[2]);
			arm_pos.pos_valid = true;
		}else{
			arm_pos.pos_valid = false;
		}
	}
	arm_status_last = arm_status;

	if(arm_pos.pos_valid == true){
		nav_dis_to_ground = nav_pos.pos_ned[2] - arm_pos.pos_ned[2];
		nav_dis_to_ground = nav_dis_to_ground > 0? nav_dis_to_ground:0.0f;
	}else{
		nav_dis_to_ground = 0.0f; 
	}
}

void nav_att_offset_update()
{
	float vel_length;
	att_s att;
	
	attitude_get(&att);
	
	vel_length = pythagorous2(nav_pos.vel_grd[0],nav_pos.vel_grd[1]);
	if(vel_length < 0.06f){
		vel_control_xy_i_stash();
		nav_att_offset[0] = complementary_filter(att.att[0],nav_att_offset[0],0.001f);
		nav_att_offset[1] = complementary_filter(att.att[1],nav_att_offset[1],0.001f);
		nav_att_offset[0]  = constrain_float(nav_att_offset[0],-10.0f,10.0f);
		nav_att_offset[1]  = constrain_float(nav_att_offset[1],-10.0f,10.0f);
	}
}

void nav_get_att_offset(float att_offset[2])
{
	att_offset[0] = nav_att_offset[0];
	att_offset[1] = nav_att_offset[1];
}

void nav_recover_acc_bias()
{
	if(acc_bias_backup_flag == true){
		acc_bias_backup_flag = false;
		v3f_set(inav_est.acc_bias_body,acc_bias_backup);
	}
}

void nav_backup_acc_bias()
{
	if(acc_bias_backup_flag == false){
		acc_bias_backup_flag = true;
		v3f_set(acc_bias_backup,inav_est.acc_bias_body);
	}
}

void nav_get_pos(nav_s *dat)
{
	memcpy(dat,&nav_pos,sizeof(nav_s));
}

void nav_get_arm_pos(nav_s *dat)
{
	memcpy(dat,&arm_pos,sizeof(nav_s));
}

float nav_get_dis_to_grd(void)
{
	return nav_dis_to_ground;
}

float nav_get_acc_body_sum()
{
	return nav_pos.acc_body_sum;
}

float nav_get_acc_bias_body_x()
{
	return nav_pos.acc_bias_body[0];
}

float nav_get_acc_bias_body_y()
{
	return nav_pos.acc_bias_body[1];
}

float nav_get_acc_bias_body_z()
{
	return nav_pos.acc_bias_body[2];
}

float nav_get_acc_ned_x()
{
	return nav_pos.acc_ned[0];
}

float nav_get_acc_ned_y()
{
	return nav_pos.acc_ned[1];
}

float nav_get_acc_ned_z()
{
	return nav_pos.acc_ned[2];
}

float nav_get_pos_ned_x()
{
	return nav_pos.pos_ned[0];
}

float nav_get_pos_ned_y()
{
	return nav_pos.pos_ned[1];
}

float nav_get_pos_ned_z()
{
	return nav_pos.pos_ned[2];
}

float nav_get_vel_ned_x()
{
	return nav_pos.vel_ned[0];
}

float nav_get_vel_ned_y()
{
	return nav_pos.vel_ned[1];
}

float nav_get_vel_ned_z()
{
	return nav_pos.vel_ned[2];
}

float nav_get_vel_grd_x()
{
	return nav_pos.vel_grd[0];
}

float nav_get_vel_grd_y()
{
	return nav_pos.vel_grd[1];
}

float nav_get_vel_grd_z()
{
	return nav_pos.vel_grd[2];
}

bool nav_get_pos_valid()
{
	return nav_pos.pos_valid;
}

bool nav_get_vel_valid()
{
	return nav_pos.vel_valid;
}

bool nav_get_alt_valid()
{
	return nav_pos.alt_valid;
}

void nav_map_gps(double lat,double lon,float * x,float * y)
{
	map_projection_project(&(inav_est.ref),lat,lon,x,y);
}

void nav_get_estimator_data(inav_estimator *est)
{
	memcpy(est,&inav_est,sizeof(inav_est));
}

void nav_reset()
{
	inav_reset(&inav_est);
}

