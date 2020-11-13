#include "app_param.h"
#include "app_rc.h"
#include "app_nav.h"
#include "app_imu.h"
#include "app_system.h"
#include "app_debug.h"
#include "app_control.h"
#include "app_motor.h"
#include "app_control_common.h"
#include "app_pos_control.h"
#include "hal_aruco_linux.h"
#include "hal_dps280.h"
#include "app_batt.h"
#include "hal.h"
#include "lib_math.h"
#include "app_control_takeoff.h"
#include "app_failsafe.h"

#define DEBUG_ID DEBUG_ID_CONTROL

static float land_vel_target;

static float land_check_acc_val;
static float land_check_thr_val;

static float land_check_thr_time;
static float land_check_thr_timeout;

static float land_pos_sp[3] = {0.0f};

bool control_land_check()
{
	uint8_t check = CONTROL_CHECK_ATT | CONTROL_CHECK_ALT | CONTROL_CHECK_ARM;

	return control_check(check);
}

void control_land_param_init()
{
	param_set_var(CON__LAND__VEL_NAME				,&land_vel_target);
	param_set_var(CON__LAND__CHECK_ACC_NAME			,&land_check_acc_val);
	param_set_var(CON__LAND__CHECK_THR_NAME			,&land_check_thr_val);
	param_set_var(CON__LAND__CHECK_THR_TIME_NAME	,&land_check_thr_timeout);
}

void control_land_exit()
{
	pos_control_set_alt_pid_normal_mode();
	failsafe_set_collision_enable(true);
	failsafe_set_collision_file_enable(false);
	INFO(DEBUG_ID,"land exit");
}

void control_land_init(float param1,float param2)
{
	nav_s pos;
	nav_get_pos(&pos);
	
	land_pos_sp[0] = pos.pos_ned[0];
	land_pos_sp[1] = pos.pos_ned[1];

	land_check_thr_time = 0.0f;
 //   debug_t("vol = %d\n",batt_get_origin_vol());
	pos_control_set_alt_pid_land_mode();
    failsafe_set_collision_enable(false);

	INFO(DEBUG_ID,"land init");
}

bool control_land_update_land_check_acc(float dt)
{
	float acc[3];
	
	imu_get_acc(acc);

	if(acc[2] < land_check_acc_val){
		INFO(DEBUG_ID,"land acc checked"); 
		return true;
	}

	return false;
}

void control_land_update_land_check_thr_reset()
{
	land_check_thr_time = 0.0f;
}

bool control_land_update_land_check_thr(float dt)
{
	if(motor_control_get_thr() <= land_check_thr_val){
		land_check_thr_time += dt;
	}else{
		land_check_thr_time = 0.0f;
	}
	
	if(land_check_thr_time > land_check_thr_timeout){
		INFO(DEBUG_ID,"land thr checked"); 
		return true;
	}

	return false;
}

bool control_land_update_land_check(float dt)
{
	if(control_land_update_land_check_acc(dt) == true){
		return true;
	}

	if(control_land_update_land_check_thr(dt) == true){
		return true;
	}
	
	return false;
}

void control_land_update(float dt,rc_s * rc)
{
	nav_s pos;
	nav_get_pos(&pos);
	set_takeoff_flag_alt();
	if(fabs(rc->roll) > 0 || fabs(rc->pitch) > 0){
		control_att_common_rc(dt,rc);
		land_pos_sp[0] = pos.pos_ned[0];
		land_pos_sp[1] = pos.pos_ned[1];
	}else{
		if(pos.pos_valid){
			control_pos_ned_xy_common(dt,land_pos_sp,0,0.5f);
		}else{
			control_att_common_rc(dt,rc);
		}	
	}

	control_yaw_common_rc(dt,rc);
	control_alt_vel_common(dt,land_vel_target,land_vel_target);
#if 0
	if(fabs(rc->thr_raw) > 0){      //²âÊÔÉÏÏÂÒÆ¶¯£¬½µÂäÖ¸Áî²»ÏìÓ¦
	//	control_set_normal_mode();
	}
#endif	
	if(control_land_update_land_check(dt) == true){
		INFO(DEBUG_ID,"land done"); 
		control_set_mode(CONTROL_MODE_STOP,0,0);
	}
  
   
}

