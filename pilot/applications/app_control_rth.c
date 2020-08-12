#include "app_rc.h"
#include "app_debug.h"
#include "app_imu.h"
#include "app_nav.h"
#include "app_param.h"
#include "app_attitude.h"
#include "app_system.h"
#include "app_control.h"
#include "app_att_control.h"
#include "app_pos_control.h"
#include "app_control_common.h"

#include "lib_math.h"

#define RTH_HEIGHT 3.0f 
#define RTH_RC_ALT_DEADZONE 0.15f

typedef enum{
	RTH_STATUS_IDLE = 0,
	RTH_STATUS_RAISING,
	RTH_STATUS_GO_HOME,
	RTH_STATUS_LANDING,
	RTH_STATUS_LANDED,
}rth_status_e;

#define DEBUG_ID DEBUG_ID_CONTROL
#define LAND_RADIUS 	0.2f

static rth_status_e rth_status;
static nav_s pos_curr;

static float pos_sp[3] = {0.0f};
static nav_s arm_pos;

#define RTH_LAND_ACC_CHECK_THRESHOLD   (-2.0f * 9.8f)   // m/s^2
#define RTH_LAND_VEL_CHECK_THRESHOLD   (0.3f)
#define RTH_LAND_VEL_CHECK_TIMEOUT     (3.0f)
static float control_rth_land_check_timer = 0.0f;

bool control_rth_check()
{
	uint8_t check = CONTROL_CHECK_ATT | CONTROL_CHECK_ALT | CONTROL_CHECK_ABS_POS | CONTROL_CHECK_ARM;

	if(control_check(check) == true){
		nav_get_pos(&pos_curr);
		nav_get_arm_pos(&arm_pos);
		if(pos_curr.pos_valid == true && arm_pos.pos_valid == true){
			return true;
		}else{
			INFO(DEBUG_ID,"rth check pos failed");
			return false;
		}
	}

	return false;
}

void control_rth_param_init()
{
}

void control_rth_exit()
{
	INFO(DEBUG_ID,"rth exit");
}

void control_rth_init(float param1,float param2)
{	
	v3f_set(pos_sp,pos_curr.pos_ned);

	float dis_to_home = pythagorous2((pos_curr.pos_ned[0] - arm_pos.pos_ned[0]),(pos_curr.pos_ned[1] - arm_pos.pos_ned[1]));
	if(dis_to_home < 3.0f){
		pos_sp[2] = arm_pos.pos_ned[2];
	}else{
		pos_sp[2] = arm_pos.pos_ned[2] + RTH_HEIGHT;
	}
	
	rth_status = RTH_STATUS_RAISING;

	INFO(DEBUG_ID,"rth init (%3.3f,%3.3f,%3.3f)",pos_sp[0],pos_sp[1],pos_sp[2]);
}

bool control_rth_raising(float dt,rc_s * rc)
{	
	if(pos_curr.pos_ned[2] < pos_sp[2]){
		control_pos_ned_xy_common(dt,pos_sp,0,0.5f);
		control_yaw_common_rc(dt,rc);
		control_alt_pos_common(dt,pos_sp[2],3.0f);
		if(fabs(pos_sp[2] - pos_curr.pos_ned[2]) < 0.3f){
			pos_sp[0] = arm_pos.pos_ned[0];
			pos_sp[1] = arm_pos.pos_ned[1];
			pos_sp[2] = pos_curr.pos_ned[2];
			INFO(DEBUG_ID,"going to home:%5.1f,%5.1f,%5.1f",pos_sp[0],pos_sp[1],pos_sp[2]);
			return true;
		}else{
			return false;
		}
	}else{
		pos_sp[0] = arm_pos.pos_ned[0];
		pos_sp[1] = arm_pos.pos_ned[1];
		pos_sp[2] = pos_curr.pos_ned[2];
		
		return true;
	}
	
	return true;
}

bool control_rth_go_home(float dt,rc_s * rc)
{
	float length_to_sp = pythagorous2((pos_sp[0] - pos_curr.pos_ned[0]),(pos_sp[1] - pos_curr.pos_ned[1]));
	float length_vel = pythagorous2(pos_curr.vel_ned[0],pos_curr.vel_ned[1]);

	if(length_to_sp < LAND_RADIUS ){
		if(length_vel < 0.3f){
			INFO(DEBUG_ID,"landing:%5.1f,%5.1f,%5.1f",pos_curr.pos_ned[0],pos_curr.pos_ned[1],pos_curr.pos_ned[2]);
			return true;
		}
		if(fabs(pos_curr.pos_ned[2] - pos_sp[2]) < 0.3f){
			INFO(DEBUG_ID,"landing:%5.1f,%5.1f,%5.1f",pos_curr.pos_ned[0],pos_curr.pos_ned[1],pos_curr.pos_ned[2]);
			return true;
		}
	}

	if(fabs(rc->thr_raw) > RTH_RC_ALT_DEADZONE && fabs(pos_curr.pos_ned[2] - pos_sp[2]) < 0.5f){
		control_alt_common_rc(dt,rc);
		pos_sp[2] = pos_control_get_pos_ned_z_target();
	}else{
		control_alt_pos_common(dt,pos_sp[2],2.0f);
	}

	control_pos_ned_xy_common(dt,pos_sp,0,0.5f);
	control_yaw_common_rc(dt,rc);
	return false;
}

bool control_rth_land_check(float dt)
{
	float acc[3];
	float vel_z;
	imu_get_acc(acc);
	if(acc[2] < RTH_LAND_ACC_CHECK_THRESHOLD){
		return true;
	}

	vel_z = nav_get_vel_ned_z();
	if(fabs(vel_z - 0.0f) < RTH_LAND_VEL_CHECK_THRESHOLD){
		control_rth_land_check_timer += dt;
	}else{
		control_rth_land_check_timer = 0.0f;
	}
	
	if(control_rth_land_check_timer > RTH_LAND_VEL_CHECK_TIMEOUT){
		return true;
	}

	return false;
}

bool control_rth_landing(float dt,rc_s * rc)
{
	control_pos_ned_xy_common(dt,pos_sp,0,0.5f);
	control_yaw_common_rc(dt,rc);
	control_alt_vel_common(dt,-0.8f,0.8f);
	
	if(control_rth_land_check(dt) == true){
		return true;
	}
	
	return false;
}

bool control_rth_landed(float dt,rc_s * rc)
{
	control_set_mode(CONTROL_MODE_STOP,0,0);
	return true;
}

void control_rth_update(float dt,rc_s * t_rc)
{
	nav_get_pos(&pos_curr);
	if(pos_curr.pos_valid == false){
		printf("[RTH]position lost\n");
		control_set_mode(CONTROL_MODE_ALTHOLD,0,0);
	}

	switch(rth_status){
		case RTH_STATUS_RAISING:
			if(control_rth_raising(dt,t_rc) == true){
				rth_status = RTH_STATUS_GO_HOME;
			}
			break;
		case RTH_STATUS_GO_HOME:
			if(control_rth_go_home(dt,t_rc) == true){
				rth_status = RTH_STATUS_LANDING;
			}
			break;
		case RTH_STATUS_LANDING:
			if(control_rth_landing(dt,t_rc) == true){
				rth_status = RTH_STATUS_LANDED;
			}
			break;
		case RTH_STATUS_LANDED:
			if(control_rth_landed(dt,t_rc) == true){
				rth_status = RTH_STATUS_IDLE;
			}
			break;
		case RTH_STATUS_IDLE:
			break;
	}
}

