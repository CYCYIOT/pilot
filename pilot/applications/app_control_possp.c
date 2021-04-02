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
#include "hal.h"
#include "lib_math.h"

#define POSSP_RC_ALT_DEADZONE 0.15f

typedef enum{
	POSSP_STATUS_IDLE = 0,
	POSSP_STATUS_GO_HOME,
	POSSP_STATUS_LANDING,
	POSSP_STATUS_LANDED,
}possp_status_e;

#define DEBUG_ID DEBUG_ID_CONTROL
#define POSSP_RADIUS 	0.1f

static possp_status_e possp_status;
static nav_s pos_curr;
static float dt_sum_timeout =0;

static float pos_sp[3] = {0.0f};


#define POSSP_LAND_ACC_CHECK_THRESHOLD   (-2.0f * 9.8f)   // m/s^2
#define POSSP_LAND_VEL_CHECK_THRESHOLD   (0.3f)
#define POSSP_LAND_VEL_CHECK_TIMEOUT     (3.0f)
static float control_possp_land_check_timer = 0.0f;

bool control_possp_check()
{
	uint8_t check = CONTROL_CHECK_ATT | CONTROL_CHECK_ALT  | CONTROL_CHECK_ARM;

	if(control_check(check) == true){
		nav_get_pos(&pos_curr);
		if(pos_curr.pos_valid == true){
    	debug_t("cur_pos(%3.3f,%3.3f)\n",pos_curr.pos_ned[0],pos_curr.pos_ned[1]);
			return true;
		}else{
			INFO(DEBUG_ID,"possp check pos failed");
			debug_t("possp check pos failed\n");
			return false;
		}
	}

	return false;
}

void control_possp_param_init()
{
}

void control_possp_exit()
{
	INFO(DEBUG_ID,"possp exit");
}

void control_possp_init(float param1,float param2)
{	
	v3f_set(pos_sp,pos_curr.pos_ned);
    dt_sum_timeout =0.0f;
	pos_sp[0] = param1;
	pos_sp[1] = param2;
 	
	possp_status = POSSP_STATUS_GO_HOME;

	INFO(DEBUG_ID,"possp init (%3.3f,%3.3f,%3.3f)",pos_sp[0],pos_sp[1],pos_sp[2]);
}

bool control_possp_go_home(float dt,rc_s * rc)
{
	float length_to_sp = pythagorous2((pos_sp[0] - pos_curr.pos_ned[0]),(pos_sp[1] - pos_curr.pos_ned[1]));
	float length_vel = pythagorous2(pos_curr.vel_ned[0],pos_curr.vel_ned[1]);

	dt_sum_timeout+=dt;
	if(dt_sum_timeout > 15.0f){
	   dt_sum_timeout = 0.0f;
	   debug_t("timeout possp length_vel = %f	length_to_sp = %f\n",length_vel,length_to_sp);
	 return true;
	}

	if(length_to_sp < POSSP_RADIUS ){
		if(length_vel < 0.1f){
			INFO(DEBUG_ID,"landing:%5.1f,%5.1f,%5.1f",pos_curr.pos_ned[0],pos_curr.pos_ned[1],pos_curr.pos_ned[2]);
			debug_t(" possp vel  = % f pos:%5.1f,%5.1f,%5.1f\n",length_vel,pos_curr.pos_ned[0],pos_curr.pos_ned[1],pos_curr.pos_ned[2]);
			return true;
		}
		
	}

	if(fabs(rc->thr_raw) > POSSP_RC_ALT_DEADZONE && fabs(pos_curr.pos_ned[2] - pos_sp[2]) < 0.5f){
		control_alt_common_rc(dt,rc);
		pos_sp[2] = pos_control_get_pos_ned_z_target();
	}else{
		control_alt_pos_common(dt,pos_sp[2],2.0f);
	}

	control_pos_ned_xy_common(dt,pos_sp,0,0.5f);
	control_yaw_common_rc(dt,rc);
	return false;
}

bool control_possp_land_check(float dt)
{
	float acc[3];
	float vel_z;
	imu_get_acc(acc);
	if(acc[2] < POSSP_LAND_ACC_CHECK_THRESHOLD){
		return true;
	}

	vel_z = nav_get_vel_ned_z();
	if(fabs(vel_z - 0.0f) < POSSP_LAND_VEL_CHECK_THRESHOLD){
		control_possp_land_check_timer += dt;
	}else{
		control_possp_land_check_timer = 0.0f;
	}
	
	if(control_possp_land_check_timer > POSSP_LAND_VEL_CHECK_TIMEOUT){
		return true;
	}

	return false;
}

bool control_possp_landing(float dt,rc_s * rc)
{
	control_pos_ned_xy_common(dt,pos_sp,0,0.5f);
	control_yaw_common_rc(dt,rc);
	control_alt_vel_common(dt,-0.8f,0.8f);
	
	if(control_possp_land_check(dt) == true){
		return true;
	}
	
	return false;
}

bool control_possp_landed(float dt,rc_s * rc)
{
	control_set_mode(CONTROL_MODE_STOP,0,0);
	return true;
}

void control_possp_update(float dt,rc_s * t_rc)
{
	nav_get_pos(&pos_curr);
	if(pos_curr.pos_valid == false){
		printf("[POSSP]position lost\n");
		control_set_mode(CONTROL_MODE_ALTHOLD,0,0);
	}

	switch(possp_status){
		case POSSP_STATUS_GO_HOME:
			if(control_possp_go_home(dt,t_rc) == true){
			 //	possp_status = POSSP_STATUS_LANDING;
				control_set_normal_mode();
			}
			break;
		case POSSP_STATUS_LANDING:
			if(control_possp_landing(dt,t_rc) == true){
				possp_status = POSSP_STATUS_LANDED;
			}
			break;
		case POSSP_STATUS_LANDED:
			if(control_possp_landed(dt,t_rc) == true){
				possp_status = POSSP_STATUS_IDLE;
			}
			break;
		case POSSP_STATUS_IDLE:
			break;
	}
}

