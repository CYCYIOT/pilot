#include "app_rc.h"
#include "app_debug.h"
#include "app_nav.h"
#include "app_attitude.h"
#include "app_control.h"
#include "app_att_control.h"
#include "app_control_common.h"

#include "lib_math.h"
#include "lib_inav_flow.h"

static float l_pos_sp[3] = {0.0f,0.0f,0.0f};

#define DEBUG_ID DEBUG_ID_CONTROL
#define YAW_RATE 0.3f//rad/s

enum local360_status{
	local360_status_start = 0,
	local360_status_rotating,
	local360_status_cancalling,
	local360_status_stopping,
	local360_status_stopped,
};

static float init_yaw=0.0f;
enum local360_status l_status;

bool control_local360_check()
{
	uint8_t check = CONTROL_CHECK_ATT | CONTROL_CHECK_ALT | CONTROL_CHECK_VEL | CONTROL_CHECK_ARM;

	return control_check(check);
}

void control_local360_exit()
{
	nav_recover_acc_bias();
	inav_flow_set_normal_mode();
	
	INFO(DEBUG_ID,"local360 exit");
}

void control_local360_init(float param1,float param2)
{
	nav_s pos;
	
	nav_get_pos(&pos);
	v3f_set(l_pos_sp,pos.pos_ned);
	 
	init_yaw = attitude_get_att_yaw();
	l_status = local360_status_start;
	
	nav_backup_acc_bias();
	inav_flow_set_rotate_mode();
	
	INFO(DEBUG_ID,"local360 init (%3.3f,%3.3f)",l_pos_sp[0],l_pos_sp[1]);
}

void control_local360_param_init()
{
}

void control_local360_update(float dt,rc_s * rc)
{
	att_s att;
	nav_s pos;
	
	attitude_get(&att);
	nav_get_pos(&pos);
	
	switch(l_status){
		case local360_status_start:
			if(fabsf(att.att[2] - init_yaw) > 90.0f){
				l_status = 	local360_status_rotating;
			}
			break;
		case local360_status_rotating:
			if(fabsf(att.att[2] - init_yaw) < 1.0f){
				l_status = 	local360_status_stopping;
			}
			break;
		case local360_status_stopping:
			control_set_normal_mode();
			break;
		default:
			control_set_normal_mode();
			break;
	};

	if(pos.pos_valid == true){
		control_pos_ned_xy_common(dt,l_pos_sp,0,0.5f);
	}else{
		control_att_common_rc(dt,rc);
	}

	control_yaw_rate_common(dt,YAW_RATE,0.0f);
	control_alt_common_rc(dt,rc);

	if(fabs(rc->roll) > 0 || fabs(rc->pitch) > 0){
		control_set_normal_mode();
	}
}

