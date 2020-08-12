#include "app_rc.h"
#include "app_motor.h"
#include "app_debug.h"
#include "app_system.h"

#include "app_control.h"
#include "app_control_common.h"

#define DEBUG_ID DEBUG_ID_CONTROL

bool control_stabilize_check()
{
	uint8_t check = CONTROL_CHECK_ATT | CONTROL_CHECK_DISARM;

	if(control_check(check) == true){
		if(system_set_armed(true) == true){
			return true;
		}
	}

	return false;
}

void control_stabilize_init(float param1,float param2)
{
	INFO(DEBUG_ID,"stabilize init");
}

void control_stabilize_exit()
{
	INFO(DEBUG_ID,"stabilize exit");
}

void control_stabilize_param_init()
{
}

void control_stabilize_update(float dt,rc_s * rc)
{
	control_att_common_rc(dt,rc);
	control_yaw_common_rc(dt,rc);
	motor_control_set_thr((rc->thr + 1.0f) / 2.0f);
}

