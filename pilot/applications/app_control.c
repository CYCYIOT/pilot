#include "app_rc.h"
#include "app_debug.h"
#include "app_system.h"
#include "app_control.h"
#include "app_pos_control.h"
#include "app_att_control.h"
#include "app_attitude.h"
#include "app_nav.h"
#include "app_gps.h"
#include "app_imu.h"
#include "app_sensors.h"

#include "app_control_common.h"
#include "app_control_poshold.h"
#include "app_control_althold.h"
#include "app_control_stabilize.h"
#include "app_control_takeoff.h"
#include "app_control_land.h"
#include "app_control_stop.h"
#include "app_control_followme.h"
#include "app_control_local360.h"
#include "app_control_rth.h"
#include "app_control_flip.h"
#include "app_control_circle.h"
#include "app_control_thrown.h"

#define DEBUG_ID DEBUG_ID_CONTROL

typedef struct{
	bool (*check)(void);
	void (*init)(float,float);
	void (*exit)(void);
	void (*update)(float,rc_s *);
}control_s;

static uint8_t control_mode = CONTROL_MODE_STOP;
static uint8_t control_mode_last = CONTROL_MODE_STOP;

static control_s control_modes[CONTROL_MODE_MAX];


bool control_check(uint8_t check)
{
	if(check & CONTROL_CHECK_ATT){
		if(imu_get_acc_status() != SENSOR_STATUS_OK){
			INFO(DEBUG_ID,"ACC check failed");
			return false;
		}
		
		if(imu_get_gyro_status() != SENSOR_STATUS_OK){
			INFO(DEBUG_ID,"GYRO check failed");
			return false;
		}

		if(attitude_get_valid() == false){
			INFO(DEBUG_ID,"ATT check failed");
			return false;
		}
	}

	if(check & CONTROL_CHECK_ALT){
		if(nav_get_alt_valid() == false){
			INFO(DEBUG_ID,"ALT check failed");
			return false;
		}
	}

	if(check & CONTROL_CHECK_VEL){
		if(nav_get_vel_valid() == false){
			INFO(DEBUG_ID,"VEL check failed");
			return false;
		}
	}

	if(check & CONTROL_CHECK_REL_POS){
		if(nav_get_pos_valid() == false){
			INFO(DEBUG_ID,"POS check failed");
			return false;
		}
	}

	if(check & CONTROL_CHECK_ABS_POS){
		if(gps_get_status() != SENSOR_STATUS_OK){
			INFO(DEBUG_ID,"GPS check failed");
			return false;
		}
	}

	if(check & CONTROL_CHECK_ARM){
		if(system_get_armed() == false){
			INFO(DEBUG_ID,"ARM check failed");
			return false;
		}
	}

	if(check & CONTROL_CHECK_DISARM){
		if(system_get_armed() == true){
			INFO(DEBUG_ID,"DISARM check failed");
			return false;
		}
	}

	return true;
}

bool control_mode_check(uint8_t mode)
{
	if(mode < 0 || mode >= CONTROL_MODE_MAX){
		INFO(DEBUG_ID,"mode:%d over",mode);
		return false;
	}

	if(control_modes[mode].check == NULL){
		INFO(DEBUG_ID,"mode:%d check null",mode);
		return false;
	}
	
	if(control_modes[mode].init == NULL){
		INFO(DEBUG_ID,"mode:%d init null",mode);
		return false;
	}
	
	if(control_modes[mode].exit == NULL){
		INFO(DEBUG_ID,"mode:%d exit null",mode);
		return false;
	}
	
	if(control_modes[mode].update == NULL){
		INFO(DEBUG_ID,"mode:%d update null",mode);
		return false;
	}

	return true;
}

bool control_set_mode(uint8_t new_mode,float param1,float param2)
{

 if(control_mode == CONTROL_MODE_POSHOLD && new_mode == CONTROL_MODE_TAKEOFF){
   return false;
 }

	if(control_mode != new_mode){

		if(control_mode_check(new_mode) == false){
			return false;
		}

		INFO(DEBUG_ID,"set mode:%d(%d) (%3.3f,%3.3f)",new_mode,control_mode,param1,param2);

		if(control_modes[new_mode].check() == false){
			return false;
		}
		
		control_modes[control_mode].exit();
		
		control_mode_last = control_mode;
		control_mode = new_mode;

		control_modes[control_mode].init(param1,param2);

	}
	
	return true;
}

void control_set_normal_mode()
{
	if(control_set_mode(CONTROL_MODE_POSHOLD,0,0) == false){
		control_set_mode(CONTROL_MODE_ALTHOLD,0,0);
	}
}

uint8_t control_get_mode()
{	
	return control_mode;
}

uint8_t control_get_mode_last()
{	
	return control_mode_last;
}

bool control_set_mode_last()
{	
	return control_set_mode(control_mode_last,0.0f,0.0f);
}

void control_param_init()
{
	control_common_param_init();

	control_stabilize_param_init();
	control_althold_param_init();
	control_poshold_param_init();
	control_takeoff_param_init();
	control_land_param_init();
	control_stop_param_init();
	control_followme_param_init();
	control_local360_param_init();
	control_rth_param_init();
	control_flip_param_init();
	control_circle_param_init();
	control_thrown_param_init();
}

void control_init()
{
	int count;

	control_common_init();

	for(count = 0 ; count < CONTROL_MODE_MAX ; count++){
		control_modes[count].check = NULL;
		control_modes[count].init = NULL;
		control_modes[count].exit = NULL;
		control_modes[count].update = NULL;
	}

	control_modes[CONTROL_MODE_STABILIZE].check = control_stabilize_check;
	control_modes[CONTROL_MODE_STABILIZE].init = control_stabilize_init;
	control_modes[CONTROL_MODE_STABILIZE].exit = control_stabilize_exit;
	control_modes[CONTROL_MODE_STABILIZE].update = control_stabilize_update;

	control_modes[CONTROL_MODE_ALTHOLD].check = control_althold_check;
	control_modes[CONTROL_MODE_ALTHOLD].init = control_althold_init;
	control_modes[CONTROL_MODE_ALTHOLD].exit = control_althold_exit;
	control_modes[CONTROL_MODE_ALTHOLD].update = control_althold_update;

	control_modes[CONTROL_MODE_POSHOLD].check = control_poshold_check;
	control_modes[CONTROL_MODE_POSHOLD].init = control_poshold_init;
	control_modes[CONTROL_MODE_POSHOLD].exit = control_poshold_exit;
	control_modes[CONTROL_MODE_POSHOLD].update = control_poshold_update;

	control_modes[CONTROL_MODE_TAKEOFF].check = control_takeoff_check;
	control_modes[CONTROL_MODE_TAKEOFF].init = control_takeoff_init;
	control_modes[CONTROL_MODE_TAKEOFF].exit = control_takeoff_exit;
	control_modes[CONTROL_MODE_TAKEOFF].update = control_takeoff_update;

	control_modes[CONTROL_MODE_LAND].check = control_land_check;
	control_modes[CONTROL_MODE_LAND].init = control_land_init;
	control_modes[CONTROL_MODE_LAND].exit = control_land_exit;
	control_modes[CONTROL_MODE_LAND].update = control_land_update;

	control_modes[CONTROL_MODE_CIRCLE].check = control_circle_check;
	control_modes[CONTROL_MODE_CIRCLE].init = control_circle_init;
	control_modes[CONTROL_MODE_CIRCLE].exit = control_circle_exit;
	control_modes[CONTROL_MODE_CIRCLE].update = control_circle_update;

	control_modes[CONTROL_MODE_FLIP].check = control_flip_check;
	control_modes[CONTROL_MODE_FLIP].init = control_flip_init;
	control_modes[CONTROL_MODE_FLIP].exit = control_flip_exit;
	control_modes[CONTROL_MODE_FLIP].update = control_flip_update;

	control_modes[CONTROL_MODE_RTH].check = control_rth_check;
	control_modes[CONTROL_MODE_RTH].init = control_rth_init;
	control_modes[CONTROL_MODE_RTH].exit = control_rth_exit;
	control_modes[CONTROL_MODE_RTH].update = control_rth_update;

	control_modes[CONTROL_MODE_STOP].check = control_stop_check;
	control_modes[CONTROL_MODE_STOP].init = control_stop_init;
	control_modes[CONTROL_MODE_STOP].exit = control_stop_exit;
	control_modes[CONTROL_MODE_STOP].update = control_stop_update;

	control_modes[CONTROL_MODE_LOCAL360].check = control_local360_check;
	control_modes[CONTROL_MODE_LOCAL360].init = control_local360_init;
	control_modes[CONTROL_MODE_LOCAL360].exit = control_local360_exit;
	control_modes[CONTROL_MODE_LOCAL360].update = control_local360_update;

    control_modes[CONTROL_MODE_THROWN].check = control_thrown_check;
	control_modes[CONTROL_MODE_THROWN].init = control_thrown_init;
	control_modes[CONTROL_MODE_THROWN].exit = control_thrown_exit;
	control_modes[CONTROL_MODE_THROWN].update = control_thrown_update;
	
	INFO(DEBUG_ID,"init");
}

void control_update(float dt)
{
	rc_s rc;
	rc_get(&rc);

	if(control_mode >= 0 && control_mode < CONTROL_MODE_MAX && control_modes[control_mode].update != NULL){
		control_modes[control_mode].update(dt,&rc);
	}

	if(system_get_armed() == false){
		rc_set_headfree_yaw(attitude_get_att_yaw());
		
		att_control_set_target_att_yaw(attitude_get_att_yaw());
		att_control_set_target_att_roll(attitude_get_att_roll());
		att_control_set_target_att_pitch(attitude_get_att_pitch());
		
		att_control_reinit();
		pos_control_reinit();
		return;
	}
}

