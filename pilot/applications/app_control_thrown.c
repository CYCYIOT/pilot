#include "app_rc.h"
#include "app_system.h"
#include "app_motor.h"
#include "app_debug.h"
#include "app_nav.h"
#include "app_attitude.h"
#include "app_param.h"
#include "app_sensors.h"
#include "app_rangefinder.h"

#include "app_att_control.h"
#include "app_pos_control.h"
#include "hal_dps280.h"
#include "app_control.h"
#include "app_control_common.h"
#include "app_control_takeoff.h"
#include "app_flow.h"
#include "hal_tof_vl53l1x.h"
#include "app_failsafe.h"

#include "hal.h"
#include "lib_math.h"
#include "lib_inav_baro.h"
#include "lib_inav_flow.h"

#define DEBUG_ID DEBUG_ID_CONTROL

#define TAKEOFF_STEP_SPIN		0
#define TAKEOFF_STEP_NORMAL		1

static float takeoff_spin_time;
//static float takeoff_spin_timeout = 1.0f;
static float takeoff_spin;
static float takeoff_alt;
static float takeoff_rc_check;
static float takeoff_vel_limit;
static uint8_t takeoff_step;

static float tkof_pos_sp[3] = {0.0f};


bool control_thrown_check()
{
	uint8_t check = CONTROL_CHECK_ATT | CONTROL_CHECK_ALT | CONTROL_CHECK_DISARM;

	if(control_check(check) == true){
		if(system_set_armed(true) == true){
			return true;
		}
	}

	return false;
}

void control_thrown_param_init()
{
	param_set_var(CON__TAKEOFF__SPIN_NAME		,&takeoff_spin);
	//param_set_var(CON__TAKEOFF__ALT_NAME		,&takeoff_alt);
	takeoff_alt=0.5;
	param_set_var(CON__TAKEOFF__VEL_LIMIT_NAME	,&takeoff_vel_limit);
	param_set_var(CON__TAKEOFF__RC_CHECK_NAME	,&takeoff_rc_check);
}

void control_thrown_exit()
{
	inav_baro_set_normal_mode();
	inav_flow_set_normal_mode();
	att_control_set_att_pid_normal_mode();
	pos_control_set_vel_pid_normal_mode();
	flow_set_dtg(0.7f,2.0f);
	failsafe_set_att_limit_enable(true);
	INFO(DEBUG_ID,"thrown exit");
}

void control_thrown_init(float param1,float param2)
{
	nav_s pos;

	nav_get_pos(&pos);
	
	tkof_pos_sp[0] = pos.pos_ned[0];
	tkof_pos_sp[1] = pos.pos_ned[1];
	tkof_pos_sp[2] = pos.pos_ned[2] + takeoff_alt;
	
	takeoff_step = TAKEOFF_STEP_SPIN;
	takeoff_spin_time = 0.0f;
	flow_set_dtg(0.2f,2.0f);
	control_set_alt_start(nav_get_pos_ned_z());
	failsafe_set_att_limit_enable(false);
	INFO(DEBUG_ID,"thrown init");
}

void control_thrown_update(float dt,rc_s * rc)
{
	float vel_sp[3] = {0};
	nav_s pos;
	att_s att;
	uint8_t count = 0;
	
	nav_get_pos(&pos);
	attitude_get(&att);

	if(takeoff_step == TAKEOFF_STEP_SPIN){

		float motor[MAX_NUM_MOTORS];

		for(count = 0 ; count < MAX_NUM_MOTORS ;count++){
			motor[count] = 0.15;
		}
		motor_set_test(false,0.01f,motor);
		
		if(pos.vel_ned[2] < -0.8 || pos.vel_ned[2] > 0.5 || pos.vel_ned[1] < -0.8 || pos.vel_ned[1] > 0.5 || pos.vel_ned[0] < -0.8 || pos.vel_ned[0] > 0.5){
			tkof_pos_sp[2] = pos.pos_ned[2] + takeoff_alt;
			//tkof_pos_sp[2] = takeoff_alt;
			debug_t("== pos.pos_ned[2] = %f tkof_pos_sp[2] = %f att.roll = %f att.pitch = %f\n ",pos.pos_ned[2],tkof_pos_sp[2],att.att[0],att.att[1]);

			inav_baro_set_thrown_mode();
			//inav_flow_set_takeoff_mode();
            inav_flow_set_thrown_mode();
			
			pos_control_set_pos_ned_z_target(tkof_pos_sp[2]);
			control_set_rf_alt_target(takeoff_alt);
			
			//att_control_set_att_pid_takeoff_mode();
			//att_control_set_att_pid_normal_mode();
			//pos_control_set_vel_pid_takeoff_mode();
			pos_control_set_vel_pid_thrown_mode();
			takeoff_step = TAKEOFF_STEP_NORMAL;
			INFO(DEBUG_ID,"thrown now");
		}
		
	}else{
		if(fabs(rc->pitch) > 0 || fabs(rc->roll) > 0){
			control_set_normal_mode();
		}else{
			if(pos.pos_valid){
				//control_pos_ned_xy_common(dt,tkof_pos_sp,0,0.5f);
				//control_vel_ned_xy_common(dt,vel_sp,0,0.5f);
				control_vel_ned_xy_common(dt,vel_sp,0,1.0f);
			}else{
				control_att_common_rc(dt,rc);
			}
		}

	//	control_att_common_rc(dt,rc);    //ÐÂÔö ¿ØÖÆ×ËÌ¬
			
		control_yaw_common_rc(dt,rc);
		//control_alt_pos_common(dt,tkof_pos_sp[2],takeoff_vel_limit);
		control_alt_pos_common(dt,tkof_pos_sp[2],1.0f);

		nav_att_offset_update();		

		if(fabs(rc->thr_raw) > takeoff_rc_check || fabs(pos.pos_ned[2] - tkof_pos_sp[2]) < 0.10f || get_tof_data_yaw() > 1.5f ){
		debug_t("pos.pos_ned[2] = %f tkof_pos_sp[2] = %f att.roll = %f att.pitch = %f tof_data = %f\n ",pos.pos_ned[2],tkof_pos_sp[2],att.att[0],att.att[1],get_tof_data_yaw());
			control_set_normal_mode();
		}	

	}

}

