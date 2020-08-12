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
#include "hal_aruco_linux.h"
#include "hal_tof_vl53l1x.h"
#include "awlink_item_control.h"

#include "hal.h"
#include "lib_math.h"
#include "lib_inav_baro.h"
#include "lib_inav_flow.h"
#include "hal_stm8s.h"
#define DEBUG_ID DEBUG_ID_CONTROL
#define ADD_DISTANCE            0.1
#define TAKEOFF_STEP_SPIN		0
#define TAKEOFF_STEP_NORMAL		1

static float takeoff_spin_time;
static float takeoff_spin_timeout = 1.0f;
static float takeoff_spin;
static float takeoff_alt;
static float takeoff_rc_check;
static float takeoff_vel_limit;
static uint8_t takeoff_step;

static float tkof_pos_sp[3] = {0.0f};
bool takeoff_flag_alt=false;
bool takeoff_flag_pos=false;

bool control_takeoff_check()
{
	uint8_t check = CONTROL_CHECK_ATT | CONTROL_CHECK_ALT | CONTROL_CHECK_DISARM;

	if(control_check(check) == true){
		if(system_set_armed(true) == true){
			return true;
		}
	}

	return false;
}

void control_takeoff_param_init()
{
	param_set_var(CON__TAKEOFF__SPIN_NAME		,&takeoff_spin);
	param_set_var(CON__TAKEOFF__ALT_NAME		,&takeoff_alt);
	param_set_var(CON__TAKEOFF__VEL_LIMIT_NAME	,&takeoff_vel_limit);
	param_set_var(CON__TAKEOFF__RC_CHECK_NAME	,&takeoff_rc_check);
}

void set_takeoff_alt(int alt)
{

  takeoff_alt = (float)alt/100;

}

void control_takeoff_exit()
{
	inav_baro_set_normal_mode();
	inav_flow_set_normal_mode();
	att_control_set_att_pid_normal_mode();
	pos_control_set_vel_pid_normal_mode();
	flow_set_dtg(0.7f,2.0f);
	INFO(DEBUG_ID,"takeoff exit");
}

//static bool alt_t_flag = false;
void control_takeoff_init(float param1,float param2)
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
	set_takeoff_flag(0);
    set_power_zero();
	//debug_t("pos_init pos.pos_ned[2] = %f tof = %f \n",pos.pos_ned[2],get_tof_data_yaw());
	INFO(DEBUG_ID,"takeoff init");
}
void set_takeoff_flag_alt()
{
	takeoff_flag_alt=false;
}
bool get_takeoff_flag()
{
	return takeoff_flag_alt;
}
void set_takeoff_flag_poshold()
{
	takeoff_flag_pos=false;
}
bool get_takeoff_flag_poshold()
{
	return takeoff_flag_pos;
}

void control_takeoff_update(float dt,rc_s * rc)
{
	float vel_sp[3] = {0};
	nav_s pos;
	att_s att;
	uint8_t count = 0;
	//float alt_z=0;
	takeoff_flag_alt=true;
	takeoff_flag_pos=true;
	nav_get_pos(&pos);
	attitude_get(&att);

	if(takeoff_step == TAKEOFF_STEP_SPIN){
		float motor[MAX_NUM_MOTORS];

		for(count = 0 ; count < MAX_NUM_MOTORS ;count++){
			motor[count] = takeoff_spin;
		}
		motor_set_test(false,0.01f,motor);
		takeoff_spin_time += dt;
		
		if(takeoff_spin_time >= takeoff_spin_timeout){
			debug_t("pos_init**** pos.pos_ned[2] = %f tof = %f vel = %f\n",pos.pos_ned[2],get_tof_data_yaw(),takeoff_vel_limit);
			tkof_pos_sp[2] = pos.pos_ned[2] + takeoff_alt;
#if 0
			if(pos.pos_ned[2] < -0.1){
             pos.pos_ned[2]=fabs(pos.pos_ned[2]);
		    }
		    alt_z=(pos.pos_ned[2]);
		  //  printf("alt_z = %f pos = %f \n",alt_z,pos.pos_ned[2]);
            if(alt_z > 0.1 && alt_z < 0.3){
            tkof_pos_sp[2] =  takeoff_alt - alt_z + ADD_DISTANCE;
			}else if(alt_z > 0.3 && alt_z < 0.5){
            tkof_pos_sp[2] =  takeoff_alt - alt_z + 0.2;   
			}else if(alt_z > 0.5 && alt_z < 0.8){
            tkof_pos_sp[2] =  takeoff_alt - alt_z + 0.4;         
			}else if(alt_z > 0.8){
			tkof_pos_sp[2] =  takeoff_alt - alt_z + 0.6; 
			}else{
		    tkof_pos_sp[2] =  takeoff_alt - alt_z;
            }
#endif
			debug_t("take_sp = %f\n",tkof_pos_sp[2]);
			inav_baro_set_takeoff_mode();
			inav_flow_set_takeoff_mode();

			pos_control_set_pos_ned_z_target(tkof_pos_sp[2]);
			control_set_rf_alt_target(takeoff_alt);
			
			//att_control_set_att_pid_takeoff_mode();
			pos_control_set_vel_pid_takeoff_mode();
			takeoff_step = TAKEOFF_STEP_NORMAL;
			INFO(DEBUG_ID,"takeoff now");
		}
	}else{
		if(fabs(rc->pitch) > 0 || fabs(rc->roll) > 0){
			//att_control_set_att_pid_normal_mode();
			//pos_control_set_vel_pid_normal_mode();
			//control_att_common_rc(dt,rc);
			
			control_set_normal_mode();

		}else{
			if(pos.pos_valid){
				//control_pos_ned_xy_common(dt,tkof_pos_sp,0,0.5f);
				control_vel_ned_xy_common(dt,vel_sp,0,0.5f);
			}else{
				control_att_common_rc(dt,rc);
			}
		}
		
		control_yaw_common_rc(dt,rc);
		control_alt_pos_common(dt,tkof_pos_sp[2],takeoff_vel_limit);
       
		nav_att_offset_update();		
#if 1
		if(fabs(rc->thr_raw) > takeoff_rc_check || fabs(pos.pos_ned[2] - tkof_pos_sp[2]) < 0.10f){
			debug_t("pos pos.pos_ned[2] = %f tof = %f \n",pos.pos_ned[2],get_tof_data_yaw());
			control_set_normal_mode();
		}
#endif        
        if(get_tof_data_yaw() > takeoff_alt ){
			debug_t("tof pos.pos_ned[2] = %f tof = %f \n",pos.pos_ned[2],get_tof_data_yaw());
           control_set_normal_mode();
		}
		
		if(rangefinder_get_status() == SENSOR_STATUS_OK && fabs(rangefinder_get_range_f() - takeoff_alt) < 0.10f){
			control_set_normal_mode();
		}
	}
}

