#include "app_param.h"
#include "app_debug.h"
#include "app_rc.h"
#include "app_system.h"
#include "app_control.h"
#include "app_attitude.h"
#include "app_mag.h"
#include "app_imu.h"

#include "lib_lpf2p.h"
#include "lib_math.h"

#include "hal.h"

#define DEBUG_ID DEBUG_ID_RC

#define RC_LIMIT 1.0f

float rc_mode = CONTROL_MODE_STOP;

rc_s rc;

float rc_remote[5];
float rc_awlink[4];
float rc_raw[4];
float rc_f[4];

bool rc_remote_check = false;
bool rc_awlink_check = false;
bool rc_control_check = false;

float rc_remote_update_time = 0;
float rc_remote_update_timeout = 1.0f;
float rc_awlink_update_time = 0;
float rc_awlink_update_timeout = 1.0f;

float rc_headfree_yaw;
bool rc_headfree = false;

lpf2p_s rc_filter[4];

float rc_dz_roll;
float rc_dz_pitch;
float rc_dz_yaw;
float rc_dz_thr;

float rc_mode_high_val = 1.0f;
float rc_mode_medium_val = 1.0f;
float rc_mode_low_val = 1.0f;

void rc_set_headfree(bool enable)
{
	rc_headfree = enable;
	INFO(DEBUG_ID,"headfree:%d",rc_headfree);
}

void rc_set_headfree_yaw(float yaw)
{
	rc_headfree_yaw = yaw;
}

bool rc_get_headfree()
{
	return rc_headfree;
}

void rc_param_init()
{
	param_set_var(RC__DEADZONE_ROLL_NAME	,&rc_dz_roll);    //0.05
	param_set_var(RC__DEADZONE_PITCH_NAME	,&rc_dz_pitch);
	param_set_var(RC__DEADZONE_YAW_NAME		,&rc_dz_yaw);
	param_set_var(RC__DEADZONE_THR_NAME		,&rc_dz_thr);
	
	param_set_var(RC__MODE_HIGH_NAME		,&rc_mode_high_val);
	param_set_var(RC__MODE_MEDIUM_NAME		,&rc_mode_medium_val);
	param_set_var(RC__MODE_LOW_NAME			,&rc_mode_low_val);
}

void rc_init(void)
{
	INFO(DEBUG_ID,"init");

	rc.mode = RC_MODE_HIGH;
	
	lpf2p_init(&rc_filter[0],(float)MAIN_LOOP_HZ,20.0f);
	lpf2p_init(&rc_filter[1],(float)MAIN_LOOP_HZ,20.0f);
	lpf2p_init(&rc_filter[2],(float)MAIN_LOOP_HZ,20.0f);
	lpf2p_init(&rc_filter[3],(float)MAIN_LOOP_HZ,20.0f);
}

void rc_mode_update(uint8_t mode)
{
	switch(mode){
		case 5:
			control_set_mode(CONTROL_MODE_TAKEOFF,0,0);
			break;
		case 6:
			control_set_mode(CONTROL_MODE_POSHOLD,0,0);
			break;
		case 9:
			control_set_mode(CONTROL_MODE_ALTHOLD,0,0);
			break;
		case 10:
			imu_set_acc_calib(true);
			break;
		case 11:
			control_set_mode(CONTROL_MODE_STABILIZE,0,0);
			break;
		case 15:
			control_set_mode(CONTROL_MODE_STOP,0,0);
			break;
		case 14:
			control_set_mode(CONTROL_MODE_LAND,0,0);
			break;
		case 8:
			mag_set_calib(true);
			break;	
	}
}

bool rc_get_update()
{
	return rc_control_check;
}

void rc_awlink_set_rc(float roll,float pitch,float yaw,float thr)
{
	rc_control_check = true;
	rc_awlink_check = true;
	rc_awlink_update_time = 0;
	
	rc_awlink[0] = roll;
	rc_awlink[1] = pitch;
	rc_awlink[2] = yaw;
	rc_awlink[3] = thr;
}
void rc_awlink_set_rc_h()
{

    rc_control_check = true;
	rc_awlink_check = true;
	rc_awlink_update_time = 0;
	
}

float rc_deadzone_calc(float in,float dz)
{
	float out = 0.0f;

	in = constrain_float(in,-RC_LIMIT,RC_LIMIT);

	if(in >= dz){
		in -= dz;
		in *= RC_LIMIT / (RC_LIMIT - dz);
		out = in;
	}else if(in <= -dz){
		in += dz;
		in *= RC_LIMIT / (RC_LIMIT - dz);
		out = in;
	}

	return out;
}

void rc_update(float dt)
{
	float scale = 1.0f;
	
	rc_awlink_update_time += dt;
	if(rc_awlink_update_time >= rc_awlink_update_timeout && rc_awlink_check == true){
		INFO(DEBUG_ID,"RC AWLINK LOST");
		rc_awlink_check = false;
	}
			
	if(hal_get_rc(dt,rc_remote) == true){
		rc_control_check = true;
		rc_remote_check = true;
		rc_remote_update_time = 0;		
	}else{
		rc_remote_update_time += dt;
		if(rc_remote_update_time >= rc_remote_update_timeout && rc_remote_check == true){
			INFO(DEBUG_ID,"RC REMOTE LOST");
			rc_remote_check = false;
		}
	}

	if(rc_remote_check == false && rc_awlink_check == false && rc_control_check == true){
		rc_control_check = false;
		ERR(DEBUG_ID,"RC LOST");
	}

	if(rc_remote_check == true){
		rc_raw[0]	= rc_remote[0];
		rc_raw[1]	= rc_remote[1];
		rc_raw[2]	= rc_remote[2];
		rc_raw[3]	= rc_remote[3];
		
		if(rc_mode != rc_remote[4]){
			rc_mode_update((uint8_t)rc_remote[4]);
			rc_mode = rc_remote[4];
		}
		
	}else if(rc_awlink_check == true){
		rc_raw[0]	= rc_awlink[0];
		rc_raw[1]	= rc_awlink[1];
		rc_raw[2]	= rc_awlink[2];
		rc_raw[3]	= rc_awlink[3];
	}else{
		rc_raw[0] = 0;
		rc_raw[1] = 0;
		rc_raw[2] = 0;
		rc_raw[3] = 0;
	}	

	if(rc.mode == RC_MODE_HIGH){
		scale = rc_mode_high_val;
	}else if(rc.mode == RC_MODE_MEDIUM){
		scale = rc_mode_medium_val;
	}else if(rc.mode == RC_MODE_LOW){
		scale = rc_mode_low_val;
	}

	if(rc_headfree == true){
		float yaw_diff = radians(attitude_get_att_yaw() - rc_headfree_yaw);
		float rc_rotate_roll;
		float rc_rotate_pitch;
		
		rc_rotate_roll  = rc_raw[1] * sinf(yaw_diff) + rc_raw[0] * cosf(yaw_diff);
		rc_rotate_pitch = rc_raw[1] * cosf(yaw_diff) - rc_raw[0] * sinf(yaw_diff);

		rc_raw[0] = rc_rotate_roll;
		rc_raw[1] = rc_rotate_pitch;
	}


	rc_f[0] = lpf2p_update(&rc_filter[0],rc_raw[0]);
	rc_f[1] = lpf2p_update(&rc_filter[1],rc_raw[1]);
	rc_f[2] = lpf2p_update(&rc_filter[2],rc_raw[2]);
	rc_f[3] = lpf2p_update(&rc_filter[3],rc_raw[3]);

	rc.roll_raw		= rc_deadzone_calc(rc_f[0],rc_dz_roll);
	rc.pitch_raw	= rc_deadzone_calc(rc_f[1],rc_dz_pitch);
	rc.yaw_raw		= rc_deadzone_calc(rc_f[2],rc_dz_yaw);
	rc.thr_raw		= rc_deadzone_calc(rc_f[3],rc_dz_thr);	

	rc.roll 	= rc.roll_raw * scale;
	rc.pitch	= rc.pitch_raw * scale;
	rc.yaw		= rc.yaw_raw * scale;
	rc.thr		= rc.thr_raw * scale;
	
	DEBUG_HZ(DEBUG_ID,2,dt,"%+3.3f,%+3.3f,%+3.3f,%+3.3f,%3.3f",rc.roll,rc.pitch,rc.yaw,rc.thr,rc_mode);	
}

void rc_get(rc_s * rc_tmp)
{
	memcpy(rc_tmp,&rc,sizeof(rc_s));
}

void rc_remote_get(float rc[5])
{
	rc[0] = rc_remote[0];
	rc[1] = rc_remote[1];
	rc[2] = rc_remote[2];
	rc[3] = rc_remote[3];
	rc[4] = rc_remote[4];
}

void rc_awlink_get(float rc[4])
{
	rc[0] = rc_awlink[0];
	rc[1] = rc_awlink[1];
	rc[2] = rc_awlink[2];
	rc[3] = rc_awlink[3];
}

void rc_set_mode(uint8_t mode)
{
	rc.mode = mode;
}

uint8_t rc_get_mode()
{
	return rc.mode;
}


