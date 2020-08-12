#include "app_param.h"
#include "app_debug.h"
#include "app_sensors.h"
#include "app_attitude.h"

#include "lib_math.h"
#include "hal.h"
#include "param.h"

#define DEBUG_ID DEBUG_ID_RANGEFINDER

float rf_dist_f;
float rf_dist_old_f;
float rf_dist;
float rf_vel;

float rf_min_val;
float rf_max_val;
float rf_rotate_check;
float rf_rotate;

bool rf_val_check;
bool rf_update_val;

float rf_error_timeout = 0.2f;
float rf_error_time = 0.0f;

float rangefinder_get_error_time()
{
	return rf_error_time;
}

uint8_t rangefinder_get_status()
{
	if(rf_error_time >= rf_error_timeout || rf_val_check == false){
		return SENSOR_STATUS_FAIL;
	}else{
		return SENSOR_STATUS_OK;
	}
}

bool rangefinder_get_update()
{
	return rf_update_val;
}

float rangefinder_get_rotate()
{
	return rf_rotate;
}

float rangefinder_get_range()
{
	return rf_dist;
}

float rangefinder_get_range_f()
{
	return rf_dist_f;
}

float rangefinder_get_vel()
{
	return rf_vel;
}

void rangefinder_param_init(void)
{
	param_set_var(RANGEFINDER__MAX_VAL_NAME			,&rf_max_val);
	param_set_var(RANGEFINDER__MIN_VAL_NAME			,&rf_min_val);
	param_set_var(RANGEFINDER__ROTATE_CHECK_NAME	,&rf_rotate_check);
}

void rangefinder_init(void)
{
	INFO(DEBUG_ID,"init");
	rf_update_val = false;
	rf_val_check = false;
}

void rangefinder_update(float dt)
{
	static float dt_sum;
	float dist;
	att_s att;

	dt_sum += dt;

	if(hal_get_rangefinder(dt,&dist) == true){
		attitude_get(&att);
		if(att.valid == false){
			return;
		}

		rf_rotate = att.r[2][2];

		rf_dist = dist * att.r[2][2];
		rf_dist_f = complementary_filter(rf_dist_f,rf_dist,0.30f);
		rf_vel = (rf_dist_f - rf_dist_old_f) / dt_sum;
		rf_dist_old_f = rf_dist_f;

               // DEBUG(DEBUG_ID,"rf_dist=%f att.r[2][2]=%f rf_dist_f=%f\n",rf_dist,att.r[2][2],rf_dist_f);
		dt_sum = 0.0f;
		
		if(rf_dist >= rf_min_val && rf_dist <= rf_max_val && att.r[2][2] >= rf_rotate_check){
			rf_error_time = 0.0f;
			rf_update_val = true;
			rf_val_check = true;
		}else{
			rf_error_time += dt;
			rf_update_val = false;
			rf_val_check = false;
		}
		
		DEBUG(DEBUG_ID,"range:(%3.3f,%3.3f) vel:%3.3f rotate:%3.3f",dist,rf_dist,rf_vel,att.r[2][2]);
	}else{
             //   DEBUG(DEBUG_ID,"tof get data error\n");
		rf_error_time += dt;
		rf_update_val = false;
	}
}


