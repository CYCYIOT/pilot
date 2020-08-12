#include "app_param.h"
#include "app_param_calib.h"
#include "app_sensors.h"
#include "app_debug.h"
#include "app_mag.h"
#include "app_imu.h"
#include "app_mag_calib.h"
#include "app_system.h"

#include "lib_math.h"
#include "lib_rotate.h"

#include "hal.h"

#define DEBUG_ID DEBUG_ID_MAG
#define MAG_CALIB_CHECK_NUM        100
#define GYRO_SPEED                  1
#define MAG_CALIB_TIME              1

float mag_val[3] = {0};
float mag_offset[3] = {0};

static float mag_rorate;

bool mag_update_val;

float mag_error_timeout = 0.5f;
float mag_error_time = 0.0f;
float mag_calib_status = 0.0f;

bool mag_calibing = false;

calibrator mag_calibrator;

static float mag_health_timer = 0.0f;
static uint8_t horizontal_flag = 0;
static uint8_t vertical_flag = 0;
static uint8_t mag_calib_succeed_num = 0;
static uint8_t mag_calib_init_flag = 0;
static uint8_t mag_calib_check_flag = 0;
static uint8_t mag_calib_check_cnt = 0;
static uint8_t mag_calib_checking = 0;
static uint8_t mag_calib_progress_flag = 0;
static float gyro[3] = {0.0f};
static float mag[3]= {0.0f};

float mag_get_error_time()
{
	return mag_error_time;
}

bool get_mag_calib_status()
{
	if(mag_get_calib_progress() >= 98 && mag_calib_progress_flag == 0){
		 mag_calib_checking = true;
		 mag_calib_progress_flag = 1;
	}	 
	return mag_calib_checking;
}	

void mag_calib_init()
{
	horizontal_flag = 0;
	vertical_flag = 0;
	mag_calib_succeed_num = 0;
	mag_calib_init_flag = 0;
	mag_calib_check_flag = 0;
	mag_calib_checking = 0;
	mag_calib_progress_flag = 0;
	mag_calib_check_cnt = 0;
}

bool mag_calib_status_check(float dt)
{
	static float rotation_time = 0.0f;
	if(horizontal_flag == 0){
		imu_get_gyro(gyro);		
		if(fabs(gyro[2]) > GYRO_SPEED){
			rotation_time += dt;
			if(rotation_time > MAG_CALIB_TIME){
				INFO(DEBUG_ID,"[CALIB] Horizon calib done");
				horizontal_flag = 1;
				rotation_time = 0.0f;
			}
		}	
	}
	else{
		imu_get_gyro(gyro);
		if(fabs(gyro[0]) > GYRO_SPEED && vertical_flag == 0){
			rotation_time += dt;			
			if(rotation_time > MAG_CALIB_TIME){
				INFO(DEBUG_ID,"[CALIB] Vertical calib done");
				vertical_flag = 1;
				rotation_time = 0.0f;
			}	
		}		
	}
	if(vertical_flag)
		return true;
	else
		return false;
}

void mag_calib_result_checking()
{
	if(vertical_flag == 1){
		if(mag_calib_check_flag == 0 && mag_calib_check_cnt < MAG_CALIB_CHECK_NUM){
			mag_get_val(mag);
			if(mag_calib_check(mag)){
				mag_calib_succeed_num ++;
			}			
			mag_calib_check_cnt ++; 	
			if(mag_calib_check_cnt == MAG_CALIB_CHECK_NUM){
				INFO(DEBUG_ID,"[CALIB]check_cnt done");
				mag_calib_check_flag = 1;
				mag_calib_checking = 0;
				INFO(DEBUG_ID,"[CALIB]mag calibcheck done"); 
				INFO(DEBUG_ID,"[CALIB]mag calib check result %f",(float)mag_calib_succeed_num/(float)MAG_CALIB_CHECK_NUM * 100.0f);
			}		
		}
	}else if(mag_calib_check_flag == 0){
		mag_calib_check_flag = 1;
		mag_calib_checking = 0;				 
	}
}

bool mag_calib_check_result()
{
	if(mag_calib_succeed_num >= 0.9f * MAG_CALIB_CHECK_NUM){
		return true;
	}else{
		return false;
	}
	return false;
}

uint8_t mag_get_calib_progress ()
{
	return mag_callib_get_progress();
}

void mag_set_calib(bool start)
{
	if(system_get_armed() == false){
		if(start == true){
			mag_calibrator.status = start_calibration;
		}else{
			mag_calibrator.status = calibrated;
		}
	}
}

bool mag_need_calib_check()
{
	if(mag_calibrator.status == need_calibrate){
		return true;
	}
	return false;
}

uint8_t mag_get_status()
{
	if(mag_error_time >= mag_error_timeout){
		return SENSOR_STATUS_FAIL;
	}else if(mag_calibrator.status == calibrating){
		return SENSOR_STATUS_CALIBING;
	}else if(mag_need_calib_check() == true || mag_calibrator.status == need_calibrate || mag_calib_status == 0 ){
		return SENSOR_STATUS_NEED_CALIB;
	}else{
		return SENSOR_STATUS_OK;
	}
}

bool mag_get_update()
{
	return mag_update_val;
}

void mag_set_cal_offset(float offset[3])
{
	v3f_set(mag_offset,offset);
}

void mag_get_val(float mag[3])
{
	v3f_set(mag,mag_val);
}

void mag_health_check(float dt,float mag[3])
{
	if(mag_calib_check(mag) == false){
		mag_error_time += dt;
	}else{
		mag_error_time -= dt * 0.2f;
	}
	if(mag_error_time < 0.0f){
		mag_error_time = 0.0f;
	}
	if(mag_error_time > 2.0f){
		mag_error_time = 0.0f;
		mag_calibrator.status = need_calibrate;
	}
}

void mag_param_init(void)
{
	mag_calib_define_param();
	
	param_calib_define(MAG_CALIB_STATUS_NAME,MAG_CALIB_STATUS,&mag_calib_status);
	
	param_set_var(MAG__ROTATION_NAME,&mag_rorate);
}

void mag_init(void)
{
	INFO(DEBUG_ID,"init rotate:%d",(uint8_t)mag_rorate);

	mag_calib_get_offset(mag_offset);
	INFO(DEBUG_ID,"mag offset:%4.4f,%4.4f,%4.4f",mag_offset[0],mag_offset[1],mag_offset[2]);

	mag_update_val = 0;

	mag_calibrator.status = calibrated;
}
void mag_calib_status_save()
{
	mag_calib_status = 1;
	param_calib_set(MAG_CALIB_STATUS_NAME,mag_calib_status);	
	param_calib_save();
}

void mag_update(float dt)
{
	uint8_t i;
	float mag[3];

	if(hal_get_mag(dt,mag) == true){
		for(i = 0;i < 3;i++){
			mag_val[i] = mag[i];
		}

		rotate3(mag_val,(uint8_t)mag_rorate);
		
		switch(mag_calibrator.status){
			case start_calibration:
				mag_calib_start();
				mag_calibrator.status = calibrating;
				INFO(DEBUG_ID,"mag calibration started");
				break;
			case calibrating:
				if(mag_calib_init_flag)
					mag_calib_init();
				mag_calib_status_check(dt);
				if(mag_calib_update(dt,mag_val) == true){
					mag_calibrator.status = calibrated;
					mag_calib_end();
					mag_calib_status_save();
					mag_calib_get_offset(mag_offset);
				}
				break;
			case calibrated:
				mag_calib_init_flag = 1;
				mag_val[0] -= mag_offset[0];
				mag_val[1] -= mag_offset[1];
				mag_val[2] -= mag_offset[2];
				mag_calib_result_checking();
				break;
			case need_calibrate:
				break;
			default:
				mag_calibrator.status = calibrated;
				break;
			
		};

		mag_health_timer += dt;
		if(mag_health_timer > 0.1f){  //10Hz check rate
			mag_health_check(mag_health_timer,mag);
			mag_health_timer = 0.0f;
		}
		
		mag_error_time = 0.0f;
		mag_update_val = true;
	}else{
		mag_error_time += dt;
		mag_update_val = false;
	}
	
	DEBUG_HZ(DEBUG_ID,5,dt,"mag:(%3.3f,%3.3f,%3.3f)",mag_val[0],mag_val[1],mag_val[2]);
}


