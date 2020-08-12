#include "app_param.h"
#include "app_param_calib.h"
#include "app_imu_calib.h"
#include "app_debug.h"
#include "app_sensors.h"
#include "app_system.h"
#include "app_attitude.h"
#include "app_nav.h"

#include "lib_lpf2p.h"	
#include "lib_math.h"
#include "lib_rotate.h"

#include "hal.h"

#define DEBUG_ID DEBUG_ID_IMU

lpf2p_s imu_acc_filter[3];
lpf2p_s imu_gyro_filter[3];

static float imu_rorate;

float imu_acc_xy_filte;
float imu_acc_z_filte;

float imu_temp;
float imu_acc[3] = {0};
float imu_acc_raw[3] = {0};
float imu_acc_filted[3] = {0};
float imu_gyro[3] = {0};

float imu_acc_offset[3] = {0};
float imu_acc_scale[3] = {0};
float imu_gyro_offset[3] = {0};
float imu_gyro_scale[3] = {0};

bool imu_gyro_cal = false;
float imu_gyro_cal_thr = 0.30f;
float imu_gyro_cal_time = 0.0f;
float imu_gyro_cal_timeout = 1.0f;

bool imu_update_val;

float imu_error_timeout = 0.5f;
float imu_error_time = 0.0f;
float imu_acc_calib_status = 0.0f;

bool imu_acc_calibing = false;

float imu_get_error_time()
{
	return imu_error_time;
}

void imu_set_acc_calib(bool start)
{
	if(system_get_armed() == false){
		if(imu_acc_calibing == true && start == true){
			imu_calib_acc_confirm_step();
		}else if(imu_acc_calibing == false && start == true){
			imu_calib_acc_start();
			imu_acc_calibing = true;
		}else if(imu_acc_calibing == true && start == false){
			imu_acc_calibing = false;
		}		
	}
}

uint8_t imu_get_acc_calib_status()
{
	return imu_calib_acc_get_step();
}

bool imu_need_acc_calib_check()
{
	if(float_is_zero(imu_acc_calib_status) == true){
		return true;
	}else{
		return false;
	}
}

uint8_t imu_get_acc_status()
{
	if(imu_error_time >= imu_error_timeout){
		return SENSOR_STATUS_FAIL;
	}else if(imu_acc_calibing == true){
		return SENSOR_STATUS_CALIBING;
	}else if(imu_need_acc_calib_check() == true){
		return SENSOR_STATUS_NEED_CALIB;
	}else{
		return SENSOR_STATUS_OK;
	}
}

uint8_t imu_get_gyro_status()
{
	if(imu_error_time >= imu_error_timeout){
		return SENSOR_STATUS_FAIL;
	}else if(imu_gyro_cal == false){
		return SENSOR_STATUS_INIT;
	}else{
		return SENSOR_STATUS_OK;
	}
}

float imu_get_temp()
{
	return imu_temp;
}

bool imu_get_update()
{
	return imu_update_val;
}

void imu_set_acc_cal(float offset[3],float scale[3])
{
	param_calib_set(ACC_OFFSET_X_NAME,offset[0]);
	param_calib_set(ACC_OFFSET_Y_NAME,offset[1]);
	param_calib_set(ACC_OFFSET_Z_NAME,offset[2]);

	param_calib_set(ACC_SCALE_X_NAME,scale[0]);
	param_calib_set(ACC_SCALE_Y_NAME,scale[1]);
	param_calib_set(ACC_SCALE_Z_NAME,scale[2]);

	imu_acc_calib_status = 1.0;
	param_calib_set(ACC_CALIB_STATUS_NAME,imu_acc_calib_status);

	param_calib_save();
}

void imu_get_acc(float acc[3])
{
	v3f_set(acc,imu_acc);
}

void imu_get_acc_raw(float acc[3])
{
	v3f_set(acc,imu_acc_raw);
}

void imu_get_acc_filted(float acc[3])
{
	v3f_set(acc,imu_acc_filted);
}

void imu_get_gyro_bias(float gyro_b[3])
{
	v3f_set(gyro_b,imu_gyro_offset);
}

void imu_get_gyro(float gyro[3])
{
	v3f_set(gyro,imu_gyro);
}

void imu_param_init(void)
{
	param_calib_define(ACC_OFFSET_X_NAME,ACC_OFFSET_X_DEF,&imu_acc_offset[0]);
	param_calib_define(ACC_OFFSET_Y_NAME,ACC_OFFSET_Y_DEF,&imu_acc_offset[1]);
	param_calib_define(ACC_OFFSET_Z_NAME,ACC_OFFSET_Z_DEF,&imu_acc_offset[2]);

	param_calib_define(ACC_SCALE_X_NAME,ACC_SCALE_X_DEF,&imu_acc_scale[0]);
	param_calib_define(ACC_SCALE_Y_NAME,ACC_SCALE_Y_DEF,&imu_acc_scale[1]);
	param_calib_define(ACC_SCALE_Z_NAME,ACC_SCALE_Z_DEF,&imu_acc_scale[2]);
	param_calib_define(ACC_CALIB_STATUS_NAME,ACC_CALIB_STATUS,&imu_acc_calib_status);

	param_set_var(IMU__ACC_XY_LOW_FILTER_NAME	,&imu_acc_xy_filte);
	param_set_var(IMU__ACC_Z_LOW_FILTER_NAME	,&imu_acc_z_filte);

	param_set_var(IMU__GYRO_SCALE_X_NAME		,&imu_gyro_scale[0]);
	param_set_var(IMU__GYRO_SCALE_Y_NAME		,&imu_gyro_scale[1]);
	param_set_var(IMU__GYRO_SCALE_Z_NAME		,&imu_gyro_scale[2]);

	param_set_var(IMU__ROTATION_NAME			,&imu_rorate);
	
	imu_calib_acc_param_init();
}

void imu_init(void)
{
	INFO(DEBUG_ID,"init rotate:%d calib(%3.3f,%3.3f,%3.3f)",(uint8_t)imu_rorate,imu_acc_offset[0],imu_acc_offset[1],imu_acc_offset[2]);
	if(fabs(imu_acc_offset[0]) > 0.5f || fabs(imu_acc_offset[1]) > 0.5f || fabs(imu_acc_offset[2]) > 0.5f){
		ERR(DEBUG_ID,"acc calib value too large");
	}
	
	lpf2p_init(&imu_acc_filter[0],(float)MAIN_LOOP_HZ, imu_acc_xy_filte);
	lpf2p_init(&imu_acc_filter[1],(float)MAIN_LOOP_HZ, imu_acc_xy_filte);
	lpf2p_init(&imu_acc_filter[2],(float)MAIN_LOOP_HZ, imu_acc_z_filte);

	lpf2p_init(&imu_gyro_filter[0],(float)MAIN_LOOP_HZ, 1.0f);
	lpf2p_init(&imu_gyro_filter[1],(float)MAIN_LOOP_HZ, 1.0f);
	lpf2p_init(&imu_gyro_filter[2],(float)MAIN_LOOP_HZ, 1.0f);

	imu_gyro_cal_time = 0.0f;
	imu_gyro_cal = false;

	imu_update_val = false;

	imu_calib_acc_init();
}

void imu_calib_gyro_update(float dt,float gyro[3])
{	
	if(system_get_armed() == false && pythagorous_v3f(gyro) <= imu_gyro_cal_thr){
		imu_gyro_offset[0] = lpf2p_update(&imu_gyro_filter[0],gyro[0]);
		imu_gyro_offset[1] = lpf2p_update(&imu_gyro_filter[1],gyro[1]);
		imu_gyro_offset[2] = lpf2p_update(&imu_gyro_filter[2],gyro[2]);

		if(imu_gyro_cal == false){
			imu_gyro_cal_time += dt;
			if(imu_gyro_cal_time >= imu_gyro_cal_timeout){
				INFO(DEBUG_ID,"gyro cal done:%3.3f,%3.3f,%3.3f",imu_gyro_offset[0],imu_gyro_offset[1],imu_gyro_offset[2]);
				imu_gyro_cal = true;
			}
		}
	}else{
		imu_gyro_cal_time = 0.0f;
	}
}

void imu_update(float dt)
{
	uint8_t i;
	float acc[3];
	float gyro[3];

	if(hal_get_imu(dt,acc,gyro,&imu_temp) == true){

		gyro[0] *= imu_gyro_scale[0];
		gyro[1] *= imu_gyro_scale[1];
		gyro[2] *= imu_gyro_scale[2];
		
		rotate3(acc,(uint8_t)imu_rorate);
		rotate3(gyro,(uint8_t)imu_rorate);

		v3f_set(imu_acc_raw,acc);
		v3f_set(imu_gyro,gyro);
		
		for(i = 0;i < 3;i++){
			imu_acc[i] = imu_acc_raw[i] * imu_acc_scale[i];
			imu_acc[i] = imu_acc[i] - imu_acc_offset[i];
			imu_acc_filted[i] = lpf2p_update(&imu_acc_filter[i],imu_acc[i]);
			
			imu_gyro[i] -= imu_gyro_offset[i];
		}
		
		imu_calib_gyro_update(dt,gyro);
		imu_calib_acc_update();
		if(imu_calib_acc_get_status() == IMU_CALIB_ACC_STATUS_EXIST){
			imu_calib_acc_set_status_null();
			nav_reset();
			attitude_reset();
			imu_acc_calibing = false;
		}

		imu_update_val = true;
		imu_error_time = 0.0f;
	}else{
		imu_error_time += dt;
		imu_update_val = false;
	}
	
	//DEBUG_HZ(DEBUG_ID,5,dt,"gyro:(%3.3f,%3.3f,%3.3f)(%3.3f,%3.3f,%3.3f)-(%3.3f,%3.3f,%3.3f) %3.3f",imu_gyro[0],imu_gyro[1],imu_gyro[2],gyro[0],gyro[1],gyro[2],imu_gyro_offset[0],imu_gyro_offset[1],imu_gyro_offset[2],pythagorous_v3f(imu_gyro));
	DEBUG_HZ(DEBUG_ID,5,dt,"acc:(%3.3f,%3.3f,%3.3f)(%3.3f,%3.3f,%3.3f) gyro:(%3.3f,%3.3f,%3.3f)(%3.3f,%3.3f,%3.3f) temp:%3.3f",imu_acc[0],imu_acc[1],imu_acc[2],acc[0],acc[1],acc[2],imu_gyro[0],imu_gyro[1],imu_gyro[2],gyro[0],gyro[1],gyro[2],imu_temp);
}

