#include "app_imu.h"
#include "app_system.h"
#include "app_debug.h"
#include "app_control.h"
#include "app_param.h"
#include "app_attitude.h"
#include "app_batt.h"
#include "app_awlink.h"

#include "hal.h"
#include "hal_stm8s.h"

#define DEBUG_ID DEBUG_ID_SYSTEM

#define SYSTEM_AWLINK_HEART_HZ	20

bool system_run = true;

float system_att_limit;
float system_imu_temp_limit;
float system_batt_low;

system_status_s system_status;

uint16_t awlink_heart_count = 0;
float awlink_heart_rate;
float awlink_heart_dt_sum = 0.0f;

float awlink_heart_send_time = 0.0f;
float awlink_heart_time = 0.0f;
float awlink_heart_timeout = 2.0f;
bool awlink_heart_online = false;

float system_cpu_free;
float system_mem_free;

float system_get_cpu_free()
{
	return system_cpu_free;
}

float system_get_mem_free()
{
	return system_mem_free;
}

float system_get_awlink_heart_rate()
{
	return awlink_heart_rate;
}

void system_set_awlink_heart_count()
{
	awlink_heart_count++;
	awlink_heart_time = 0.0f;
	awlink_heart_online = true;
}

uint8_t system_get_status()
{
	return system_status.status;
}

bool system_get_run()
{
	return system_run;
}

void system_set_run(bool run)
{
	system_run = run;
	INFO(DEBUG_ID,"system_set_run:%d",system_run);
}

bool system_get_awlink_online()
{
	return awlink_heart_online;
}

bool system_get_att_limit()
{
	return system_status.att_limit;
}

bool system_get_imu_temp_limit()
{
	return system_status.imu_temp_limit;
}

bool system_get_batt_low()
{
	return system_status.batt_low;
}

bool system_get_armed()
{
	return system_status.armed;
}

bool system_armed_check()
{
	//if(batt_get_charge() == BATT_CHARGING_STATUS_ON){
	//	INFO(DEBUG_ID,"batt charging");
	//	return false;
	//}

	if(system_status.att_limit == true){
		INFO(DEBUG_ID,"att limit");
		return false;
	}

	if(system_status.batt_low == true){
		INFO(DEBUG_ID,"batt low");
		return false;
	}

	return true;
}

bool system_set_armed(bool armed)
{
	if(armed == true){
		if(system_armed_check() == true){
			system_status.armed = true;
			INFO(DEBUG_ID,"armed");
		}
	}else{
		system_status.armed = false;
		INFO(DEBUG_ID,"disarm");
	}
	return system_status.armed;
}

void system_param_init(void)
{
	param_set_var(FS__ATT_LIMIT_NAME		,&system_att_limit);      //60f
	param_set_var(FS__IMU_TEMP_LIMIT_NAME	,&system_imu_temp_limit);  //80f
	param_set_var(FS__LOW_BATT_NAME			,&system_batt_low);     //10f
}

void system_init(void)
{
	INFO(DEBUG_ID,"init");

	system_run = true;

	system_status.status = SYSTEM_STATUS_INIT;
	system_status.armed = false;
	system_status.att_limit = false;
	system_status.imu_temp_limit = false;
	system_status.batt_low = false;
}

void system_update(float dt)
{
	awlink_heart_time += dt;
	awlink_heart_dt_sum += dt;
	awlink_heart_send_time += dt;

	if(awlink_heart_send_time >= (1.0f / SYSTEM_AWLINK_HEART_HZ)){
		awlink_heart();           //AflingÒ£¿Ø²»ÄÜÊ¹ÓÃ
		awlink_heart_send_time = 0.0f;
	}

	if(awlink_heart_time >= awlink_heart_timeout){
		if(awlink_heart_online == true){
			awlink_heart_online = false;
			INFO(DEBUG_ID,"awlink offline");
		}
	}
	
	if(awlink_heart_dt_sum >= 1.0f){
		awlink_heart_rate = (float)awlink_heart_count / awlink_heart_dt_sum;
		awlink_heart_dt_sum = 0.0f;
		awlink_heart_count = 0;
	}

	if(fabs(attitude_get_att_roll()) > system_att_limit || fabs(attitude_get_att_pitch()) > system_att_limit){
		system_status.att_limit = true;
	}else{
		system_status.att_limit = false;
	}

	if(imu_get_temp() > system_imu_temp_limit){
		system_status.imu_temp_limit = true;
	}else{
		system_status.imu_temp_limit = false;
	}

	if(batt_get_cap() < (uint8_t)system_batt_low){
		system_status.batt_low = true;
	}else{
		system_status.batt_low = false;
	}

	//hal_get_cpu_free(dt,&system_cpu_free);
	//hal_get_mem_free(dt,&system_mem_free);
	
	DEBUG_HZ(DEBUG_ID,1,dt,"CPU:%3.3f MEM:%3.3f heart rate:%3.3f",system_cpu_free,system_mem_free,awlink_heart_rate);
}

