#include <stdbool.h>

#include "app_debug.h"
#include "app_rangefinder.h"
#include "app_sensors.h"
#include "app_param.h"
#include "app_attitude.h"
#include "app_nav.h"

#include "hal.h"
#include "lib_rotate.h"
#include "iir_filter.h"
#include "lib_lpf2p.h"

#define DEBUG_ID DEBUG_ID_FLOW

#define FLOW_GYRO_BUF_LEN  100

float flow2_vel_raw[2] = {0.0f,0.0f};
float flow2_quality = 0.0f;
uint16_t flow2_version = 0;

float flow3_vel_raw[2] = {0.0f,0.0f};
float flow3_quality = 0.0f;
uint16_t flow3_version = 0;

static float flow_vel_raw[2] = {0.0f,0.0f};
static float flow_vel_scale[2] = {0.0f,0.0f};
static float flow_vel[2] = {0.0f,0.0f};
static float flow_quality = 0.0f;
static float flow_quality_f = 0.0f;
static uint16_t flow_version = 0;

static bool flow_update_val = false;
static bool flow_init_check = false;
static float flow_error_time = 0.0f;
static float flow_error_timeout = 5.0f;

static iir_filter gyro_filter[3];
static float flow_gyro[3];
static float flow_delay;
static float flow_scale;
static int flow_gyro_r_ptr = 0;
static int flow_gyro_w_ptr = 0;
static float flow_gyro_buffer[3][FLOW_GYRO_BUF_LEN + 5];

static float flow_dtg;
static float flow_dtg_min;
static float flow_dtg_max;

static float flow_rorate;
static lpf2p_s flow_qua_lpf;

float flow_get_error_time()
{
	return flow_error_time;
}

uint8_t flow_get_status()
{
	if(flow_error_time >= flow_error_timeout){
		return SENSOR_STATUS_FAIL;
	}else if(flow_init_check == false){
		return SENSOR_STATUS_INIT;
	}else{
		return SENSOR_STATUS_OK;
	}
}

bool flow_get_update()
{
	return flow_update_val;
}

void flow_get_vel(float vel[2])
{
	vel[0] = flow_vel[0];
	vel[1] = flow_vel[1];
}

void flow2_get_vel_raw(float vel[2])
{
	vel[0] = flow2_vel_raw[0];
	vel[1] = flow2_vel_raw[1];
}

void flow3_get_vel_raw(float vel[2])
{
	vel[0] = flow3_vel_raw[0];
	vel[1] = flow3_vel_raw[1];
}

void flow_get_vel_raw(float vel[2])
{
	vel[0] = flow_vel_raw[0];
	vel[1] = flow_vel_raw[1];
}

void flow_get_vel_scale(float vel[2])
{
	vel[0] = flow_vel_scale[0];
	vel[1] = flow_vel_scale[1];
}

void flow_get_gyro(float gyro[2])
{
	gyro[0] = flow_gyro[0];
	gyro[1] = flow_gyro[1];
}

float flow_get_quality()
{
	return flow_quality;
}

float flow_get_quality_f()
{
	return flow_quality_f;
}

float flow_get_dtg()
{
	return flow_dtg;
}

void flow_set_dtg(float min,float max)
{
	flow_dtg_min = min;
	flow_dtg_max = max;
}

uint16_t flow_get_version()
{
	return flow_version;
}

void flow_param_init(void)
{
	param_set_var(FLOW__DELAY_NAME		,&flow_delay);
	param_set_var(FLOW__SCALE_NAME		,&flow_scale);
	
	param_set_var(FLOW__ROTATION_NAME	,&flow_rorate);
}

void flow_init(void)
{
	INFO(DEBUG_ID,"init rotate:%d",(uint8_t)flow_rorate);
	
	iir_filter_init(&gyro_filter[0],CHEBY2,IIR_LOWPASS,10,20,(float)MAIN_LOOP_HZ);
	iir_filter_init(&gyro_filter[1],CHEBY2,IIR_LOWPASS,10,20,(float)MAIN_LOOP_HZ);
	iir_filter_init(&gyro_filter[2],CHEBY2,IIR_LOWPASS,10,20,(float)MAIN_LOOP_HZ);
	
	lpf2p_init(&flow_qua_lpf,(float)MAIN_LOOP_HZ,40.0f);
	
	flow_set_dtg(0.2f,2.0f);
}

void flow_update(float dt)
{
	att_s att;

	flow_error_time += dt;
	
	attitude_get(&att);
	if(att.valid == true){
		flow_gyro_buffer[0][flow_gyro_w_ptr % FLOW_GYRO_BUF_LEN] = signal_iir_filter(&gyro_filter[0],att.rate[0]);
		flow_gyro_buffer[1][flow_gyro_w_ptr % FLOW_GYRO_BUF_LEN] = signal_iir_filter(&gyro_filter[1],att.rate[1]);
		flow_gyro_buffer[2][flow_gyro_w_ptr % FLOW_GYRO_BUF_LEN] = signal_iir_filter(&gyro_filter[2],att.rate[2]);
		flow_gyro_w_ptr++;
	}

	//hal_get_flow2(dt,flow2_vel_raw,&flow2_quality,&flow2_version);
	//hal_get_flow3(dt,flow3_vel_raw,&flow3_quality,&flow3_version);
	
	if(hal_get_flow(dt,flow_vel_raw,&flow_quality,&flow_version) == true){
		rotate2(flow_vel_raw,(uint8_t)flow_rorate);
		
		flow_quality_f = lpf2p_update(&flow_qua_lpf,flow_quality);
		if(flow_quality < flow_quality_f){
			flow_quality_f = flow_quality; 
		}
		
		flow_vel_scale[0] = flow_vel_raw[0] * flow_scale;
		flow_vel_scale[1] = flow_vel_raw[1] * flow_scale;
		
		flow_gyro_r_ptr = flow_gyro_w_ptr - (flow_delay * MAIN_LOOP_HZ);
		if(flow_gyro_r_ptr < 0){
			flow_gyro_r_ptr = 0;
		}

		flow_gyro[0] = flow_gyro_buffer[0][flow_gyro_r_ptr % FLOW_GYRO_BUF_LEN];
		flow_gyro[1] = flow_gyro_buffer[1][flow_gyro_r_ptr % FLOW_GYRO_BUF_LEN];
		flow_gyro[2] = flow_gyro_buffer[2][flow_gyro_r_ptr % FLOW_GYRO_BUF_LEN];
		
		flow_vel[0] =  flow_vel_scale[0] - flow_gyro[1];
		flow_vel[1] =  flow_vel_scale[1] + flow_gyro[0];
		
		if(rangefinder_get_status() == SENSOR_STATUS_OK){
			flow_dtg = constrain_float(rangefinder_get_range_f(),0.2f,2.0f);
		}else{
			flow_dtg = constrain_float(nav_get_dis_to_grd(),flow_dtg_min,flow_dtg_max);
		}

		flow_vel[0] *= flow_dtg;
		flow_vel[1] *= flow_dtg;
		
		flow_error_time = 0.0f;
		flow_update_val = true;
		flow_init_check = true;
	}else{
		flow_update_val = false;
	}
	
	DEBUG_HZ(DEBUG_ID,10,dt,"vel(%3.3f,%3.3f) quality:%3.3f version:%d",flow_vel[0],flow_vel[1],flow_quality,flow_version);
	//DEBUG_HZ(DEBUG_ID,10,dt,"vel(%3.3f,%3.3f) vel2(%3.3f,%3.3f) vel3(%3.3f,%3.3f) quality:%3.3f version:%d",flow_vel[0],flow_vel[1],flow2_vel_raw[0],flow2_vel_raw[1],flow3_vel_raw[0],flow3_vel_raw[1],flow_quality,flow_version);
}



