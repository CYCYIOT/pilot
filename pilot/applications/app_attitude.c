#include "app_attitude.h"
#include "app_log.h"
#include "app_mag.h"
#include "app_imu.h"
#include "app_mag_calib.h"
#include "app_sensors.h"
#include "app_debug.h"
#include "app_param.h"
#include "app_param_calib.h"
#include "app_gps.h"
#include "app_system.h"

#include "lib_math.h"
#include "lib_lpf2p.h"

#include "ahrs.h"
#include "mag_declination.h"

#define DEBUG_ID DEBUG_ID_ATTITUDE

att_s attitude;
ahrs_estimator ahrs_est;

float acc_avg[3] = {0};
float mag_avg[3] = {0};
float init_time = 0;
int init_count = 0;
bool reinit = false;

bool att_mag_init = false;

static bool mag_dec_inited = false;
static gps_info_s gps_info;
static float mag_dec = 0.0f;

float rate_rp_low_filter;
float rate_yaw_low_filter;

lpf2p_s rate_filter[3];

void attitude_param_init(void)
{
	param_set_var(ATT__RATE_RP_LOW_FILTER_NAME		,&rate_rp_low_filter);
	param_set_var(ATT__RATE_YAW_LOW_FILTER_NAME		,&rate_yaw_low_filter);

	ahrs_param_init();
}

void attitude_init(void)
{
	INFO(DEBUG_ID,"init");

	ahrs_est.inited = false;
	attitude.valid = false;
	
	lpf2p_init(&rate_filter[0],(float)MAIN_LOOP_HZ, rate_rp_low_filter);
	lpf2p_init(&rate_filter[1],(float)MAIN_LOOP_HZ, rate_rp_low_filter);
	lpf2p_init(&rate_filter[2],(float)MAIN_LOOP_HZ, rate_yaw_low_filter);
}

void attitude_get_gyro_bias(float bias[3])
{
	v3f_set(bias,ahrs_est.gyro_err_int);
}

float attitude_get_att_roll()
{
	return attitude.att[0];
}

float attitude_get_att_pitch()
{
	return attitude.att[1];
}

float attitude_get_att_yaw()
{
	return attitude.att[2];
}

float attitude_get_rate_roll()
{
	return attitude.rate[0];
}

float attitude_get_rate_pitch()
{
	return attitude.rate[1];
}

float attitude_get_rate_yaw()
{
	return attitude.rate[2];
}

float attitude_get_rate_roll_f()
{
	return attitude.rate_f[0];
}

float attitude_get_rate_pitch_f()
{
	return attitude.rate_f[1];
}

float attitude_get_rate_yaw_f()
{
	return attitude.rate_f[2];
}

bool attitude_get_valid(void)
{
	return attitude.valid;
}

void attitude_reset()
{
	ahrs_est.inited = false;
	init_time = 0.0f;
	init_count = 0;
	v3f_set_val(acc_avg,0.0f);
	v3f_set_val(mag_avg,0.0f);
	
	INFO(DEBUG_ID,"reset");
}

void attitude_reinit(float dt,ahrs_estimator *est,float acc[3],float mag[3])
{
	int i;

	if(fabs(acc[0]) > (CONSTANTS_ONE_G + 1.0f) || fabs(acc[1]) > (CONSTANTS_ONE_G + 1.0f) || fabs(acc[2]) > (CONSTANTS_ONE_G + 1.0f)){
		init_count = 0;
		init_time = 0;
		v3f_set_val(acc_avg,0.0f);
		v3f_set_val(mag_avg,0.0f);
	}
	
	init_time += dt;
	if(init_time >= 0.2f){
		for(i = 0;i < 3;i++){
			acc_avg[i] +=  acc[i];
			mag_avg[i] +=  mag[i];
		}
		init_count++;
	}

	if(init_time >= 1.0f){
		for(i = 0;i < 3;i++){
			acc_avg[i] /= (float)init_count;
			mag_avg[i] /= (float)init_count;
		}
		
		ahrs_init(est,acc_avg,mag_avg);
	}
}

void attitude_update(float dt)
{
	uint8_t mag_status = mag_get_status();
	bool armed = system_get_armed();
	float acc[3] = {0};
	float gyro[3] = {0};
	float mag[3] = {0};

	if(imu_get_update() == false || imu_get_gyro_status() != SENSOR_STATUS_OK){
		return;
	}

	imu_get_acc_filted(acc);
	ahrs_apply_acc(&ahrs_est,acc);
	
	imu_get_gyro(gyro);
	ahrs_apply_gyro(&ahrs_est,gyro);

	if(mag_status == SENSOR_STATUS_OK && mag_get_update() == true){
		mag_get_val(mag);
		ahrs_apply_mag(&ahrs_est,mag);
	}

	if(att_mag_init == false && mag_status == SENSOR_STATUS_OK){
		reinit = true;
		att_mag_init = true;
	}else if(mag_status == SENSOR_STATUS_CALIBING){
		att_mag_init = false;
		attitude.valid = false;
	}

	if(reinit == false && armed == true){
		reinit = true;
	}else if(reinit == true && armed == false){
		reinit = false;
		attitude_reset();
	}

	if(ahrs_est.inited == false){
		attitude_reinit(dt,&ahrs_est,acc,mag);
		attitude.valid = false;
	}else{

		ahrs_update(dt,&ahrs_est,armed);

		if(mag_dec_inited == false ){
			gps_get_info(&gps_info);	
			if((gps_info.fix_type == 3) && (gps_info.eph < 3.0f)){
				mag_dec = get_mag_declination((float)(gps_info.lat),(float)(gps_info.lon));
				mag_dec_inited = true;
			}
		}else{
			attitude.att[2] = wrap_180_cd_float(attitude.att[2] - mag_dec);
		}
		
		v3f_set(attitude.acc,ahrs_est.acc_raw);
		v3f_set(attitude.rate,ahrs_est.gyro);
		ahrs_qua2euler(ahrs_est.q,attitude.att);
		ahrs_euler2dcm(attitude.att,attitude.r);

		attitude.rate_f[0] = lpf2p_update(&rate_filter[0],attitude.rate[0]);
		attitude.rate_f[1] = lpf2p_update(&rate_filter[1],attitude.rate[1]);
		attitude.rate_f[2] = lpf2p_update(&rate_filter[2],attitude.rate[2]);
		
		attitude.valid = true;
	}

	DEBUG_HZ(DEBUG_ID,5,dt,"att:(%3.3f,%3.3f,%3.3f)",attitude.att[0],attitude.att[1],attitude.att[2]);
	//DEBUG_HZ(DEBUG_ID,5,dt,"dcm:(%3.3f,%3.3f,%3.3f)(%3.3f,%3.3f,%3.3f)(%3.3f,%3.3f,%3.3f)",attitude.r[0][0],attitude.r[0][1],attitude.r[0][2],attitude.r[1][0],attitude.r[1][1],attitude.r[1][2],attitude.r[2][0],attitude.r[2][1],attitude.r[2][2]);
}

void attitude_get(att_s *dat)
{
	memcpy(dat,&attitude,sizeof(att_s));
}

