#include <fcntl.h>

#include "app_debug.h"
#include "app_param.h"
#include "app_baro.h"
#include "app_rangefinder.h"
#include "app_sensors.h"

#include "lib_inav.h"
#include "lib_inav_baro.h"

#include "lib_lpf2p.h"
#include "lib_math.h"

extern void inav_inertial_filter_predict(float dt,float *vel,float *pos,float acc);
extern void inav_inertial_filter_correct(float dt, float e, float *target,float w);

#define DEBUG_ID DEBUG_ID_NAV_BARO

uint8_t baro_mode = BARO_MODE_NORMAL;

static float baro_alt_f;
static float baro_vel_f;
static float baro_vel;

bool baro_need_init = false;
bool baro_noise_enable = true;

float baro_vel_check_val;
float baro_vel_check_timeout;
float baro_vel_check_time = 0;
float baro_acc_check_time = 0;

float baro_fuse_pos_action;
float baro_fuse_vel_action;
float baro_fuse_bias_action;

float baro_fuse_pos_break;
float baro_fuse_vel_break;
float baro_fuse_bias_break;

float baro_fuse_pos_flip;
float baro_fuse_vel_flip;
float baro_fuse_bias_flip;

float baro_fuse_pos_takeoff;
float baro_fuse_vel_takeoff;
float baro_fuse_bias_takeoff;

float baro_fuse_pos_normal;
float baro_fuse_vel_normal;
float baro_fuse_bias_normal;

float baro_fuse_pos_noise;
float baro_fuse_vel_noise;
float baro_fuse_bias_noise;

float baro_fuse_pos_buf;
float baro_fuse_vel_buf;
float baro_fuse_bias_buf;

float baro_fuse_pos_step;
float baro_fuse_vel_step;
float baro_fuse_bias_step;

float baro_fuse_pos;
float baro_fuse_vel;
float baro_fuse_bias;
float baro_bias_value[3] = {0.0f};

float baro_vel_noise;
float baro_vel_smooth_time = 0;

float baro_acc_z_filter_val;
lpf2p_s baro_acc_z_filter;

void inav_baro_set_noise_enable(bool enable)
{
	baro_noise_enable = enable;
}

uint8_t inav_baro_get_fuse_mode()
{
	return baro_mode;
}

void inav_baro_get_fuse_val(float * pos,float * vel,float * bias)
{
	if(pos != NULL && vel != NULL && bias != NULL){
		*pos = baro_fuse_pos;
		*vel = baro_fuse_vel;
		*bias = baro_fuse_bias;
	}
}

void inav_baro_get_bias_value(float bias[3])
{
	v3f_set(bias,baro_bias_value);
}
void inav_baro_set_takeoff_mode()
{
	baro_mode = BARO_MODE_TAKEOFF;

	baro_fuse_pos_buf = baro_fuse_pos_takeoff; //1.2 1 5.5
	baro_fuse_vel_buf = baro_fuse_vel_takeoff;
	baro_fuse_bias_buf = baro_fuse_bias_takeoff;
}

void inav_baro_set_thrown_mode()
{
	baro_mode = BARO_MODE_FLIP;  //8 1.5 0

	//baro_fuse_pos_buf = 1.2f ; 
	//baro_fuse_vel_buf = 1.0f;
	//baro_fuse_bias_buf = 5.5f;
}

void inav_baro_set_normal_mode()
{
	baro_mode = BARO_MODE_NORMAL;
}

void inav_baro_set_flip_mode()
{
	baro_mode = BARO_MODE_FLIP;
}

void inav_baro_set_action_mode()
{
	baro_mode = BARO_MODE_ACTION;
}

void inav_baro_set_break_mode()
{
	baro_mode = BARO_MODE_BREAK;
}

void inav_baro_param_init(inav_estimator *est)
{
	param_set_var(INAV__BARO__VEL_CHECK_ACC_FILTER_NAME	,&baro_acc_z_filter_val);
	param_set_var(INAV__BARO__VEL_CHECK_VAL_NAME		,&baro_vel_check_val);
	param_set_var(INAV__BARO__VEL_CHECK_TIMEOUT_NAME	,&baro_vel_check_timeout);

	param_set_var(INAV__BARO__FUSE_POS_ACTION_NAME		,&baro_fuse_pos_action);
	param_set_var(INAV__BARO__FUSE_VEL_ACTION_NAME		,&baro_fuse_vel_action);
	param_set_var(INAV__BARO__FUSE_BIAS_ACTION_NAME		,&baro_fuse_bias_action);

	param_set_var(INAV__BARO__FUSE_POS_BREAK_NAME		,&baro_fuse_pos_break);
	param_set_var(INAV__BARO__FUSE_VEL_BREAK_NAME		,&baro_fuse_vel_break);
	param_set_var(INAV__BARO__FUSE_BIAS_BREAK_NAME		,&baro_fuse_bias_break);

	param_set_var(INAV__BARO__FUSE_POS_FLIP_NAME		,&baro_fuse_pos_flip);
	param_set_var(INAV__BARO__FUSE_VEL_FLIP_NAME		,&baro_fuse_vel_flip);
	param_set_var(INAV__BARO__FUSE_BIAS_FLIP_NAME		,&baro_fuse_bias_flip);

	param_set_var(INAV__BARO__FUSE_POS_TAKEOFF_NAME		,&baro_fuse_pos_takeoff);
	param_set_var(INAV__BARO__FUSE_VEL_TAKEOFF_NAME		,&baro_fuse_vel_takeoff);
	param_set_var(INAV__BARO__FUSE_BIAS_TAKEOFF_NAME	,&baro_fuse_bias_takeoff);

	param_set_var(INAV__BARO__FUSE_POS_NORMAL_NAME		,&baro_fuse_pos_normal);
	param_set_var(INAV__BARO__FUSE_VEL_NORMAL_NAME		,&baro_fuse_vel_normal);
	param_set_var(INAV__BARO__FUSE_BIAS_NORMAL_NAME		,&baro_fuse_bias_normal);

	param_set_var(INAV__BARO__FUSE_POS_NOISE_NAME		,&baro_fuse_pos_noise);
	param_set_var(INAV__BARO__FUSE_VEL_NOISE_NAME		,&baro_fuse_vel_noise);
	param_set_var(INAV__BARO__FUSE_BIAS_NOISE_NAME		,&baro_fuse_bias_noise);

	param_set_var(INAV__BARO__FUSE_POS_STEP_NAME		,&baro_fuse_pos_step);
	param_set_var(INAV__BARO__FUSE_VEL_STEP_NAME		,&baro_fuse_vel_step);
	param_set_var(INAV__BARO__FUSE_BIAS_STEP_NAME		,&baro_fuse_bias_step);
	
	param_set_var(INAV__BARO__VEL_NOISE_NAME			,&baro_vel_noise);
}

void inav_baro_init(inav_estimator *est)
{
	INFO(DEBUG_ID,"init acc_filter:%3.3f",baro_acc_z_filter_val);
	
	lpf2p_init(&baro_acc_z_filter,(float)MAIN_LOOP_HZ,baro_acc_z_filter_val);
}

void inav_baro_fuse_step()
{
	if(baro_fuse_pos_buf > baro_fuse_pos_normal){
		baro_fuse_pos_buf -= (baro_fuse_pos_step / MAIN_LOOP_HZ);
	}else if(baro_fuse_pos_buf < baro_fuse_pos_normal){
		baro_fuse_pos_buf += (baro_fuse_pos_step / MAIN_LOOP_HZ);
	}
	
	if(baro_fuse_vel_buf > baro_fuse_vel_normal){
		baro_fuse_vel_buf -= (baro_fuse_vel_step / MAIN_LOOP_HZ);
	}else if(baro_fuse_vel_buf < baro_fuse_vel_normal){
		baro_fuse_vel_buf += (baro_fuse_vel_step / MAIN_LOOP_HZ);
	}
	
	if(baro_fuse_bias_buf > baro_fuse_bias_normal){
		baro_fuse_bias_buf -= (baro_fuse_bias_step / MAIN_LOOP_HZ);
	}else if(baro_fuse_bias_buf < baro_fuse_bias_normal){
		baro_fuse_bias_buf += (baro_fuse_bias_step / MAIN_LOOP_HZ);
	}
}

void inav_baro_update(float dt,inav_estimator *est,bool armed)
{
	static float correct_pos = 0.0f;
	static float correct_vel = 0.0f;
	float baro_bias_ef[3] = {0.0f};
	float baro_bias_bf[3] = {0.0f};
	
	if(baro_get_status() != SENSOR_STATUS_OK){
		est->baro_valid = false;
		return;
	}else{
		est->baro_valid = true;
		
		if(baro_get_update() == true){
			baro_alt_f = baro_get_alt_f();
			baro_vel_f = baro_get_vel_f();
			baro_vel = baro_get_vel();
			correct_pos = ((-baro_alt_f) - est->pos_ned[2]);
			correct_vel = ((-baro_vel_f) - est->vel_ned[2]);
		}
		
		if(armed == true){
			if(fabs(baro_vel) > baro_vel_noise && fabs(est->vel_ned[2] - baro_vel) > baro_vel_noise && baro_noise_enable == true){
				baro_vel_smooth_time = 0.02f;
			}
			
			if(baro_vel_smooth_time >= 0){
				baro_vel_smooth_time -= dt;
				baro_fuse_pos = baro_fuse_pos_noise;
				baro_fuse_vel = baro_fuse_vel_noise;
				baro_fuse_bias = baro_fuse_bias_noise;
				
				inav_baro_fuse_step();
			}else{
				if(baro_mode == BARO_MODE_NORMAL){
					inav_baro_fuse_step();
					
					baro_fuse_pos = baro_fuse_pos_buf;
					baro_fuse_vel = baro_fuse_vel_buf;
					baro_fuse_bias = baro_fuse_bias_buf;		
				}else if(baro_mode == BARO_MODE_FLIP){
					baro_fuse_pos = baro_fuse_pos_flip;
					baro_fuse_vel = baro_fuse_vel_flip;
					baro_fuse_bias = baro_fuse_bias_flip;
				}else if(baro_mode == BARO_MODE_ACTION){
					baro_fuse_pos = baro_fuse_pos_action;
					baro_fuse_vel = baro_fuse_vel_action;
					baro_fuse_bias = baro_fuse_bias_action;
				}else if(baro_mode == BARO_MODE_BREAK){
					baro_fuse_pos = baro_fuse_pos_break;
					baro_fuse_vel = baro_fuse_vel_break;
					baro_fuse_bias = baro_fuse_bias_break;
				}else{
					baro_fuse_pos = baro_fuse_pos_takeoff;
					baro_fuse_vel = baro_fuse_vel_takeoff;
					baro_fuse_bias = baro_fuse_bias_takeoff;
				}
			}
			
			baro_need_init = true;
		}else{
			if(est->alt_valid == false && baro_need_init == true){
				est->pos_ned[2] = -baro_alt_f;
				est->vel_ned[2] = -baro_vel_f;
		
				baro_acc_check_time += dt;
				if(pythagorous_v3f(est->acc_ned) < 1.0f){
					if(baro_acc_check_time >= 0.3f){
						baro_vel_check_time = 0.0f;
						baro_acc_check_time = 0.0f;
						baro_need_init = false;
					}
				}else{
					baro_acc_check_time = 0.0f;
					//est->alt_valid = true;
				}
			}
		
			baro_fuse_pos_buf = baro_fuse_pos_normal;
			baro_fuse_vel_buf = baro_fuse_vel_normal;
			baro_fuse_bias_buf = baro_fuse_bias_normal;
		
			baro_fuse_pos = baro_fuse_pos_takeoff;
			baro_fuse_vel = baro_fuse_vel_takeoff;
			baro_fuse_bias = 0;
			
			baro_bias_ef[2] = lpf2p_update(&baro_acc_z_filter,est->acc_ned[2]);
			ef_to_bf(est->r,baro_bias_bf,baro_bias_ef);
			est->acc_bias_body[2] = baro_bias_bf[2];
		
			if(fabs(est->pos_ned[2] + baro_alt_f) > 10.0f){
				INFO(DEBUG_ID,"fuse error:%3.3f,%3.3f",-est->pos_ned[2],baro_alt_f);
				baro_need_init = true;
				est->alt_valid = false;
			}
		}
		
		if(armed == false){
			v3f_set_val(baro_bias_value,0.0);
		}
		
		baro_bias_ef[2] = correct_pos * baro_fuse_bias * 0.05f * dt;
		ef_to_bf(est->r,baro_bias_bf,baro_bias_ef);
		
		if(est->rf_valid == false){
			baro_bias_value[0] -= baro_bias_bf[0];
			baro_bias_value[1] -= baro_bias_bf[1];
			baro_bias_value[2] -= baro_bias_bf[2];
			est->acc_bias_body[0] -= baro_bias_bf[0];
			est->acc_bias_body[1] -= baro_bias_bf[1];
			est->acc_bias_body[2] -= baro_bias_bf[2];
		
			inav_inertial_filter_correct(dt,correct_vel,&est->vel_ned[2],baro_fuse_vel);
		}
			
		inav_inertial_filter_correct(dt,correct_pos,&est->pos_ned[2],baro_fuse_pos);
		
		if(baro_need_init == false && est->alt_valid == false){
			if(fabs(est->vel_ned[2]) < baro_vel_check_val){
				baro_vel_check_time += dt;
				if(baro_vel_check_time >= baro_vel_check_timeout){
					INFO(DEBUG_ID,"bias ready %3.3f",est->acc_bias_body[2]);
					est->alt_valid = true;
				}
			}else{
				baro_vel_check_time = 0.0f;
			}
		}
		
		DEBUG_HZ(DEBUG_ID,5,dt,"baro:(%+3.3f) pv:(%+3.3f,%+3.3f) bias:%3.3f acc:%3.3f",baro_alt_f,-est->pos_ned[2],-est->vel_ned[2],est->acc_bias_body[2],est->acc_ned[2]); 
	}

}

