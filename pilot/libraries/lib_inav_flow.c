#include <fcntl.h>

#include "app_debug.h"
#include "app_param.h"
#include "app_flow.h"
#include "app_sensors.h"

#include "lib_inav.h"
#include "lib_inav_flow.h"

#include "lib_math.h"

extern void inav_inertial_filter_predict(float dt,float *vel,float *pos,float acc);
extern void inav_inertial_filter_correct(float dt, float e, float *target,float w);

#define DEBUG_ID DEBUG_ID_NAV_FLOW

uint8_t flow_mode = FLOW_MODE_NORMAL;

bool flow_flip_step_mode = false;

float flow_fuse_pos_takeoff;
float flow_fuse_vel_takeoff;
float flow_fuse_bias_takeoff;

float flow_fuse_pos_normal;
float flow_fuse_vel_normal;
float flow_fuse_bias_normal;

float flow_fuse_pos;
float flow_fuse_vel;
float flow_fuse_bias;

float flow_fuse_pos_tmp;
float flow_fuse_vel_tmp;
float flow_fuse_bias_tmp;

float flow_fuse_pos_flip1;
float flow_fuse_vel_flip1;
float flow_fuse_bias_flip1;

float flow_fuse_pos_flip2;
float flow_fuse_vel_flip2;
float flow_fuse_bias_flip2;

float flow_fuse_pos_flip_step;
float flow_fuse_vel_flip_step;
float flow_fuse_bias_flip_step;

float flow_fuse_pos_rotate;
float flow_fuse_vel_rotate;
float flow_fuse_bias_rotate;

float flow_fuse_pos_action;
float flow_fuse_vel_action;
float flow_fuse_bias_action;

float flow_bias_value[3] = {0};

float flow_pos_ned[3] = {0};
float flow_pos_ned_corr[3] = {0};
float flow_vel_ned[3] = {0};
float flow_vel_ned_corr[3] = {0};

bool flow_quality_enable = false;
float fq_weight = 1.0f;
float fq_weight_min = 0.0f;
float fq_weight_scale = 1.0f;

void inav_flow_get_bias_value(float bias[3])
{
	v3f_set(bias,flow_bias_value);
}

void inav_flow_get_vel_ned_corr(float corr[3])
{
	v3f_set(corr,flow_vel_ned_corr);
}

void inav_flow_get_pos_ned_corr(float corr[3])
{
	v3f_set(corr,flow_pos_ned_corr);
}

void inav_flow_get_vel_ned(float vel_ned[2])
{
	vel_ned[0] = flow_vel_ned[0];
	vel_ned[1] = flow_vel_ned[1];
}

void inav_flow_get_pos_ned(float pos_ned[2])
{
	pos_ned[0] = flow_pos_ned[0];
	pos_ned[1] = flow_pos_ned[1];
}

void inav_flow_get_fuse_val(float * pos,float * vel,float * bias)
{
	if(pos != NULL && vel != NULL && bias != NULL){
		*pos = flow_fuse_pos;
		*vel = flow_fuse_vel;
		*bias = flow_fuse_bias;
	}
}

float inav_flow_get_fq_weight()
{
	return fq_weight;
}

uint8_t inav_flow_get_fuse_mode()
{
	return flow_mode;
}

void inav_flow_set_flip1_mode()
{
	flow_flip_step_mode = false;
	flow_mode = FLOW_MODE_FLIP1;
	
	flow_fuse_pos_tmp = flow_fuse_pos_flip1;   // 3  10  0
	flow_fuse_vel_tmp = flow_fuse_vel_flip1;
	flow_fuse_bias_tmp = flow_fuse_bias_flip1;
}

void inav_flow_set_flip2_mode()
{
	flow_flip_step_mode = true;
	flow_mode = FLOW_MODE_FLIP2;
	
	flow_fuse_pos_tmp = flow_fuse_pos_flip2;   // 1.5 6 13
	flow_fuse_vel_tmp = flow_fuse_vel_flip2;
	flow_fuse_bias_tmp = flow_fuse_bias_flip2;
}

void inav_flow_set_takeoff_mode()
{
	flow_mode = FLOW_MODE_TAKEOFF;
	flow_fuse_pos_tmp = flow_fuse_pos_takeoff;   // 1 3 3
	flow_fuse_vel_tmp = flow_fuse_vel_takeoff;
	flow_fuse_bias_tmp = flow_fuse_bias_takeoff;
}

void inav_flow_set_thrown_mode()
{
	flow_mode = FLOW_MODE_THROWN;
	flow_fuse_pos_tmp = 0;
	flow_fuse_vel_tmp = 10;
	flow_fuse_bias_tmp = 0;
}

void inav_flow_set_rotate_mode()
{
	flow_mode = FLOW_MODE_ROTATE;
	flow_fuse_pos_tmp = flow_fuse_pos_rotate;
	flow_fuse_vel_tmp = flow_fuse_vel_rotate;
	flow_fuse_bias_tmp = flow_fuse_bias_rotate;
}

void inav_flow_set_action_mode()
{
	flow_mode = FLOW_MODE_ACTION;
	flow_fuse_pos_tmp = flow_fuse_pos_action;
	flow_fuse_vel_tmp = flow_fuse_vel_action;
	flow_fuse_bias_tmp = flow_fuse_bias_action;
}

void inav_flow_set_normal_mode()
{
	flow_mode = FLOW_MODE_NORMAL;
	flow_fuse_pos_tmp = flow_fuse_pos_normal; // 1 2.5 0.2
	flow_fuse_vel_tmp = flow_fuse_vel_normal;
	flow_fuse_bias_tmp = flow_fuse_bias_normal;
	
}

void inav_flow_set_quality_fuse_enable(bool enable)
{
	flow_quality_enable = enable;
}

void inav_flow_set_quality_fuse()
{
	if(flow_quality_enable){
		fq_weight = flow_get_quality_f() * fq_weight_scale;
		fq_weight = constrain_float(fq_weight,fq_weight_min,1.0f);
	}else{
		fq_weight = 1.0f;
	}

	flow_fuse_pos = fq_weight * flow_fuse_pos_tmp;
	flow_fuse_vel = fq_weight * flow_fuse_vel_tmp;
	flow_fuse_bias = fq_weight * flow_fuse_bias_tmp;
}

void inav_flow_param_init(inav_estimator *est)
{
	param_set_var(INAV__FLOW__FUSE_POS_FLIP1_NAME		,&flow_fuse_pos_flip1);
	param_set_var(INAV__FLOW__FUSE_VEL_FLIP1_NAME		,&flow_fuse_vel_flip1);
	param_set_var(INAV__FLOW__FUSE_BIAS_FLIP1_NAME		,&flow_fuse_bias_flip1);

	param_set_var(INAV__FLOW__FUSE_POS_FLIP2_NAME		,&flow_fuse_pos_flip2);
	param_set_var(INAV__FLOW__FUSE_VEL_FLIP2_NAME		,&flow_fuse_vel_flip2);
	param_set_var(INAV__FLOW__FUSE_BIAS_FLIP2_NAME		,&flow_fuse_bias_flip2);

	param_set_var(INAV__FLOW__FUSE_POS_FLIP_STEP_NAME	,&flow_fuse_pos_flip_step);
	param_set_var(INAV__FLOW__FUSE_VEL_FLIP_STEP_NAME	,&flow_fuse_vel_flip_step);
	param_set_var(INAV__FLOW__FUSE_BIAS_FLIP_STEP_NAME	,&flow_fuse_bias_flip_step);

	param_set_var(INAV__FLOW__FUSE_POS_TAKEOFF_NAME		,&flow_fuse_pos_takeoff);
	param_set_var(INAV__FLOW__FUSE_VEL_TAKEOFF_NAME		,&flow_fuse_vel_takeoff);
	param_set_var(INAV__FLOW__FUSE_BIAS_TAKEOFF_NAME	,&flow_fuse_bias_takeoff);

	param_set_var(INAV__FLOW__FUSE_POS_NORMAL_NAME		,&flow_fuse_pos_normal);
	param_set_var(INAV__FLOW__FUSE_VEL_NORMAL_NAME		,&flow_fuse_vel_normal);
	param_set_var(INAV__FLOW__FUSE_BIAS_NORMAL_NAME		,&flow_fuse_bias_normal);

	param_set_var(INAV__FLOW__FUSE_POS_ROTATE_NAME		,&flow_fuse_pos_rotate);
	param_set_var(INAV__FLOW__FUSE_VEL_ROTATE_NAME		,&flow_fuse_vel_rotate);
	param_set_var(INAV__FLOW__FUSE_BIAS_ROTATE_NAME		,&flow_fuse_bias_rotate);	

	param_set_var(INAV__FLOW__FUSE_POS_ACTION_NAME		,&flow_fuse_pos_action);
	param_set_var(INAV__FLOW__FUSE_VEL_ACTION_NAME		,&flow_fuse_vel_action);
	param_set_var(INAV__FLOW__FUSE_BIAS_ACTION_NAME		,&flow_fuse_bias_action);	

	param_set_var(INAV__FLOW__QUALITY_WEIGHT_MIN_NAME	,&fq_weight_min);
	param_set_var(INAV__FLOW__QUALITY_WEIGHT_SCALE_NAME	,&fq_weight_scale);	
	fq_weight_scale = constrain_float(fq_weight_scale,1.0f,5.0f);
	
}

void inav_flow_init(inav_estimator *est)
{
	INFO(DEBUG_ID,"init");
}

void inav_flow_update(float dt,inav_estimator *est,bool armed)
{
	static float flow_dt = 0;
	float flow_vel[2];
	float flow_vel_body[3] = {0};
	float flow_bias_ef[3] = {0};
	float flow_bias_bf[3] = {0};

	if(flow_get_status() != SENSOR_STATUS_OK){
		est->flow_valid = false;
		flow_dt = 0;
	}else{
		est->flow_valid = true;
		
		flow_dt += dt;
		if(flow_get_update() == true){
			flow_get_vel(flow_vel);
			flow_vel_body[0] = flow_vel[0];
			flow_vel_body[1] = flow_vel[1];
		
			bf_to_ef(est->r,flow_vel_body,flow_vel_ned);
		
			if(armed == true){
				flow_pos_ned[0] += flow_vel_ned[0] * flow_dt;
				flow_pos_ned[1] += flow_vel_ned[1] * flow_dt;
			}else{
				flow_pos_ned[0] = est->pos_ned[0];
				flow_pos_ned[1] = est->pos_ned[1];
			}
		
			flow_pos_ned_corr[0] = flow_pos_ned[0] - est->pos_ned[0];
			flow_pos_ned_corr[1] = flow_pos_ned[1] - est->pos_ned[1];
		
			flow_vel_ned_corr[0] = flow_vel_ned[0] - est->vel_ned[0];
			flow_vel_ned_corr[1] = flow_vel_ned[1] - est->vel_ned[1];			
		
			flow_dt = 0;
		}
		
		if(armed == false){
			inav_flow_set_takeoff_mode();
		}
		
		inav_flow_set_quality_fuse();
		
		if(flow_flip_step_mode == true){
			if(flow_fuse_pos > flow_fuse_pos_normal){
				flow_fuse_pos -= (flow_fuse_pos_flip_step / MAIN_LOOP_HZ);
			}else if(flow_fuse_pos < flow_fuse_pos_normal){
				flow_fuse_pos += (flow_fuse_pos_flip_step / MAIN_LOOP_HZ);
			}
			
			if(flow_fuse_vel > flow_fuse_vel_normal){
				flow_fuse_vel -= (flow_fuse_vel_flip_step / MAIN_LOOP_HZ);
			}else if(flow_fuse_vel < flow_fuse_vel_normal){
				flow_fuse_vel += (flow_fuse_vel_flip_step / MAIN_LOOP_HZ);
			}
			
			if(flow_fuse_bias > flow_fuse_bias_normal){
				flow_fuse_bias -= (flow_fuse_bias_flip_step / MAIN_LOOP_HZ);
			}else if(flow_fuse_bias < flow_fuse_bias_normal){
				flow_fuse_bias += (flow_fuse_bias_flip_step / MAIN_LOOP_HZ);
			}
		}
		
		if(armed == false){
			v3f_set_val(flow_bias_value,0.0);
		}
		
		if(armed == true){
			flow_bias_ef[0] = flow_vel_ned_corr[0] * flow_fuse_bias * 0.05f * dt;
			flow_bias_ef[1] = flow_vel_ned_corr[1] * flow_fuse_bias * 0.05f * dt;
			
			ef_to_bf(est->r,flow_bias_bf,flow_bias_ef);
			flow_bias_value[0] -= flow_bias_bf[0];
			flow_bias_value[1] -= flow_bias_bf[1];
			flow_bias_value[2] -= flow_bias_bf[2];
			
			est->acc_bias_body[0] -= flow_bias_bf[0];
			est->acc_bias_body[1] -= flow_bias_bf[1];
			est->acc_bias_body[2] -= flow_bias_bf[2];
		}
		
		inav_inertial_filter_correct(dt,flow_pos_ned_corr[0],&est->pos_ned[0],flow_fuse_pos);
		inav_inertial_filter_correct(dt,flow_pos_ned_corr[1],&est->pos_ned[1],flow_fuse_pos);
		
		inav_inertial_filter_correct(dt,flow_vel_ned_corr[0],&est->vel_ned[0],flow_fuse_vel);
		inav_inertial_filter_correct(dt,flow_vel_ned_corr[1],&est->vel_ned[1],flow_fuse_vel);
		
		//DEBUG_HZ(DEBUG_ID,20,dt,"v(%3.3f,%3.3f)(%3.3f,%3.3f)->(%3.3f,%3.3f) (%3.3f,%3.3f)(%3.3f,%3.3f)",est->vel_ned[0],est->vel_ned[1],flow_vel_ned[0],flow_vel_ned[1],flow_vel_ned_corr[0],flow_vel_ned_corr[1],est->acc_body[0],est->acc_body[1],est->acc_ned[0],est->acc_ned[1]);
	}

}

