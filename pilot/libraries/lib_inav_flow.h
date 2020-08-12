#ifndef _LIB_INAV_FLOW_H_
#define _LIB_INAV_FLOW_H_

#include "lib_inav.h"

#define FLOW_MODE_NORMAL	0
#define FLOW_MODE_TAKEOFF	1
#define FLOW_MODE_FLIP1		2
#define FLOW_MODE_FLIP2		3
#define FLOW_MODE_ROTATE    4
#define FLOW_MODE_ACTION    5
#define FLOW_MODE_THROWN	6


void inav_flow_param_init(inav_estimator *est);
void inav_flow_init(inav_estimator *est);
void inav_flow_update(float dt,inav_estimator *est,bool armed);

void inav_flow_get_fuse_val(float * pos,float * vel,float * bias);
uint8_t inav_flow_get_fuse_mode();
float inav_flow_get_fq_weight();
void inav_flow_set_quality_fuse_enable(bool enable);
void inav_flow_get_bias_value(float bias[3]);

void inav_flow_get_vel_ned(float vel_ned[2]);
void inav_flow_get_pos_ned(float pos_ned[2]);

void inav_flow_set_thrown_mode();
void inav_flow_set_takeoff_mode();
void inav_flow_set_normal_mode();
void inav_flow_set_rotate_mode();
void inav_flow_set_flip1_mode();
void inav_flow_set_flip2_mode();
void inav_flow_set_action_mode();

#endif

