#ifndef _LIB_INAV_BARO_H_
#define _LIB_INAV_BARO_H_

#include "lib_inav.h"

#define BARO_MODE_NORMAL	0
#define BARO_MODE_TAKEOFF	1
#define BARO_MODE_FLIP		2
#define BARO_MODE_ACTION	3
#define BARO_MODE_BREAK		4
#define BARO_MODE_THROWN	5

void inav_baro_param_init(inav_estimator *est);
void inav_baro_init(inav_estimator *est);
void inav_baro_update(float dt,inav_estimator *est,bool armed);

void inav_baro_get_fuse_val(float * pos,float * vel,float * bias);
uint8_t inav_baro_get_fuse_mode();

void inav_baro_set_takeoff_mode();
void inav_baro_set_thrown_mode();
void inav_baro_set_normal_mode();
void inav_baro_set_flip_mode();
void inav_baro_set_action_mode();
void inav_baro_set_break_mode();
void inav_baro_set_noise_enable(bool enable);
void inav_baro_get_bias_value(float bias[3]);

#endif

