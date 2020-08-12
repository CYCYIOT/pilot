#ifndef _LIB_INAV_RF_H_
#define _LIB_INAV_RF_H_

#include "lib_inav.h"

#define RF_MODE_NORMAL	0
#define RF_MODE_FLIP	1

void inav_rf_param_init(inav_estimator *est);
void inav_rf_init(inav_estimator *est);
void inav_rf_update(float dt,inav_estimator *est,bool armed);

void inav_rf_get_bias_value(float bias[3]);
void inav_rf_get_fuse_val(float * vel,float * bias);
uint8_t inav_rf_get_fuse_mode();

void inav_rf_set_normal_mode();
void inav_rf_set_flip_mode();

#endif

