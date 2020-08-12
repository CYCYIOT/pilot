#ifndef _LIB_INAV_GPS_H_
#define _LIB_INAV_GPS_H_

#include "lib_inav.h"

void inav_gps_param_init(inav_estimator *est);
void inav_gps_init(inav_estimator *est);
void inav_gps_update(float dt,inav_estimator *est,bool armed);

void inav_gps_get_fuse_val(float * pos,float * vel,float * bias);
void inav_gps_get_bias_value(float bias[3]);


#endif

