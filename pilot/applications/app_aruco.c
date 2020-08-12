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

float aruco_vel_raw[2] = {0.0f,0.0f};
float aruco_quality = 0.0f;
uint16_t aruco_version = 0;


void aruco_update(float dt)
{
		
hal_get_aruco(dt,aruco_vel_raw,&aruco_quality,&aruco_version);
				
}



