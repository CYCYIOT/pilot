#include <string.h>

#include "app_debug.h"
#include "app_sensors.h"

#include "param.h"
#include "hal.h"
#include "hal_ubx.h"

#define DEBUG_ID DEBUG_ID_GPS

static gps_info_s gps_info;

float gps_error_timeout = 2.0f;
float gps_error_time = 0.0f;
float gps_get_error_time()
{
	return gps_error_time;
}

uint8_t gps_get_status()
{
	if(gps_existed_check() == false){
		return SENSOR_STATUS_FAIL;
	}else{
		return SENSOR_STATUS_OK;
	}
}

bool gps_get_info(gps_info_s * info)
{
	memcpy(info,&gps_info,sizeof(gps_info));
	if(gps_info.valid == true){		
		return true;
	}else{
		return false;
	}
}

void gps_init(void)
{
	INFO(DEBUG_ID,"init");
	gps_info.valid = false;
}

void gps_update(float dt)
{
	if(hal_get_gps(dt,&gps_info) == true){
		gps_info.valid = true;
		DEBUG(DEBUG_ID,"fix:%d num:%d eph:%3.3f %3.3f(%3.3f,%3.3f)(%lf,%lf)",gps_info.fix_type,gps_info.satellites_used,gps_info.eph,gps_info.vel_m_s,gps_info.vel_ned[0],gps_info.vel_ned[1],gps_info.lat,gps_info.lon);
		gps_error_time = 0.0f;
	}else{	
		gps_info.valid = false;
		gps_error_time += dt;
	}
}

