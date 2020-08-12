#ifndef _INAV_LIB_H_
#define _INAV_LIB_H_
#include "ubx.h"
#include "app_param.h"
#include "geo.h"

#define INAV_EST_BUF_SIZE      200

#define GPS_EPH_EPV_MAX        20.0f
#define GPS_EPH_EPV_MIN        2.5f

#define INAV_GPS_TIMEOUT		1.0f


typedef struct{
	float pos_ned[3];
	float vel_ned[3];

	bool pos_valid;
	bool vel_valid;
	bool alt_valid;

	bool reinit;

	bool gps_valid;
	bool flow_valid;
	bool baro_valid;
	bool rf_valid;

	float acc_ned[3];
	float acc_body[3];
	float acc_bias_body[3];	
	float r[3][3];

	map_projection_reference_s ref;
	float ref_alt;
	float eph;
	float epv;
	
	gps_info_s gps;
	float gps_pos_corr[3];
	float gps_vel_corr[3];
	bool  gps_updated;
	float w_gps_xy;
	float gps_check_timer;
	float gps_dcm[3][3] ;
}inav_estimator;

void inav_param_init(inav_estimator *est);
void inav_init(inav_estimator *est);
void inav_reset(inav_estimator *est);
void inav_apply_gps(inav_estimator *est,gps_info_s *gps);
void inav_update(float dt,inav_estimator *est,bool armed);

#endif

