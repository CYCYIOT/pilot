#include "app_attitude.h"
#include "app_debug.h"

#include "lib_math.h"
#include "lib_inav.h"
#include "lib_inav_baro.h"
#include "lib_inav_flow.h"
#include "lib_inav_gps.h"
#include "lib_inav_rangefinder.h"

#define DEBUG_ID DEBUG_ID_NAV

void inav_param_init(inav_estimator *est)
{
	inav_baro_param_init(est);
	inav_flow_param_init(est);
	inav_gps_param_init(est);
	inav_rf_param_init(est);
}

void inav_reset(inav_estimator *est)
{
	est->reinit = true;
}


void inav_init(inav_estimator *est)
{
	inav_baro_init(est);
	inav_flow_init(est);
	inav_gps_init(est);
	inav_rf_init(est);

	v3f_set_val(est->pos_ned,0.0f);
	v3f_set_val(est->vel_ned,0.0f);
	v3f_set_val(est->acc_ned,0.0f);
	v3f_set_val(est->acc_body,0.0f);
	v3f_set_val(est->acc_bias_body,0.0f);
	v3f_set_val(est->gps_pos_corr,0.0f);
	v3f_set_val(est->gps_vel_corr,0.0f);
		
	memset(&(est->r),0x00, 9 * sizeof(float));
	memset(&(est->gps),0x00,sizeof(gps_info_s));
	memset(&(est->ref),0x00,sizeof(map_projection_reference_s));

	est->reinit		= false;
	est->flow_valid    = false;
	est->baro_valid    = false;
	est->gps_valid    = false;
	est->gps_updated  = false;
	est->pos_valid = false;
	est->vel_valid = false;
	est->alt_valid  = false;
	est->gps_check_timer = 0.0f;
	est->eph      = GPS_EPH_EPV_MAX;
	est->epv      = 2.0f;
}

void inav_apply_gps(inav_estimator *est,gps_info_s *gps)
{
	memcpy(&(est->gps),gps, sizeof(gps_info_s));
	est->gps_updated = true;
}

void inav_inertial_filter_predict(float dt,float *vel,float *pos,float acc)
{
	if (isfinite(dt)) {
		if (!isfinite(acc)) {
			acc = 0.0f;
		}

		*pos += *vel * dt + acc * dt * dt / 2.0f;
		*vel += acc * dt;
	}
}
	
void inav_inertial_filter_correct(float dt, float e, float *target,float w)
{
	if (isfinite(e) && isfinite(w) && isfinite(dt)) {
		float ewdt = e * w * dt;
		*target += ewdt;
	}
}
void inav_update(float dt,inav_estimator *est,bool armed)
{
	att_s att;
	int i;

	if(armed == false && est->reinit == true){
		INFO(DEBUG_ID,"reinit");
		v3f_set_val(est->acc_bias_body,0.0f);
		v3f_set_val(est->vel_ned,0.0f);
		est->pos_valid = false;
		est->vel_valid = false;
		est->alt_valid = false;
		est->reinit = false;
	}else if(armed == true && est->reinit == false){
		est->reinit = true;
	}

	attitude_get(&att);
	if(att.valid == false){
		return;
	}

	memcpy(est->r,att.r,9 * sizeof(float));
	v3f_set(est->acc_body,att.acc);
	for (i = 0; i < 3; i++){
		est->acc_body[i] -= est->acc_bias_body[i];
	}
	bf_to_ef(est->r,est->acc_body,est->acc_ned);
	est->acc_ned[2] += CONSTANTS_ONE_G;
	
	inav_inertial_filter_predict(dt, &est->vel_ned[0],&est->pos_ned[0], est->acc_ned[0]);
	inav_inertial_filter_predict(dt, &est->vel_ned[1],&est->pos_ned[1], est->acc_ned[1]);
	inav_inertial_filter_predict(dt, &est->vel_ned[2],&est->pos_ned[2], est->acc_ned[2]);

	inav_baro_update(dt,est,armed);
	inav_gps_update(dt,est,armed);
	inav_flow_update(dt,est,armed);
	inav_rf_update(dt,est,armed);

	if(est->flow_valid || est->gps_valid){
		est->pos_valid = true;
		est->vel_valid = true;
	}else{
		est->pos_valid = false;
		est->vel_valid = false;
	}
}

