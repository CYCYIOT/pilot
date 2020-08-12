#ifndef _APP_NAV_H_
#define _APP_NAV_H_

#include <stdbool.h>
#include "lib_inav.h"

typedef struct{
	//ned frame
	float pos_ned[3];
	float vel_ned[3];
	float acc_ned[3];

	//ground frame
	float vel_grd[3];

	//body_frame
	float acc_bias_body[3];
	float acc_body[3];
	float acc_body_sum;
	
	bool pos_valid;
	bool vel_valid;
	bool alt_valid;
}nav_s;

void nav_param_init();
void nav_init();
void nav_reset();
void nav_update(float dt);
void nav_get_pos(nav_s *dat);
void nav_get_arm_pos(nav_s *dat);

float nav_get_dis_to_grd(void);

float nav_get_acc_bias_body_x();
float nav_get_acc_bias_body_y();
float nav_get_acc_bias_body_z();

float nav_get_acc_ned_x();
float nav_get_acc_ned_y();
float nav_get_acc_ned_z();
float nav_get_pos_ned_x();
float nav_get_pos_ned_y();
float nav_get_pos_ned_z();
float nav_get_vel_ned_x();
float nav_get_vel_ned_y();
float nav_get_vel_ned_z();
float nav_get_vel_grd_x();
float nav_get_vel_grd_y();
float nav_get_vel_grd_z();

float nav_get_acc_body_sum();

bool nav_get_pos_valid();
bool nav_get_vel_valid();
bool nav_get_alt_valid();

void nav_map_gps(double lat,double lon,float * x,float * y);
void nav_get_estimator_data(inav_estimator *est);
void nav_recover_acc_bias();
void nav_backup_acc_bias();
void nav_get_att_offset(float att_offset[2]);
void nav_att_offset_update();

#endif

