#ifndef _APP_CONTROL_COMMON_H_
#define _APP_CONTROL_COMMON_H_

#include "app_rc.h"

void control_att_common_rc(float dt,rc_s * rc);
void control_yaw_common_rc(float dt,rc_s * rc);
void control_alt_common_rc(float dt,rc_s * rc);

void control_yaw_common(float dt,float yaw_target,float rate_limit);
void control_att_common(float dt,float roll_target,float pitch_target,float rate_limit);
void control_yaw_rate_common(float dt,float yaw_rate_target,float rate_limit);
void control_att_rate_common(float dt,float roll_target,float pitch_target,float rate_limit);
void control_att_common_limit(float dt,float roll_target,float pitch_target,float att_limit,float rate_limit);

void control_alt_vel_common(float dt,float vel_z_target,float vel_limit);
void control_alt_pos_common(float dt,float pos_z_target,float vel_limit);
void control_set_alt_start(float alt);

void control_pos_ned_xy_common(float dt, float pos_ned_target[3],float vel_limit,float rate_limit);
void control_vel_ned_xy_common(float dt, float vel_ned_target[3],float vel_limit,float rate_limit);
void control_vel_grd_xy_common(float dt, float vel_grd_target[2],float vel_limit,float rate_limit);

void control_common_init();
void control_common_param_init();

float control_get_rf_alt_target();
void control_set_rf_alt_target(float alt);


#endif

