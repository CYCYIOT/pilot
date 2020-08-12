#ifndef _APP_POS_CONTROL_H_
#define _APP_POS_CONTROL_H_

#include "lib_pid.h"

void pos_control_init();
void pos_control_reinit();
void pos_control_xy_reinit();
void pos_control_param_init();

pid_s * pos_control_get_pid(int8_t no);

float pos_control_get_pos_ned_x_target();
float pos_control_get_pos_ned_y_target();
float pos_control_get_pos_ned_z_target();

float pos_control_get_vel_ned_x_target();
float pos_control_get_vel_ned_y_target();
float pos_control_get_vel_ned_z_target();
float pos_control_get_vel_grd_x_target();
float pos_control_get_vel_grd_y_target();

float pos_control_get_acc_z_target();

void pos_control_set_pos_ned_x_target(float pos);
void pos_control_set_pos_ned_y_target(float pos);
void pos_control_set_pos_ned_z_target(float pos);

void pos_control_set_vel_ned_x_target(float vel);
void pos_control_set_vel_ned_y_target(float vel);
void pos_control_set_vel_ned_z_target(float vel);
void pos_control_set_vel_grd_x_target(float vel);
void pos_control_set_vel_grd_y_target(float vel);

void pos_control_set_z_acc_target(float acc);

float pos_control_pos_ned_x_update(float dt);
float pos_control_pos_ned_y_update(float dt);
float pos_control_pos_ned_z_update(float dt);
void pos_control_set_vel_grd_x_target(float vel);
void pos_control_set_vel_grd_y_target(float vel);

float pos_control_vel_grd_x_update(float dt);
float pos_control_vel_grd_y_update(float dt);
float pos_control_vel_ned_z_update(float dt);

float pos_control_acc_z_update(float dt);

void pos_control_set_vel_pid_thrown_mode();
void pos_control_set_vel_pid_takeoff_mode();
void pos_control_set_vel_pid_normal_mode();
void pos_control_set_vel_pid_rotate_mode();
void pos_control_set_vel_pid_flip_mode();
void pos_control_set_vel_pid_break_high_speed_mode();
void pos_control_set_vel_pid_break_low_speed_mode();
void pos_control_set_vel_pid_circle_mode();


void pos_control_set_alt_pid_land_mode();
void pos_control_set_alt_pid_normal_mode();
void pos_control_set_alt_pid_flip_mode();

void vel_control_xy_i_stash();
void vel_control_xy_i_pop(int i,int mode);

#endif

