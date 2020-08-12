#ifndef _APP_CONTROL_CIRCLE_H_
#define _APP_CONTROL_CIRCLE_H_

#include <stdbool.h>
#include "app_rc.h"

enum circle_status{
	CIRCLE_START = 0,
	CIRCLE_ON,
	CIRCLE_END,
	CIRCLE_STOP,
	CIRCLE_OFF		
};

typedef struct{
	float euler_angle_rate[3];
	float circle_yaw_angle_rate;
	float circle_time;
	float circle_radius;
	float circle_vel_d[2];
	float circle_vel_c[2];
	int circle_direction;
	int circle_on_flag;
	float pos_grd_c[2];
	float pos_grd_d[2];
	float grd_heading;
	float init_yaw;
	float init_pos_ned[2];
	float vel_axis[2];
	float pos_axis[2];
	float body_z_rate_d;
	float body_z_rate_c;
	enum circle_status status;
}circle_controller;	


bool control_circle_check();
void control_circle_exit();
void control_circle_init(float param1,float param2);
void control_circle_param_init();
void control_circle_update(float dt,rc_s * rc);

void control_circle_generate_spec_points();

void calc_angle_rate(float dt,circle_controller* circle);
int control_circle_get_status();

float control_circle_vel_get_pos_sim_x();
float control_circle_vel_get_pos_sim_y();
float control_circle_vel_get_pos_cur_x();
float control_circle_vel_get_pos_cur_y();
float control_circle_vel_get_vel_x_d();
float control_circle_vel_get_vel_x_c();
float control_circle_vel_get_vel_y_d();
float control_circle_vel_get_vel_y_c();
float control_circle_vel_get_body_rate_yaw_d();
float control_circle_vel_get_body_rate_yaw_c();

float control_circle_vel_get_grd_heading();
float control_circle_vel_get_vel_axis_x();
float control_circle_vel_get_vel_axis_y();
float control_circle_vel_get_pos_axis_x();
float control_circle_vel_get_pos_axis_y();
float control_circle_vel_get_pos_flow_x_inter();
float control_circle_vel_get_pos_flow_y_inter();



#endif

