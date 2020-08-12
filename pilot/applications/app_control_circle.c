#include "app_debug.h"
#include "app_nav.h"
#include "app_attitude.h"
#include "app_flow.h"
#include "app_control.h"
#include "app_control_poshold.h"
#include "app_control_common.h"
#include "app_control_circle.h"
#include "app_pos_control.h"
#include "app_att_control.h"

#include "lib_math.h"

#define DEBUG_ID DEBUG_ID_CONTROL

circle_controller circle;

#define CIRCLE_TIME       30.0f
#define CIRCLE_RADIUS     1.0f
#define PI                3.1415926f
#define PI_2              2 * PI

#define Y_N               0
#define Y_P               1
#define X_P               2
#define X_N               3

static att_s att;
static int init_att_set_flag = 0;
static float att_pre[3];

int control_circle_get_status()
{
	return circle.status;
}



/************************control_circle_vel*************************/
float control_circle_vel_get_pos_sim_x()
{
	return circle.pos_grd_d[0];
}
float control_circle_vel_get_pos_sim_y()
{
	return circle.pos_grd_d[1];
}
float control_circle_vel_get_pos_cur_x()
{
	return circle.pos_grd_c[0];
}
float control_circle_vel_get_pos_cur_y()
{
	return circle.pos_grd_c[1];
}

float control_circle_vel_get_body_rate_yaw_d()
{
	return circle.body_z_rate_d;
}

float control_circle_vel_get_body_rate_yaw_c()
{
	return circle.body_z_rate_c;
}

float control_circle_vel_get_vel_x_d()
{
	return circle.circle_vel_d[0];
}

float control_circle_vel_get_vel_x_c()
{
	return circle.circle_vel_c[0];
}

float control_circle_vel_get_vel_y_d()
{
 	return circle.circle_vel_d[1];
}

float control_circle_vel_get_vel_y_c()
{
	return circle.circle_vel_c[1];
}


void control_circle_param_init()
{
}


void calc_pos_grd_c(circle_controller *circle)
{
	float yaw_rad,pos_ned[2];
	yaw_rad = radians(circle->init_yaw);
	pos_ned[0] = nav_get_pos_ned_x() - circle->init_pos_ned[0];
	pos_ned[1] = nav_get_pos_ned_y() - circle->init_pos_ned[1];
	circle->pos_grd_c[0] = pos_ned[0] * cosf(yaw_rad) + pos_ned[1] * sinf(yaw_rad);
	circle->pos_grd_c[1] = pos_ned[1] * cosf(yaw_rad) - pos_ned[0] * sinf(yaw_rad);
}

bool control_circle_check()
{
	uint8_t check = CONTROL_CHECK_ATT | CONTROL_CHECK_ALT | CONTROL_CHECK_VEL | CONTROL_CHECK_ARM;

	return control_check(check);
}

bool control_circle_vel_mode_init(circle_controller * circle)
{
	attitude_get(&att);
	float angle_rate = 2 * M_PI / CIRCLE_TIME;                          //角速度
	if(circle->circle_direction == Y_N){
		circle->circle_yaw_angle_rate = angle_rate;
		circle->circle_vel_d[0] = 0.0f;
		circle->circle_vel_d[1] = -angle_rate * CIRCLE_RADIUS;          //线速度
	}else if(circle->circle_direction == Y_P){
		circle->circle_yaw_angle_rate = -angle_rate;
		circle->circle_vel_d[0] = 0.0f;
		circle->circle_vel_d[1] = angle_rate * CIRCLE_RADIUS;
	}else if(circle->circle_direction == X_P){
		circle->circle_yaw_angle_rate = angle_rate;
		circle->circle_vel_d[0] = circle->circle_yaw_angle_rate * CIRCLE_RADIUS;
		circle->circle_vel_d[1] = 0.0f;
	}else{
		circle->circle_yaw_angle_rate = -angle_rate;
		circle->circle_vel_d[0] = angle_rate * CIRCLE_RADIUS;
		circle->circle_vel_d[1] = 0.0f;
	}

	circle->init_yaw = att.att[2];
	circle->status= CIRCLE_ON;
	circle->circle_on_flag = 0;
	circle->init_pos_ned[0] = nav_get_pos_ned_x();
	circle->init_pos_ned[1] = nav_get_pos_ned_y();

	return true;
}

void control_circle_exit()
{
	nav_recover_acc_bias();

	INFO(DEBUG_ID,"circle exit");
}

void control_circle_init(float param1,float param2)
{
	circle.circle_direction = Y_N;
	circle.circle_time = CIRCLE_TIME;
	circle.circle_radius = CIRCLE_RADIUS;
	
	control_circle_vel_mode_init(&circle);

	nav_backup_acc_bias();
	
	INFO(DEBUG_ID,"circle init");
}


void calc_angle_rate(float dt,circle_controller* circle)
{
	attitude_get(&att);
	if(att.valid == true){
		if(init_att_set_flag == 0)
		{
			att_pre[0] = att.att[0];
			att_pre[1] = att.att[1];
			att_pre[2] = att.att[2];
			init_att_set_flag = 1;
		}else{
			circle->euler_angle_rate[0] = DEG_TO_RAD * (att.att[0] - att_pre[0])/dt;
			circle->euler_angle_rate[1] = DEG_TO_RAD * (att.att[1] - att_pre[1])/dt;
			circle->euler_angle_rate[2] = DEG_TO_RAD * (att.att[2] - att_pre[2])/dt;
			att_pre[0] = att.att[0];
			att_pre[1] = att.att[1];
			att_pre[2] = att.att[2];
		}
	}	
}


void calc_desired_body_r_rate(circle_controller *circle)
{
	attitude_get(&att);
	float roll,pitch;
	roll = radians(att.att[0]);
	pitch = radians(att.att[1]);
	circle->body_z_rate_d = -circle->euler_angle_rate[1] * sinf(roll) + circle->circle_yaw_angle_rate * cosf(pitch) * cosf(roll);	
}

void control_circle_rate_yaw_on(float dt,circle_controller *circle)
{
	calc_angle_rate(dt,circle);
	calc_desired_body_r_rate(circle);
	control_yaw_rate_common(dt,circle->body_z_rate_d,0.0f);	
}
void control_circle_rate_yaw_end(float dt)
{
	control_yaw_rate_common(dt,0,0.0f);	
}


void control_circle_vel(float dt,circle_controller *circle)
{
	float rate_limit = 0.4f;
	float vel_limit = 0.4f;
	control_vel_grd_xy_common(dt,circle->circle_vel_d,vel_limit,rate_limit);	
}

void control_circle_log_pos_point(circle_controller *circle)
{
	float heading = 0;
	if(circle->circle_direction == Y_P || circle->circle_direction == Y_N){
		heading = circle->grd_heading + 180;
		if(heading > 360){
			heading -= 360;
		}
		circle->pos_grd_d[0] = CIRCLE_RADIUS * cosf(radians(heading)) + CIRCLE_RADIUS;
		circle->pos_grd_d[1] = CIRCLE_RADIUS * sinf(radians(heading));
	}else{
		heading = circle->grd_heading + 270;
		if(heading > 360){
			heading -= 360;
		}	
		circle->pos_grd_d[0] = CIRCLE_RADIUS * cosf(radians(heading));
		circle->pos_grd_d[1] = CIRCLE_RADIUS * sinf(radians(heading)) + CIRCLE_RADIUS;
	}
}


void control_circle_vel_on(float dt,rc_s * rc,circle_controller *circle)
{
	float grd_heading_deg = 0;
	float c_y = 0,s_y = 0;
	float vx = 0,vy = 0;
	
	attitude_get(&att);

	circle->grd_heading = att.att[2] - circle->init_yaw;
	if(circle->grd_heading < 0){
		circle->grd_heading += 360;
	}
	grd_heading_deg = radians(circle->grd_heading);
	c_y = cosf(grd_heading_deg);
	s_y = sinf(grd_heading_deg);
	circle->body_z_rate_c = att.rate[2];
	calc_pos_grd_c(circle);
	control_circle_vel(dt,circle);
	control_circle_log_pos_point(circle);
	vx = nav_get_vel_grd_x();
	vy = -nav_get_vel_grd_x();
	circle->circle_vel_c[0] = nav_get_vel_grd_x();
	circle->circle_vel_c[1] = nav_get_vel_grd_y();
	circle->vel_axis[0] = vx * c_y + vy * s_y;
	circle->vel_axis[1] = vx * s_y - vy * c_y;
	circle->pos_axis[0] += circle->vel_axis[0] * dt;
	circle->pos_axis[1] += circle->vel_axis[1] * dt;
	control_alt_common_rc(dt,rc);
	control_circle_rate_yaw_on(dt,circle);
	if(circle->circle_on_flag == 0){
		if(fabs(att.att[2] - circle->init_yaw) > 90){
			circle->circle_on_flag = 1;
		}
	}else{
		if(fabs(att.att[2] - circle->init_yaw) < 5){
			circle->circle_on_flag = 0;
			circle->status = CIRCLE_END;
		}
	}
	if(fabs(rc->roll) > 0 || fabs(rc->pitch) > 0 || fabs(rc->yaw) > 0 || fabs(rc->thr) > 0){
		circle->status = CIRCLE_STOP;
	}
}


void control_circle_vel_end(float dt,rc_s * rc,circle_controller *circle)
{
	calc_pos_grd_c(circle);
	control_circle_vel(dt,circle);
	control_alt_common_rc(dt,rc);
	control_circle_rate_yaw_end(dt);
	
	if(circle->circle_direction == Y_N){
		if(circle->pos_grd_c[1] < 0){
			circle->status = CIRCLE_OFF;
		}	
	}
	
	if(circle->circle_direction == Y_P){
		if(circle->pos_grd_c[1] > 0){
			circle->status = CIRCLE_OFF;
		}	
	}
	
	if(fabs(rc->roll) > 0 || fabs(rc->pitch) > 0 || fabs(rc->yaw) > 0 || fabs(rc->thr) > 0){
		circle->status = CIRCLE_STOP;
	}
}


void control_circle_stop()
{
	control_set_normal_mode();
}

void control_circle_off()
{
	control_set_normal_mode();
}


void cintrol_circle_pid_status(circle_controller* circle)
{
	switch(circle->status){
		case CIRCLE_START:
			pos_control_set_vel_pid_normal_mode();
			att_control_set_att_pid_normal_mode();
			break;
		case CIRCLE_ON:
			pos_control_set_vel_pid_circle_mode();
			att_control_set_att_pid_circle_mode();
			break;
		case CIRCLE_STOP:
			pos_control_set_vel_pid_normal_mode();
			att_control_set_att_pid_normal_mode();
			break;
		case CIRCLE_OFF:
			pos_control_set_vel_pid_normal_mode();
			att_control_set_att_pid_normal_mode();
			break;
		default:
			break;
		}
}
	
void control_circle_update(float dt,rc_s * rc)
{
	cintrol_circle_pid_status(&circle);
	switch(circle.status){
		case CIRCLE_ON:
			control_circle_vel_on(dt,rc,&circle);
			break;
		case CIRCLE_END:
			control_circle_vel_end(dt,rc,&circle);
			break;
		case CIRCLE_STOP:
			control_circle_stop();
			break;
		case CIRCLE_OFF:
			control_circle_off();
			break;
		default:
			break;
	}
}
