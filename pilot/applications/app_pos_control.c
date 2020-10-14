#include "app_param.h"
#include "app_pos_control.h"
#include "app_motor.h"
#include "app_nav.h"

#include "lib_pid.h"
#include "lib_math.h"

pid_s pos_pid[3];
pid_s vel_pid[3];
pid_s acc_pid[3];

float z_target = 0.0f;

float pid_pos_z_p = 0.0f;
float pid_pos_xy_p = 0.0f;

float pid_vel_z_p = 0.0f;
float pid_vel_z_i = 0.0f;
float pid_vel_z_i_max = 0.0f;
float pid_vel_z_i_min = 0.0f;
float pid_vel_z_i_init = 0.0f;
float pid_vel_z_d = 0.0f;
float pid_vel_z_d_w = 0.0f;

float pid_vel_xy_p = 0.0f;
float pid_vel_xy_i = 0.0f;
float pid_vel_xy_d = 0.0f;
float pid_vel_xy_d_w = 0.0f;
float pid_vel_xy_i_max = 0.0f;
float pid_vel_x_i_init = 0.0f;
float pid_vel_y_i_init = 0.0f;

float pid_acc_z_p = 0.0f;
float pid_acc_z_i = 0.0f;
float pid_acc_z_imax = 0.0f;
float pid_acc_z_d = 0.0f;

float pid_vel_break_xy_p = 0.0f;
float pid_vel_break_xy_i = 0.0f;

float pos_ned_target[3] = {0.0f,0.0f,0.0f};
float vel_ned_target[3] = {0.0f,0.0f,0.0f};
float vel_grd_target[3] = {0.0f,0.0f,0.0f};
float acc_target[3] = {0.0f,0.0f,0.0f};

float vel_pid_i_backup[2]={0};

void vel_control_xy_i_stash()
{
	vel_pid_i_backup[0] = complementary_filter(vel_pid[0].ki_val,vel_pid_i_backup[0],0.001f);
	vel_pid_i_backup[1] = complementary_filter(vel_pid[1].ki_val,vel_pid_i_backup[1],0.001f);
}

void vel_control_xy_i_pop(int i,int mode)
{
	//vel_pid[i].ki_val = vel_pid_i_backup[i];
	if(mode == 0){
		vel_pid[i].ki_val = vel_pid_i_backup[i];
	}else{
		vel_pid[i].ki_val = complementary_filter(vel_pid_i_backup[i],vel_pid[i].ki_val,0.02f);
	}
}


void pos_control_set_vel_pid_thrown_mode()
{
	pid_set_p(&vel_pid[0],pid_vel_xy_p*2.0f);
	pid_set_p(&vel_pid[1],pid_vel_xy_p*2.0f);

	pid_set_i(&vel_pid[0],pid_vel_xy_i*2.0f);
	pid_set_i(&vel_pid[1],pid_vel_xy_i*2.0f);
}


void pos_control_set_vel_pid_takeoff_mode()
{
	pid_set_p(&vel_pid[0],pid_vel_xy_p*1.2f);
	pid_set_p(&vel_pid[1],pid_vel_xy_p*1.2f);

	pid_set_i(&vel_pid[0],pid_vel_xy_i*2.0f);
	pid_set_i(&vel_pid[1],pid_vel_xy_i*2.0f);
}

void pos_control_set_vel_pid_rotate_mode()
{
	pid_set_p(&vel_pid[0],pid_vel_xy_p*2.0f);
	pid_set_p(&vel_pid[1],pid_vel_xy_p*2.0f);
}

void pos_control_set_vel_pid_flip_mode()
{
	pid_set_p(&vel_pid[0],pid_vel_xy_p * 1.0f);
	pid_set_p(&vel_pid[1],pid_vel_xy_p * 1.0f);

	pid_set_i(&vel_pid[0],pid_vel_xy_i * 0.1f);
	pid_set_i(&vel_pid[1],pid_vel_xy_i * 0.1f);
}

void pos_control_set_vel_pid_normal_mode()
{
	pid_set_pid(&vel_pid[0],pid_vel_xy_p,pid_vel_xy_i,pid_vel_xy_d);
	pid_set_pid(&vel_pid[1],pid_vel_xy_p,pid_vel_xy_i,pid_vel_xy_d);
}

void pos_control_set_alt_pid_land_mode()
{
	pid_set_i(&vel_pid[2],pid_vel_z_i*2.0f);
}

void pos_control_set_alt_pid_normal_mode()
{
	pid_set_pid(&vel_pid[2],pid_vel_z_p,pid_vel_z_i,pid_vel_z_d);
}

void pos_control_set_alt_pid_flip_mode()
{
	pid_set_p(&vel_pid[2],pid_vel_z_p * 1.3f);
	pid_set_i(&vel_pid[2],pid_vel_z_i * 0.01f);
}

void pos_control_set_vel_pid_circle_mode()
{
	pid_set_pid(&vel_pid[0],pid_vel_xy_p*1.5f,pid_vel_xy_i*1.0f,pid_vel_xy_d);
	pid_set_pid(&vel_pid[1],pid_vel_xy_p*1.5f,pid_vel_xy_i*1.0f,pid_vel_xy_d);
}

void pos_control_set_vel_pid_break_high_speed_mode()
{
	float pid_i_tmp[2];
	float pid_p_tmp[2];

	if(fabs(vel_pid[0].error_now) < 0.1f){
		pid_i_tmp[0] = pid_vel_xy_i;
		pid_p_tmp[0] = pid_vel_xy_p;
	}else if(fabs(vel_pid[0].error_now) < 1.0f){
		pid_i_tmp[0] = pid_vel_break_xy_i;
#ifdef M
       pid_p_tmp[0] = pid_vel_break_xy_p * 0.5f; 
#elif  K		
       pid_p_tmp[0] = pid_vel_break_xy_p * 0.5f;
#else
       pid_p_tmp[0] = pid_vel_break_xy_p * 0.8f;
#endif
	}else{
		pid_i_tmp[0] = 0.0f;
		pid_p_tmp[0] = pid_vel_break_xy_p;
	}

	if(fabs(vel_pid[1].error_now) < 0.1f){
		pid_i_tmp[1] = pid_vel_xy_i;
		pid_p_tmp[1] = pid_vel_xy_p;
	}else if(fabs(vel_pid[1].error_now) < 1.0f){
		pid_i_tmp[1] = pid_vel_break_xy_i;
#ifdef M		
		pid_p_tmp[1] = pid_vel_break_xy_p * 0.5f;
#elif  K	
		pid_p_tmp[1] = pid_vel_break_xy_p * 0.5f;
#else
        pid_p_tmp[1] = pid_vel_break_xy_p * 0.8f;
#endif

	}else{
		pid_i_tmp[1] = 0.0f;
		pid_p_tmp[1] = pid_vel_break_xy_p;
	}

	pid_set_pid(&vel_pid[0],pid_p_tmp[0],pid_i_tmp[0],pid_vel_xy_d);
	pid_set_pid(&vel_pid[1],pid_p_tmp[1],pid_i_tmp[1],pid_vel_xy_d);
}

void pos_control_set_vel_pid_break_low_speed_mode()
{
	float pid_i_tmp[2];
	float pid_p_tmp;

	if(fabs(vel_pid[0].error_now) < 0.3f){
		pid_i_tmp[0] = pid_vel_break_xy_i;
	}else{
		pid_i_tmp[0] = 0.0f;
	}

	if(fabs(vel_pid[1].error_now) < 0.3f){
		pid_i_tmp[1] = pid_vel_break_xy_i;
	}else{
		pid_i_tmp[1] = 0.0f;
	}
	
	pid_p_tmp = pid_vel_break_xy_p;

	pid_set_pid(&vel_pid[0],pid_p_tmp,pid_i_tmp[0],pid_vel_xy_d);
	pid_set_pid(&vel_pid[1],pid_p_tmp,pid_i_tmp[1],pid_vel_xy_d);
}

void pos_control_param_init()
{
	param_set_var(PID__POS__Z_P_NAME		,&pid_pos_z_p);
	param_set_var(PID__POS__XY_P_NAME		,&pid_pos_xy_p);

	param_set_var(PID__VEL__XY_P_NAME		,&pid_vel_xy_p);
	param_set_var(PID__VEL__XY_I_NAME		,&pid_vel_xy_i);
	param_set_var(PID__VEL__XY_D_NAME		,&pid_vel_xy_d);
	param_set_var(PID__VEL__XY_D_W_NAME		,&pid_vel_xy_d_w);
	param_set_var(PID__VEL__XY_I_MAX_NAME	,&pid_vel_xy_i_max);
	param_set_var(PID__VEL__X_I_INIT_NAME	,&pid_vel_x_i_init);
	param_set_var(PID__VEL__Y_I_INIT_NAME	,&pid_vel_y_i_init);

	param_set_var(PID__VEL__Z_P_NAME		,&pid_vel_z_p);
	param_set_var(PID__VEL__Z_I_NAME		,&pid_vel_z_i);
	param_set_var(PID__VEL__Z_D_NAME		,&pid_vel_z_d);
	param_set_var(PID__VEL__Z_D_W_NAME		,&pid_vel_z_d_w);
	param_set_var(PID__VEL__Z_I_MAX_NAME	,&pid_vel_z_i_max);
	param_set_var(PID__VEL__Z_I_MIN_NAME	,&pid_vel_z_i_min);
	param_set_var(PID__VEL__Z_I_INIT_NAME	,&pid_vel_z_i_init);
	
	param_set_var(PID__VEL__BREAK_XY_P_NAME	,&pid_vel_break_xy_p);
	param_set_var(PID__VEL__BREAK_XY_I_NAME	,&pid_vel_break_xy_i);	
}

void pos_control_init()
{
	pid_init(&pos_pid[0],pid_pos_xy_p	,0.0f	,0.0f	,0.0f,	0.0f);
	pid_init(&pos_pid[1],pid_pos_xy_p	,0.0f	,0.0f	,0.0f,	0.0f);
	pid_init(&pos_pid[2],pid_pos_z_p	,0.0f	,0.0f	,0.0f,	0.0f);

	pid_init(&vel_pid[0],pid_vel_xy_p	,pid_vel_xy_i	,pid_vel_xy_d	,pid_vel_xy_i_max,	-pid_vel_xy_i_max);
	pid_init(&vel_pid[1],pid_vel_xy_p	,pid_vel_xy_i	,pid_vel_xy_d	,pid_vel_xy_i_max,	-pid_vel_xy_i_max);
	pid_init(&vel_pid[2],pid_vel_z_p	,pid_vel_z_i	,pid_vel_z_d	,pid_vel_z_i_max,	pid_vel_z_i_min);

	pid_set_i_init(&vel_pid[0],pid_vel_x_i_init);
	pid_set_i_init(&vel_pid[1],pid_vel_y_i_init);
	pid_set_i_init(&vel_pid[2],pid_vel_z_i_init);

	pid_set_d_w(&vel_pid[0],pid_vel_xy_d_w);
	pid_set_d_w(&vel_pid[1],pid_vel_xy_d_w);
	pid_set_d_w(&vel_pid[2],pid_vel_z_d_w);

	pid_init(&acc_pid[2],pid_acc_z_p	,pid_acc_z_i	,pid_acc_z_d	,pid_acc_z_imax, -pid_acc_z_imax);
}

void pos_control_xy_reinit()
{
	pid_reset(&pos_pid[0]);
	pid_reset(&pos_pid[1]);
	
	pid_reset(&vel_pid[0]);
	pid_reset(&vel_pid[1]);

	pid_set_p(&pos_pid[0],pid_pos_xy_p);
	pid_set_p(&pos_pid[1],pid_pos_xy_p);
	
	pid_set_pid(&vel_pid[0],pid_vel_xy_p,pid_vel_xy_i,pid_vel_xy_d);
	pid_set_pid(&vel_pid[1],pid_vel_xy_p,pid_vel_xy_i,pid_vel_xy_d);
}

void pos_control_reinit()
{
	pos_control_xy_reinit();

	pid_reset(&vel_pid[2]);
	pid_set_pid(&vel_pid[2],pid_vel_z_p,pid_vel_z_i,pid_vel_z_d);
	
	pid_set_p(&pos_pid[2],pid_pos_z_p);
}

pid_s * pos_control_get_pid(int8_t no)
{
	no = constrain_int8(no,0,2);
	return &vel_pid[no];
}

float pos_control_get_pos_ned_x_target()
{
	return pos_ned_target[0];
}

float pos_control_get_vel_ned_x_target()
{
	return vel_ned_target[0];
}

float pos_control_get_vel_grd_x_target()
{
	return vel_grd_target[0];
}

float pos_control_get_pos_ned_y_target()
{
	return pos_ned_target[1];
}

float pos_control_get_vel_ned_y_target()
{
	return vel_ned_target[1];
}

float pos_control_get_vel_grd_y_target()
{
	return vel_grd_target[1];
}

float pos_control_get_pos_ned_z_target()
{
	return pos_ned_target[2];
}

float pos_control_get_vel_ned_z_target()
{
	return vel_ned_target[2];
}

float pos_control_get_acc_z_target()
{
	return acc_target[2];
}

void pos_control_set_pos_ned_x_target(float pos)
{
	pos_ned_target[0] = pos;
}

void pos_control_set_vel_ned_x_target(float vel)
{
	vel_ned_target[0] = vel;
}

void pos_control_set_vel_grd_x_target(float vel)
{
	vel_grd_target[0] = vel;
}

void pos_control_set_pos_ned_y_target(float pos)
{
	pos_ned_target[1] = pos;
}

void pos_control_set_vel_ned_y_target(float vel)
{
	vel_ned_target[1] = vel;
}

void pos_control_set_vel_grd_y_target(float vel)
{
	vel_grd_target[1] = vel;
}

void pos_control_set_pos_ned_z_target(float pos)
{
	pos_ned_target[2] = pos;
}

void pos_control_set_vel_ned_z_target(float vel)
{
	vel_ned_target[2] = vel;
}

void pos_control_set_acc_z_target(float acc)
{
	acc_target[2] = acc;
}

float pos_control_pos_ned_z_update(float dt)
{
	float pos_z_curr;
	float output;
	float error;
	
	pos_z_curr = nav_get_pos_ned_z();
	error = pos_ned_target[2] - pos_z_curr;
	output = pid_update(&pos_pid[2],error,dt);

	return output;
}

float pos_control_vel_ned_z_update(float dt)
{
	float vel_z_curr;
	float output;
	float error;

	vel_z_curr = nav_get_vel_ned_z();
	error = vel_ned_target[2] - vel_z_curr;
	output = pid_update(&vel_pid[2],error,dt);

	return output;
}

float pos_control_acc_ned_z_update(float dt)
{
	float acc_z_curr;
	float output;
	float error;

	acc_z_curr = nav_get_acc_ned_z();
	error = acc_target[2] - acc_z_curr;
	output = pid_update(&acc_pid[2],error,dt);

	return output;
}

float pos_control_pos_ned_x_update(float dt)
{
	float pos_x_curr = 0.0f;
	float output;
	float error;
	
	pos_x_curr = nav_get_pos_ned_x();
	error = pos_ned_target[0] - pos_x_curr;
	output = pid_update(&pos_pid[0],error,dt);

	return output;
}

float pos_control_vel_grd_x_update(float dt)
{
	float vel_x_curr = 0.0f;
	float output;
	float error;

	vel_x_curr = nav_get_vel_grd_x();
	error = vel_grd_target[0] - vel_x_curr;
	output = pid_update(&vel_pid[0],error,dt);

	return output;
}

float pos_control_pos_ned_y_update(float dt)
{
	float pos_y_curr = 0.0f;
	float output;
	float error;
	
	pos_y_curr = nav_get_pos_ned_y();
	error = pos_ned_target[1] - pos_y_curr;
	output = pid_update(&pos_pid[1],error,dt);

	return output;
}

float pos_control_vel_grd_y_update(float dt)
{
	float vel_y_curr = 0.0f;
	float output;
	float error;

	vel_y_curr = nav_get_vel_grd_y();
	error = vel_grd_target[1] - vel_y_curr;
	output = pid_update(&vel_pid[1],error,dt);

	return output;
}


