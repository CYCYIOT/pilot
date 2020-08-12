#include "app_param.h"
#include "app_motor.h"
#include "app_att_control.h"
#include "app_attitude.h"

#include "lib_pid.h"

#include "param.h"

pid_s att_pid[3];
pid_s rate_pid[3];

float att_target[3];
float rate_target[3];

float pid_att_roll_p;
float pid_att_pitch_p;
float pid_att_yaw_p;

float pid_rate_roll_p;
float pid_rate_roll_i;
float pid_rate_roll_d;
float pid_rate_roll_d_w;
float pid_rate_roll_i_max;
float pid_rate_roll_i_init;

float pid_rate_pitch_p;
float pid_rate_pitch_i;
float pid_rate_pitch_d;
float pid_rate_pitch_d_w;
float pid_rate_pitch_i_max;
float pid_rate_pitch_i_init;

float pid_rate_yaw_p;
float pid_rate_yaw_i;
float pid_rate_yaw_d;
float pid_rate_yaw_i_max;
float pid_rate_yaw_i_init;

float pid_flip_rate_roll_p;
float pid_flip_rate_roll_i;
float pid_flip_rate_roll_d;

float pid_flip_rate_pitch_p;
float pid_flip_rate_pitch_i;
float pid_flip_rate_pitch_d;

float pid_flip_rate_yaw_p;
float pid_flip_rate_yaw_i;
float pid_flip_rate_yaw_d;

void att_control_param_init(void)
{
	param_set_var(PID__ATT__ROLL_P_NAME			,&pid_att_roll_p);
	param_set_var(PID__ATT__PITCH_P_NAME		,&pid_att_pitch_p);
	param_set_var(PID__ATT__YAW_P_NAME			,&pid_att_yaw_p);

	param_set_var(PID__RATE__ROLL_P_NAME		,&pid_rate_roll_p);
	param_set_var(PID__RATE__ROLL_I_NAME		,&pid_rate_roll_i);
	param_set_var(PID__RATE__ROLL_D_NAME		,&pid_rate_roll_d);
	param_set_var(PID__RATE__ROLL_D_W_NAME		,&pid_rate_roll_d_w);
	param_set_var(PID__RATE__ROLL_I_MAX_NAME	,&pid_rate_roll_i_max);
	param_set_var(PID__RATE__ROLL_I_INIT_NAME	,&pid_rate_roll_i_init);

	param_set_var(PID__RATE__PITCH_P_NAME		,&pid_rate_pitch_p);
	param_set_var(PID__RATE__PITCH_I_NAME		,&pid_rate_pitch_i);
	param_set_var(PID__RATE__PITCH_D_NAME		,&pid_rate_pitch_d);
	param_set_var(PID__RATE__PITCH_D_W_NAME		,&pid_rate_pitch_d_w);
	param_set_var(PID__RATE__PITCH_I_MAX_NAME	,&pid_rate_pitch_i_max);
	param_set_var(PID__RATE__PITCH_I_INIT_NAME	,&pid_rate_pitch_i_init);

	param_set_var(PID__RATE__YAW_P_NAME			,&pid_rate_yaw_p);
	param_set_var(PID__RATE__YAW_I_NAME			,&pid_rate_yaw_i);
	param_set_var(PID__RATE__YAW_D_NAME			,&pid_rate_yaw_d);
	param_set_var(PID__RATE__YAW_I_MAX_NAME		,&pid_rate_yaw_i_max);
	param_set_var(PID__RATE__YAW_I_INIT_NAME	,&pid_rate_yaw_i_init);

	param_set_var(PID__RATE__FLIP_ROLL_P_NAME	,&pid_flip_rate_roll_p);
	param_set_var(PID__RATE__FLIP_ROLL_I_NAME	,&pid_flip_rate_roll_i);
	param_set_var(PID__RATE__FLIP_ROLL_D_NAME	,&pid_flip_rate_roll_d);

	param_set_var(PID__RATE__FLIP_PITCH_P_NAME	,&pid_flip_rate_pitch_p);
	param_set_var(PID__RATE__FLIP_PITCH_I_NAME	,&pid_flip_rate_pitch_i);
	param_set_var(PID__RATE__FLIP_PITCH_D_NAME	,&pid_flip_rate_pitch_d);

	param_set_var(PID__RATE__FLIP_YAW_P_NAME	,&pid_flip_rate_yaw_p);
	param_set_var(PID__RATE__FLIP_YAW_I_NAME	,&pid_flip_rate_yaw_i);
	param_set_var(PID__RATE__FLIP_YAW_D_NAME	,&pid_flip_rate_yaw_d);
}

void att_control_init(void)
{
	pid_init(&att_pid[0],pid_att_roll_p		,0.0f	,0.0f	,0.0f,	0.0f);
	pid_init(&att_pid[1],pid_att_pitch_p	,0.0f	,0.0f	,0.0f,	0.0f);
	pid_init(&att_pid[2],pid_att_yaw_p		,0.0f	,0.0f	,0.0f,	0.0f);
	
	pid_init(&rate_pid[0],pid_rate_roll_p	,pid_rate_roll_i	,pid_rate_roll_d	,pid_rate_roll_i_max,	-pid_rate_roll_i_max);
	pid_init(&rate_pid[1],pid_rate_pitch_p	,pid_rate_pitch_i	,pid_rate_pitch_d	,pid_rate_pitch_i_max,	-pid_rate_pitch_i_max);
	pid_init(&rate_pid[2],pid_rate_yaw_p	,pid_rate_yaw_i		,pid_rate_yaw_d		,pid_rate_yaw_i_max,	-pid_rate_yaw_i_max);

	pid_set_i_init(&rate_pid[0],pid_rate_roll_i_init);
	pid_set_i_init(&rate_pid[1],pid_rate_pitch_i_init);
	pid_set_i_init(&rate_pid[2],pid_rate_yaw_i_init);

	pid_set_d_w(&rate_pid[0],pid_rate_roll_d_w);
	pid_set_d_w(&rate_pid[1],pid_rate_pitch_d_w);
}

void att_control_reinit(void)
{
	pid_reset(&rate_pid[0]);
	pid_reset(&rate_pid[1]);
	pid_reset(&rate_pid[2]);

	pid_set_p(&att_pid[0],pid_att_roll_p);
	pid_set_p(&att_pid[1],pid_att_pitch_p);
	pid_set_p(&att_pid[2],pid_att_yaw_p);

	pid_set_pid(&rate_pid[0],pid_rate_roll_p,pid_rate_roll_i,pid_rate_roll_d);
	pid_set_pid(&rate_pid[1],pid_rate_pitch_p,pid_rate_pitch_i,pid_rate_pitch_d);
	pid_set_pid(&rate_pid[2],pid_rate_yaw_p,pid_rate_yaw_i,pid_rate_yaw_d);
}

pid_s * att_control_get_pid(int8_t no)
{
	no = constrain_int8(no,0,2);
	return &rate_pid[no];
}

void att_control_set_att_pid_takeoff_mode()
{
	pid_set_p(&att_pid[0],pid_att_roll_p*1.5f);
	pid_set_p(&att_pid[1],pid_att_pitch_p*1.5f);

	pid_set_p(&rate_pid[0],pid_rate_roll_p*1.5f);
	pid_set_p(&rate_pid[1],pid_rate_pitch_p*1.5f);

	pid_set_i(&rate_pid[0],pid_rate_roll_i*10.0f);
	pid_set_i(&rate_pid[1],pid_rate_pitch_i*10.0f);

	pid_set_d(&rate_pid[0],pid_rate_roll_d*1.5f);
	pid_set_d(&rate_pid[1],pid_rate_pitch_d*1.5f);
}

void att_control_set_att_pid_flip_mode()
{
	pid_set_pid(&rate_pid[0],pid_flip_rate_roll_p,pid_flip_rate_roll_i,pid_flip_rate_roll_d);
	pid_set_pid(&rate_pid[1],pid_flip_rate_pitch_p,pid_flip_rate_pitch_i,pid_flip_rate_pitch_d);
	pid_set_pid(&rate_pid[2],pid_flip_rate_yaw_p,pid_flip_rate_yaw_i,pid_flip_rate_yaw_d);
}

void att_control_set_att_pid_circle_mode()
{
	pid_set_p(&att_pid[0],pid_att_roll_p);
	pid_set_p(&att_pid[1],pid_att_pitch_p);
	pid_set_p(&att_pid[2],pid_att_yaw_p);

	pid_set_pid(&rate_pid[0],pid_rate_roll_p,pid_rate_roll_i,pid_rate_roll_d);
	pid_set_pid(&rate_pid[1],pid_rate_pitch_p,pid_rate_pitch_i,pid_rate_pitch_d);
	pid_set_pid(&rate_pid[2],pid_rate_yaw_p,pid_rate_yaw_i,pid_rate_yaw_d);
}


void att_control_set_att_pid_normal_mode()
{
	pid_set_p(&att_pid[0],pid_att_roll_p);
	pid_set_p(&att_pid[1],pid_att_pitch_p);
	pid_set_p(&att_pid[2],pid_att_yaw_p);

	pid_set_pid(&rate_pid[0],pid_rate_roll_p,pid_rate_roll_i,pid_rate_roll_d);
	pid_set_pid(&rate_pid[1],pid_rate_pitch_p,pid_rate_pitch_i,pid_rate_pitch_d);
	pid_set_pid(&rate_pid[2],pid_rate_yaw_p,pid_rate_yaw_i,pid_rate_yaw_d);
}

void att_control_set_att_pid_error_clear()
{
	pid_clear_error(&rate_pid[0]);
	pid_clear_error(&rate_pid[1]);
}

void att_control_set_target_att_roll(float roll)
{
	att_target[0] = roll;
}

void att_control_set_target_att_pitch(float pitch)
{
	att_target[1] = pitch;
}

void att_control_set_target_att_yaw(float yaw)
{
	att_target[2] = yaw;
}

void att_control_set_target_rate_roll(float roll)
{
	rate_target[0] = roll;
}

void att_control_set_target_rate_pitch(float pitch)
{
	rate_target[1] = pitch;
}

void att_control_set_target_rate_yaw(float yaw)
{
	rate_target[2] = yaw;
}

float att_control_get_target_att_roll()
{
	return att_target[0];
}

float att_control_get_target_att_pitch()
{
	return att_target[1];
}

float att_control_get_target_att_yaw()
{
	return att_target[2];
}

float att_control_get_target_rate_roll()
{
	return rate_target[0];
}

float att_control_get_target_rate_pitch()
{
	return rate_target[1];
}

float att_control_get_target_rate_yaw()
{
	return rate_target[2];
}

float att_control_att_roll_update(float dt)
{
	float output;
	float att_curr_roll;
	float error;

	att_curr_roll = attitude_get_att_roll();
	error = att_target[0] - att_curr_roll;
	output = pid_update(&att_pid[0],error,dt);

	return output;
}

float att_control_att_pitch_update(float dt)
{
	float output;
	float att_curr_pitch;
	float error;

	att_curr_pitch = attitude_get_att_pitch();
	error = att_target[1] - att_curr_pitch;
	output = pid_update(&att_pid[1],error,dt);
	
	return output;
}

float att_control_att_yaw_update(float dt)
{
	float output;
	float att_curr_yaw;
	float error;
	
	att_curr_yaw = attitude_get_att_yaw();
	error = att_target[2] - att_curr_yaw;
	error = wrap_180_cd_float(error);
	output = pid_update(&att_pid[2],error,dt);
	
	return output;
}

float att_control_rate_roll_update(float dt)
{
	float output;
	float rate_curr_roll;
	float error;

	rate_curr_roll = attitude_get_rate_roll_f();
	error = rate_target[0] - rate_curr_roll;
	output = pid_update(&rate_pid[0],error,dt);
	
	return output;
}

float att_control_rate_pitch_update(float dt)
{
	float output;
	float rate_curr_pitch;
	float error;

	rate_curr_pitch = attitude_get_rate_pitch_f();
	error = rate_target[1] - rate_curr_pitch;
	output = pid_update(&rate_pid[1],error,dt);
	
	return output;
}

float att_control_rate_yaw_update(float dt)
{
	float output;
	float rate_curr_yaw;
	float error;

	rate_curr_yaw = attitude_get_rate_yaw_f();
	error = rate_target[2] - rate_curr_yaw;
	output = pid_update(&rate_pid[2],error,dt);
	
	return output;
}

