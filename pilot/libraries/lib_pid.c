#include "lib_math.h"
#include "lib_pid.h"

#include "param.h"

void pid_init(pid_s * pid,float p,float i,float d,float imax,float imin)
{
	if(pid == NULL){
		return;
	}

	pid->lp_cutoff_freq = 0.0f;
	pid->error_now = 0.0f;
	pid->error_pre = 0.0f;
	pid->error_last = 0.0f;
	pid->ki_max = imax;
	pid->ki_min = imin;
	pid->kp = p;
	pid->ki = i;
	pid->kd = d;
	pid->weight_d = 1.0f;
	pid->output = 0.0f;
	pid->output_f = 0.0f;
	pid->ki_init_val = 0.0f;
	pid->ki_val = 0.0f;
	pid->kd_val = 0.0f;
}

void pid_reset(pid_s * pid)
{
	pid->error_now = 0.0f;
	pid->error_pre = 0.0f;
	pid->error_last = 0.0f;
	pid->output = 0;
	pid->kd_val = 0;
	pid->ki_val = pid->ki_init_val;
	pid->ki_val = constrain_float(pid->ki_val,pid->ki_min,pid->ki_max);
}

void pid_clear_error(pid_s * pid)
{
	pid->error_now = 0.0f;
	pid->error_pre = 0.0f;
	pid->error_last = 0.0f;
}

float pid_update(pid_s * pid,float error,float dt)
{
	float new_d_val = 0.0f;
	
	pid->error_now = error;
	
	if(dt < 0.0001f){
		pid->output = 0;
	}else{
		pid->kp_val = pid->kp * pid->error_now;
		
		pid->ki_val += pid->ki * pid->error_now * dt;
		pid->ki_val = constrain_float(pid->ki_val,pid->ki_min,pid->ki_max);
		
		new_d_val = (pid->error_now - pid->error_pre) * pid->kd / dt;
		pid->kd_val = new_d_val * pid->weight_d + (1.0f - pid->weight_d) * pid->kd_val;

		pid->output = pid->kp_val + pid->ki_val + pid->kd_val;

		if(pid->lp_cutoff_freq > 0.001f){
			pid->output_f = lpf2p_update(&pid->lpf2p,pid->output);
		}
	}
	
	pid->error_pre = pid->error_now;
	pid->error_last = pid->error_pre;


	if(pid->lp_cutoff_freq > 0.001f){
		return pid->output_f;
	}else{
		return pid->output;
	}
}

void pid_set_pid(pid_s * pid, float p,float i,float d)
{
	pid->kp = p;
	pid->ki = i;
	pid->kd = d;
}

void pid_set_p(pid_s * pid, float p)
{
	pid->kp = p;
}

void pid_set_i(pid_s * pid, float i)
{
	pid->ki = i;
}

void pid_set_i_init(pid_s * pid, float i_init)
{
	pid->ki_init_val = i_init;
}

void pid_set_i_limit(pid_s * pid, float imax,float imin)
{
	pid->ki_max = imax;
	pid->ki_min = imin;
}

void pid_set_d(pid_s * pid, float d)
{
    pid->kd = d;
}

void pid_set_d_w(pid_s * pid, float weight)
{
    pid->weight_d = constrain_float(weight,0.0f,1.0f);
}

void pid_set_lp(pid_s * pid,float sample_freq,float cutoff_freq)
{
	pid->lp_cutoff_freq = cutoff_freq;

	if(pid->lp_cutoff_freq > 0.001f){
		lpf2p_init(&pid->lpf2p,sample_freq,pid->lp_cutoff_freq);
	}
}

