#ifndef _LIB_PID_H_
#define _LIB_PID_H_

#include "lib_lpf2p.h"	

typedef struct
{
	lpf2p_s lpf2p;
	float lp_cutoff_freq;
	
	float error_now;       
	float error_pre;
	float error_last;
	float kp;           
	float ki;           
	float kd;           
	float output_f;
	float output;
	float ki_max;     
	float ki_min;     
	float kp_val;           
	float ki_val;           
	float ki_init_val;           
	float kd_val;           
	float weight_d;
}pid_s;

void pid_init(pid_s * pid,float p,float i,float d,float imax,float imin);
void pid_reset(pid_s * pid);
void pid_clear_error(pid_s * pid);
float pid_update(pid_s * pid,float error,float dt);

void pid_set_pid(pid_s * pid, float p,float i,float d);
void pid_set_p(pid_s * pid, float p);
void pid_set_d(pid_s * pid, float d);
void pid_set_d_w(pid_s * pid, float weight);
void pid_set_i(pid_s * pid, float i);
void pid_set_i_limit(pid_s * pid, float imax,float imin);
void pid_set_i_init(pid_s * pid, float i_init);
void pid_set_lp(pid_s * pid,float sample_freq,float cutoff_freq);


#endif

