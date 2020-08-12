#ifndef _APP_ATT_CONTROL_H_
#define _APP_ATT_CONTROL_H_

#include "lib_pid.h"
#include "lib_math.h"

void att_control_init(void);
void att_control_param_init(void);
void att_control_reinit(void);

void att_control_set_target_att_roll(float roll);
void att_control_set_target_att_pitch(float pitch);
void att_control_set_target_att_yaw(float yaw);
void att_control_set_target_rate_roll(float roll);
void att_control_set_target_rate_pitch(float pitch);
void att_control_set_target_rate_yaw(float yaw);

float att_control_att_roll_update(float dt);
float att_control_att_pitch_update(float dt);
float att_control_att_yaw_update(float dt);
float att_control_rate_roll_update(float dt);
float att_control_rate_pitch_update(float dt);
float att_control_rate_yaw_update(float dt);

float att_control_get_target_att_roll();
float att_control_get_target_att_pitch();
float att_control_get_target_att_yaw();
float att_control_get_target_rate_roll();
float att_control_get_target_rate_pitch();
float att_control_get_target_rate_yaw();

void att_control_set_att_pid_takeoff_mode();
void att_control_set_att_pid_normal_mode();
void att_control_set_att_pid_flip_mode();
void att_control_set_att_pid_error_clear();
void att_control_set_att_pid_circle_mode();

pid_s * att_control_get_pid(int8_t no);

#endif
