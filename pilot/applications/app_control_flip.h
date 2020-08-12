#ifndef _APP_CONTROL_FLIP_H_
#define _APP_CONTROL_FLIP_H_

#include <stdbool.h>

bool control_flip_check();
void control_flip_exit();
void control_flip_init(float param1,float param2);
void control_flip_param_init();
void control_flip_update(float dt,rc_s * rc);

float control_flip_stable_get_roll_target();
float control_flip_stable_get_pitch_target();

float control_flip_get_roll();
float control_flip_get_pitch();
float control_flip_get_yaw();
uint8_t control_flip_get_mode();

#endif

