#ifndef _APP_CONTROL_ALTHOLD_H_
#define _APP_CONTROL_ALTHOLD_H_

#include <stdbool.h>

bool control_althold_check();
void control_althold_init(float param1,float param2);
void control_althold_exit();
void control_althold_param_init();
void control_althold_update(float dt,rc_s * rc);
void control_althold_get_break_time(float time[2]);
void control_althold_get_break_angle(float angle[2]);
uint8_t control_althold_get_status_curr();

#endif

