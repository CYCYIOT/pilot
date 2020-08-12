#ifndef _APP_CONTROL_POSHOLD_H_
#define _APP_CONTROL_POSHOLD_H_

#include <stdbool.h>
#include "app_rc.h"
#include "app_nav.h"

bool control_poshold_check();
void control_poshold_exit();
void control_poshold_init(float param1,float param2);
void control_poshold_param_init();
void control_poshold_update(float dt,rc_s * rc);

void control_poshold_get_break1_time(float time[2]);
uint8_t control_poshold_get_mode();
void control_poshold_set_break2_mode(int i);
bool control_poshold_get_break2_mode();

void set_pt_flag();
bool get_pt_flag();

#endif

