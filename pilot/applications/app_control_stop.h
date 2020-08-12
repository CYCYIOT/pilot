#ifndef _APP_CONTROL_STOP_H_
#define _APP_CONTROL_STOP_H_

#include <stdbool.h>

bool control_stop_check();
void control_stop_exit();
void control_stop_init(float param1,float param2);
void control_stop_param_init();
void control_stop_update(float dt,rc_s * rc);

#endif

