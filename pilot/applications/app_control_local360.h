#ifndef _APP_CONTROL_LOCAL_360_H_
#define _APP_CONTROL_LOCAL_360_H_

#include <stdbool.h>

bool control_local360_check();
void control_local360_exit();
void control_local360_init(float param1,float param2);
void control_local360_param_init();
void control_local360_update(float dt,rc_s * rc);

#endif

