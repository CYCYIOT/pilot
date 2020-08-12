#ifndef _APP_CONTROL_THROWN_H_
#define _APP_CONTROL_THROWN_H_

#include <stdbool.h>

bool control_thrown_check();
void control_thrown_exit();
void control_thrown_init(float param1,float param2);
void control_thrown_param_init();
void control_thrown_update(float dt,rc_s * rc);

#endif

