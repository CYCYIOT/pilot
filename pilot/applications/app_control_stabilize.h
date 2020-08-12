#ifndef _APP_CONTROL_STABILIZE_H_
#define _APP_CONTROL_STABILIZE_H_

#include <stdbool.h>

bool control_stabilize_check();
void control_stabilize_init(float param1,float param2);
void control_stabilize_exit();
void control_stabilize_param_init();
void control_stabilize_update(float dt,rc_s * rc);

#endif

