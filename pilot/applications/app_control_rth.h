#ifndef _APP_CONTROL_RTH_H_
#define _APP_CONTROL_RTH_H_

bool control_rth_check();
void control_rth_param_init();
void control_rth_exit();
void control_rth_init(float param1,float param2);
void control_rth_update(float dt,rc_s * rc);

#endif

