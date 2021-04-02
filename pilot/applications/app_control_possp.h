#ifndef _APP_CONTROL_POSSP_H_
#define _APP_CONTROL_POSSP_H_

bool control_possp_check();
void control_possp_param_init();
void control_possp_exit();
void control_possp_init(float param1,float param2);
void control_possp_update(float dt,rc_s * rc);

#endif

