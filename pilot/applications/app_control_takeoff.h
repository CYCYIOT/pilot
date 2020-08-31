#ifndef _APP_CONTROL_TAKEOFF_H_
#define _APP_CONTROL_TAKEOFF_H_

#include <stdbool.h>

bool control_takeoff_check();
void control_takeoff_exit();
void control_takeoff_init(float param1,float param2);
void control_takeoff_param_init();
void control_takeoff_update(float dt,rc_s * rc);
void set_takeoff_alt(int alt);
bool get_takeoff_flag();
void set_takeoff_flag_alt();
void set_takeoff_flag_poshold();
bool get_takeoff_flag_poshold();
bool get_takeoff_flag_posalt();
void set_takeoff_flag_posalt();


#endif

