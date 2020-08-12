#ifndef _APP_CONTROL_LAND_H_
#define _APP_CONTROL_LAND_H_

#include <stdbool.h>

bool control_land_check();
void control_land_exit();
void control_land_init(float param1,float param2);
void control_land_param_init();
void control_land_update(float dt,rc_s * rc);

void control_land_update_land_check_thr_reset();
bool control_land_update_land_check_thr(float dt);
bool control_land_update_land_check_acc(float dt);

#endif

