#ifndef _APP_CONTROL_ARUCO_H_
#define _APP_CONTROL_ARUCO_H_

bool control_aruco_check();
void control_aruco_param_init();
void control_aruco_exit();
void control_aruco_init(float param1,float param2);
void control_aruco_update(float dt,rc_s * rc);

#endif

