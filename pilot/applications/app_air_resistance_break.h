#ifndef _APP_AIR_RESISTANCE_BREAK_H
#define _APP_AIR_RESISTANCE_BREAK_H

#include "app_attitude.h"
void air_res_break_angle_integrate(float dt,att_s att,float att_offset[2]);
void air_res_break_get_time(float time[2]);
void air_res_break_get_angle(float angle[2]);
void air_res_break_clear();
void air_res_break_get_angle_integrate(float integrate[2]);
float air_res_break_get_air_res_coff();
void air_res_break_set_param(float angle,float coff);

#endif

