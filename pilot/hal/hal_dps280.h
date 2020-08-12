#ifndef _HAL_DPS280_H_
#define _HAL_DPS280_H_

#include <stdbool.h>

bool dps280_open();
bool dps280_read(float dt,float * pres,float * alt,float * temp);
void set_takeoff_flag(int vel);

#endif

