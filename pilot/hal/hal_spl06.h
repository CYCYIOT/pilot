#ifndef _HAL_SPL06_H_
#define _HAL_SPL06_H_

#include <stdbool.h>

bool spl06_open();
bool spl06_read(float dt,float * pres,float * alt,float * temp);

#endif

