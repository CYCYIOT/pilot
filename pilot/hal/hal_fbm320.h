#ifndef _HAL_FBM320_H_
#define _HAL_FBM320_H_

#include <stdbool.h>

bool fbm320_open();
bool fbm320_read(float dt,float * pres,float * alt,float * temp);

#endif

