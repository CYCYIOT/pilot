#ifndef _HAL_LIS3MDL_H_
#define _HAL_LIS3MDL_H_

#include <stdbool.h>

bool lis3mdl_open();
bool lis3mdl_read(float dt,float mag[3]);

#endif

