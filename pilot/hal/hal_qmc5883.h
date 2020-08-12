#ifndef _HAL_QMC5883_H_
#define _HAL_QMC5883_H_

#include <stdbool.h>

bool qmc5883_open();
bool qmc5883_read(float dt,float mag[3]);

#endif

