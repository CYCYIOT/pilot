#ifndef _HAL_HMC5883_H_
#define _HAL_HMC5883_H_

#include <stdbool.h>

bool hmc5883_open();
bool hmc5883_read(float dt,float mag[3]);

#endif

