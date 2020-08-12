#ifndef _HAL_VL53L1X_H_
#define _HAL_VL53L1X_H_

#include <stdbool.h>

bool vl53l1x_open();
bool vl53l1x_read(float dt,float * dist);

uint8_t vl53l1x_get_status();

#endif

