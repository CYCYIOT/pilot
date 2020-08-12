#ifndef _HAL_MMC5883_H_
#define _HAL_MMC5883_H_

#include <stdbool.h>

bool mmc5883_open(void);
bool mmc5883_read(float dt,float mag[3]);

#endif

