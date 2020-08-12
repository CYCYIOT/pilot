#ifndef _HAL_IST8310_H_
#define _HAL_IST8310_H_

#include <stdbool.h>

bool ist8310_open(void);
bool ist8310_read(float dt,float mag[3]);

#endif

