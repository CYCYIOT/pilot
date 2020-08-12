#ifndef _HAL_KS103_H_
#define _HAL_KS103_H_

#include <stdbool.h>

bool ks103_open();
bool ks103_read(float dt,float * dist);

#endif

