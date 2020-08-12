#ifndef _HAL_UBX_H_
#define _HAL_UBX_H_

#include <stdbool.h>

bool ubx_open();
bool ubx_read(float dt,gps_info_s *dat);
bool gps_existed_check();

#endif

