#ifndef _HAL_NMEA_H_
#define _HAL_NMEA_H_

#include <stdbool.h>

bool nmea_open();
bool nmea_read(float dt,gps_info_s *dat);

#endif

