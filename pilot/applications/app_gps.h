#ifndef _APP_GPS_H_
#define _APP_GPS_H_

#include <stdbool.h>

#include "ubx.h"

void gps_init(void);
void gps_update(float dt);
uint8_t gps_get_status();
bool gps_get_info(gps_info_s * info);
float gps_get_error_time();

#endif
