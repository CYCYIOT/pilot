#ifndef _HAL_TOF_VL5300_H_
#define _HAL_TOF_VL5300_H_

#include <stdbool.h>
#include <stdint.h>


bool tof_vl5300_open();
bool tx_tof_data_vl5300(float *dt,float * dist);
bool hal_get_tof_fd_vl5300();

#endif

