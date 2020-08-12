#ifndef _HAL_TOF_VL53L1X_H_
#define _HAL_TOF_VL53L1X_H_

#include <stdbool.h>

bool tof_vl53l1x_open();
void i2c_write_tof(unsigned int index,unsigned char *data,unsigned int count,unsigned char byte ,unsigned short word,unsigned int write_flag);
void  i2c_read_tof(unsigned int index, unsigned char *data, unsigned int count,uint8_t *byte,uint16_t *word,unsigned int read_flag);
bool get_tof_data(float * dist);
int tof_thread_main(void* p);
bool tx_tof_data(float *dt,float * dist);
bool hal_get_tof_fd();
float hal_get_tof_data();
float get_tof_data_yaw();

#endif

