#ifndef _HAL_ARDUINO_H_
#define _HAL_ARDUINO_H_
#include<stdio.h>
#include <stdint.h>
#include <stdbool.h>



bool arduino_open();
bool arduino_write(uint8_t * w_buf,int len);

void arduino_update(float dt);


#endif



