#ifndef _HAL_ARDUINO_H_
#define _HAL_ARDUINO_H_
#include<stdio.h>
#include <stdint.h>
#include <stdbool.h>



bool arduino_open();
int arduino_write(uint8_t * w_buf,int len);

void arduino_update(float dt);
void* arduino_thread(void *arg);
int uart_shooting(uint8_t id);
void set_marking(uint8_t mark_shoot);


#endif



