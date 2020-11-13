#ifndef _HAL_ARDUINO_H_
#define _HAL_ARDUINO_H_
#include<stdio.h>
#include <stdint.h>
#include <stdbool.h>


#define UART_CONTROL__GET_SHOOTING   0x0d     //o����a������?
#define UART_CONTROL_GET_ULTRASONIC  0x0a    //3?����2��������?
#define UART_CONTROL_GET_JOYSTICK    0xfa    //����??������?
#define UART_CONTROL_GET_GPS         0x88
bool arduino_open();
int arduino_write(uint8_t * w_buf,int len);

void arduino_update(float dt);
void* arduino_thread(void *arg);
int uart_shooting(uint8_t id);
void set_marking(uint8_t mark_shoot);


#endif



