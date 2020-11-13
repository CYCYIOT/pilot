#ifndef _HAL_ARDUINO_H_
#define _HAL_ARDUINO_H_
#include<stdio.h>
#include <stdint.h>
#include <stdbool.h>


#define UART_CONTROL__GET_SHOOTING   0x0d     //o足赤a㊣那那?
#define UART_CONTROL_GET_ULTRASONIC  0x0a    //3?谷迄2“㊣那那?
#define UART_CONTROL_GET_JOYSTICK    0xfa    //辰㏒??㊣那那?
#define UART_CONTROL_GET_GPS         0x88
bool arduino_open();
int arduino_write(uint8_t * w_buf,int len);

void arduino_update(float dt);
void* arduino_thread(void *arg);
int uart_shooting(uint8_t id);
void set_marking(uint8_t mark_shoot);


#endif



