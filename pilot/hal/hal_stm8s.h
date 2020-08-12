#ifndef _HAL_STM8S_H_
#define _HAL_STM8S_H_
#include <stdint.h>
#include <stdbool.h>

#define LED_STATUS_STARTING	0
#define LED_STATUS_STANDBY	1
#define LED_STATUS_FLYING	2
#define LED_STATUS_LOWBATT	3
#define LED_STATUS_CHARGING	4

bool stm8s_open();
bool stm8s_write(float *motor,uint8_t num);
bool stm8s_write_bytes(uint8_t *dat,uint8_t num_dat);
bool stm8s_write_M();

bool stm8s_read(uint8_t * val,uint8_t len);

bool stm8s_Init(uint8_t Prescale,uint8_t Period);	

bool stm8s_STOP();
void sig_handler1(void);

void terminate1(int sig_no);
int stm8_get_cnt();
int stm8_get_error();

void stm8s_set_led_charging();
void stm8s_set_led_lowbatt();
void stm8s_set_led_starting();
void stm8s_set_led_standby();
void stm8s_set_led_flying();
void stm8s_get_motor_val(uint8_t *m_val);
void stm8s_set_led(uint8_t status);


#endif

