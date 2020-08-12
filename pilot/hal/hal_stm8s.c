#include <fcntl.h>
#include <unistd.h>
#include <stdlib.h>

#include "app_debug.h"
#include "app_system.h"

#include "hal_stm8s.h"

#define DEBUG_ID DEBUG_ID_HAL

#define STM8S_PATH "/dev/STM8s"
int stm8s_fd = -1;

static int stm8_error = 0;
static int stm8_cnt = 0;

bool stm8s_stop()
{
	int num;
	unsigned char out[5] = {0};
	
	num = write(stm8s_fd,out,5);
	if(num == 5){
		return true;
	}else{
		return false;
	}
}

int stm8_get_cnt()
{
	return stm8_cnt;
}

int stm8_get_error()
{
	return stm8_error;
}

bool stm8s_open()
{
	stm8s_fd = open(STM8S_PATH, O_RDWR);
	if(stm8s_fd < 0){
		DEBUG(DEBUG_ID,"open stm8s failed (%s)", STM8S_PATH);
		return false;
	}else{
		INFO(DEBUG_ID,"use stm8s motor driver");
	}
	
	return true;
}

bool stm8s_read(uint8_t * motor_val,uint8_t len)
{
	if(read(stm8s_fd, motor_val, len) > 0){
		return true;
	}else{
		return false;
	}
}


bool stm8s_write_M()
{
	int num_w = 0;
	
   unsigned char out[5]={0xff,0xff,0xff,0xff,0xab};
	
	num_w = write(stm8s_fd,out,sizeof(out));
	
	if(num_w == 5){
		return true;
	}else{
		return false;
	}
}



bool stm8s_write(float *motor,uint8_t num)
{
	int num_w = 0;
	unsigned char out[5] = {0};
	unsigned char i;
	unsigned short tmp;	

	for(i = 0;i < 4;i++){
		tmp = motor[i] * 249;
		out[i] = tmp & 0xff;
		out[4] |= (tmp >> 8) << (i * 2);		
	}
	num_w = write(stm8s_fd,out,sizeof(out));
	if(num_w == 5){
		return true;
	}else{
		return false;
	}
}

bool stm8s_init(uint8_t Prescale,uint8_t Period)	
{	
	uint8_t data[2];
	uint8_t num;
	
	data[0] = Prescale;
	data[1] = Period;
	num = write(stm8s_fd,data,sizeof(data));
	
	if(num == sizeof(data)){
		return true;
	}else{
		return false;
	}
}

bool stm8s_write_bytes(uint8_t *dat,uint8_t num_dat)
{
	uint8_t num = 0;
	num = write(stm8s_fd,dat,num_dat);	
	if(num == num_dat){
		return true;
	}else{
		return false;
	}
}

void stm8s_set_led_charging()
{
	uint8_t data[5] = {0xff,0xff,0xff,0xff,0xff};
	stm8s_write_bytes(data,5);
}

void stm8s_set_led_lowbatt()
{
	uint8_t data[5] = {0xff,0xff,0xff,0xff,0xfe};
	stm8s_write_bytes(data,5);
}

void stm8s_set_led_starting()
{
	uint8_t data[5] = {0xff,0xff,0xff,0xff,0xfb};
	stm8s_write_bytes(data,5);
}

void stm8s_set_led_standby()
{
	uint8_t data[5] = {0xff,0xff,0xff,0xff,0xfc};
	stm8s_write_bytes(data,5);
}

void stm8s_set_led_flying()
{
	uint8_t data[5] = {0xff,0xff,0xff,0xff,0xfd};
	stm8s_write_bytes(data,5);
}

void stm8s_set_led(uint8_t status)
{
	if(status == LED_STATUS_STARTING){
		stm8s_set_led_starting();
	}else if(status == LED_STATUS_STANDBY){
		stm8s_set_led_standby();
	}else if(status == LED_STATUS_FLYING){
		stm8s_set_led_flying();
	}else if(status == LED_STATUS_CHARGING){
		stm8s_set_led_charging();
	}else if(status == LED_STATUS_LOWBATT){
		stm8s_set_led_lowbatt();
	}
}


void stm8s_get_motor_val(uint8_t *m_val)
{
	uint8_t val[10] = {0};
	stm8s_read(val,6);
	m_val[0] = val[2];
	m_val[1] = val[3];
	m_val[2] = val[4];
	m_val[3] = val[5];
}

