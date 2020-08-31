#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include "hal_led.h"
#include "app_debug.h"

#define DEBUG_ID DEBUG_ID_HAL

static int led_fd = -1;
bool led_open(){
	
 led_fd = open("/sys/class/leds/led1/blink",O_WRONLY);
 if(led_fd < 0){
	 printf("open fail\n");
	 return false;
 }
 INFO(DEBUG_ID,"use led ok");
 return true;
}

int led_blink(unsigned int count){	
	char str_count[2];
	int ret = -1;
	
	sprintf(str_count, "%02x", count);
	if(write(led_fd, str_count, 2) > 0){
		ret = 0;
	}	
	return ret;
	
}




