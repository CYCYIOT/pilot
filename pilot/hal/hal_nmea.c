#include <stdbool.h>
#include <stdint.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <termios.h>

#include "app_debug.h"
#include "param.h"
#include "nmea.h"
#include "hal_linux.h"

nmea_decoder_t gps_nmea;
int nmea_uart_fd = -1;
float gps_wait_time = 0.0f;
#define DEBUG_ID DEBUG_ID_HAL
#define DEBUG_NMEA

bool nmea_open()
{
#ifdef GPS_UART_PATH
	bool ret = false;

	nmea_uart_fd = open(GPS_UART_PATH, O_RDWR | O_NONBLOCK | O_NDELAY | O_NOCTTY);
	hal_linux_uart_cfg(nmea_uart_fd,B115200);	
	if(nmea_uart_fd < 0){
		DEBUG(DEBUG_ID,"Can not open dev %s!",GPS_UART_PATH);
		ret = false;
	}else{
		DEBUG(DEBUG_ID,"open gps dev %s!\n",GPS_UART_PATH);
		ret = true;
	}
	return ret;
#endif
	return false;
}
void debug_nmea(char buf[],uint8_t num)
{
	uint8_t i;
	for(i = 0; i < num;i++){
		printf("%c",buf[i]);
	}	
}

bool nmea_read(float dt,gps_info_s *dat)
{
	bool ret = false;
	int data_length = 0;
	int cnt = 0;
	char buf[256] = {"\0"};
	data_length = read(nmea_uart_fd,buf,256);
	gps_wait_time += dt;
	if(data_length > 0){
		gps_wait_time = 0.0f;
		for(cnt = 0 ; cnt < data_length ; cnt++){
			decode_gps_nmea_byte(&gps_nmea,buf[cnt]);
		}
	}else{
		if(gps_wait_time > 0.01f && gps_nmea.gps_nmea_info.valid == true){
			gps_nmea.gps_nmea_info.valid = false;
			gps_wait_time = 0;
			nmea_copy(&gps_nmea,dat);
			ret = true;
		}else{	
			ret = false;
		}	
	}
	return ret;
}

