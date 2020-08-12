#include <stdbool.h>
#include <stdint.h>
#include <math.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <termios.h>

#include "param.h"
#include "hal_linux.h"
#include "app_awlink.h"
#include "awlink_protocol.h"
#include "app_arduino.h"
#include "lib_math.h"
#include "app_debug.h"
#include "app_rc.h"
#define DEBUG_ID DEBUG_ID_HAL

#define UART_ARDUINO  "/dev/ttyS2"

static int arduino_fd=-1;

bool arduino_open()
{

	arduino_fd = open(UART_ARDUINO, O_RDWR | O_NONBLOCK | O_NDELAY | O_NOCTTY);   //打开串口文件

	if(arduino_fd < 0){
		DEBUG(DEBUG_ID,"arduino init failed");
		return false;
	}else{
		hal_linux_uart_cfg(arduino_fd,B115200);  //配置串口
		INFO(DEBUG_ID,"use arduino ttyS2");
	}
	
	return true;
}

bool arduino_read()
{
	
	int num;
	uint8_t i;
	uint8_t rx_buf[15];

	num = read(arduino_fd,rx_buf,15);
	if(num > 0){
		for(i = 0;i < num;i++){
			printf("rx_buf[%d] = %d\n",i,rx_buf[i]);
		}
	 return true;	
	}

	return false;
}




bool arduino_write(uint8_t * w_buf,int len)
{
  int num;
  num = write(arduino_fd,w_buf,len);
  if(num == len){
	printf("write ok num = %d\n",num);
   return true;
  }
 return false;
}

static void crc_check(uint8_t *data,int len)
{
	int i=0;
	uint16_t checksum=crc16_init();
	for(i=1;i<len-2;i++){
	  checksum=crc16_update(data[i],checksum);
	}
	data[len-2]=checksum&0xff;
	data[len-1]=(checksum>>8)&0xff;
#if 0
    printf("=======================================\n");
	for(i = 0;i < len;i++){
			printf("rx_buf[%d] = %d %x \n",i,data[i],data[i]);
		}
#endif	
	recv_uart(data,len);
} 

int count_a =0;

void arduino_update(float dt)
{

    int num;
	uint8_t i;
	uint8_t rx_buf[100];
 #if 0   
	uint8_t buf[] ={0x3f,0x04,0x00,0xff,0x22,0xdd};    // 测试红外通信
    if(count_a ++ >1000){
	arduino_write(buf,6);
    count_a=0;
	}
#endif
	rc_awlink_set_rc_h();                      //主动刷新心跳，无断线自动降落功能
  
	num = read(arduino_fd,rx_buf,100);
	if(num > 0){
		for(i = 0;i < num;i++){
			printf("rx_buf[%d] = %d %x \n",i,rx_buf[i],rx_buf[i]);
		}

	   if(arduino_control(rx_buf[0]) == true ){     //先判断是不是自定义私有协议
         return ;
	   }

	   if(rx_buf[0] == AWLINK_MAGIC){
		 crc_check(rx_buf,num);
		 }else{
         DEBUG(DEBUG_ID,"uart send error\n");
		 crc_check(&rx_buf[8],num-8);          //串口数据会多发8位，当接受的第一位数据不是0xFA时，需要后移8位才是需要的数据
	    }
	}
  
}




