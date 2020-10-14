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
#include "app_awlink.h"

#define DEBUG_ID DEBUG_ID_HAL

#define UART_ARDUINO  "/dev/ttyS2"

static int arduino_fd=-1;

static uint8_t marking = 100; 
bool arduino_open()
{
 
	arduino_fd = open(UART_ARDUINO, O_RDWR | O_NONBLOCK| O_NDELAY | O_NOCTTY );   //打开串口文件

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
int arduino_write(uint8_t * w_buf,int len)
{
  int num;
  num = write(arduino_fd,w_buf,len);
  if(num == len){
	//printf("write ok num = %d\n",num);
   return num;
  }
 return -1;
}

int uart_shooting(uint8_t id)
{
   int ret=0;
   uint8_t buf[] ={0x3f,0x04,0x0d,0x00,0x00,0x01};    // 红外通信协议，后四位可以自定，
   buf[5]=id;		
   ret = arduino_write(buf,6);
   if(ret != 6){
    arduino_write(buf,6);
   }
  return 0;
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
	
	recv_uart(data,len);
} 

void set_marking(uint8_t mark_shoot)
{
  marking=mark_shoot;
}
void Parsing_returned_data(uint8_t ta, uint8_t id)
{
   if(id == marking && ta == 0x0d){
    awlink_shooting_send_tcp();   // 目前为udp
	printf("be hit\n");
   }
}
void arduino_update(float dt)
{

    int num;
	uint8_t i;
	uint8_t rx_buf[100];

#ifdef X_1
	rc_awlink_set_rc_h();                      //主动刷新心跳，无断线自动降落功能
#endif  

	num = read(arduino_fd,rx_buf,100);
	if(num > 0){
		for(i = 0;i < num;i++){
			printf("rx_buf[%d] = %d %x \n",i,rx_buf[i],rx_buf[i]);
		}
       Parsing_returned_data(rx_buf[0],rx_buf[3]);
#if 0
	   if(arduino_control(rx_buf[0]) == true ){     //先判断是不是自定义私有协议
         return ;
	   }
#endif
	   if(rx_buf[0] == AWLINK_MAGIC){
		 crc_check(rx_buf,num);
		 }else{
        // DEBUG(DEBUG_ID,"uart send error\n");
		 crc_check(&rx_buf[8],num-8);          //串口数据会多发8位，当接受的第一位数据不是0xFA时，需要后移8位才是需要的数据
	    }
	}

}

void* arduino_thread(void *arg)
{
  int num;
  uint8_t i;
  uint8_t rx_buf[100];
  while(1){

   num = read(arduino_fd,rx_buf,100);
   if(num > 0){
	for(i = 0;i < num;i++){
		printf("rx_buf[%d] = %d %x \n",i,rx_buf[i],rx_buf[i]);
		}
	}else{
     // printf("no data\n");
	}

  }

  usleep(2000);

 return ((void * )0);

}



