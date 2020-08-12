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

//#define BK2461_UART_PATH	"/dev/ttyS2"

int bk2461_fd = -1;

#define	 BK2461_HEAD1	0x55
#define  BK2461_HEAD2	0xAA
#define  BK2461_END		0xC0

#define  BK2461_NEW_HEAD 0xA5
#define  BK2461_NEW_END  0xC0

#define  BK2461_PACKAGE_MAX 64

#define  BK2461_TYPE_PWM 0x01
#define  BK2461_TYPE_RF  0x02

#define  BK2461_STATUS_IDLE1  0x11
#define  BK2461_STATUS_IDLE2  0x22
#define  BK2461_STATUS_START  0x33
#define  BK2461_STATUS_CHECK  0x44
#define  BK2461_STATUS_OK     0x55
#define  BK2461_STATUS_DROP   0x66

typedef struct{
	unsigned char head1;
	unsigned char head2;
	unsigned char type;
	unsigned char length;
	unsigned char *payload;
    unsigned char end;
}bk2461_s;

typedef struct{
	unsigned char status; 
	unsigned char length;
}bk2461_decoder_s;

unsigned char bk2461_buf[BK2461_PACKAGE_MAX];
unsigned char bk2461_write_ptr = 0;

bk2461_decoder_s bk2461_decoder;

float bk2461_roll;
float bk2461_pitch;
float bk2461_yaw;
float bk2461_thr;
float bk2461_mode;

void bk2461_init(bk2461_decoder_s *dec)
{
	dec->status = BK2461_STATUS_IDLE1;
}

char bk2461_purse_char(bk2461_decoder_s *dec,unsigned char *ch)
{
    char ret = 0;
	if(bk2461_write_ptr >= BK2461_PACKAGE_MAX){
		dec->status = BK2461_STATUS_IDLE1;
		bk2461_write_ptr = 0;
		return 0;
	}
	switch(dec->status){
	  case BK2461_STATUS_IDLE1:
	      if(*ch == BK2461_HEAD1){
		       dec->status =  BK2461_STATUS_IDLE2;
		  }
		  break;
	  case BK2461_STATUS_IDLE2:
	      if(*ch == BK2461_HEAD2){
		       dec->status =  BK2461_STATUS_START;
			   bk2461_write_ptr = 0;
		  }
		  break;
	  case BK2461_STATUS_START:
	      if(*ch == BK2461_END){
		      if(bk2461_buf[1] == (bk2461_write_ptr - 2)){
			      dec->status =  BK2461_STATUS_OK;
				  dec->length = bk2461_buf[1];
				  ret = 1;
			  }else if(bk2461_buf[1] > (bk2461_write_ptr - 2)){
			      dec->status = BK2461_STATUS_CHECK;
				  bk2461_buf[bk2461_write_ptr++] = *ch;
			  }else if(bk2461_buf[1] < (bk2461_write_ptr - 2)){
			      dec->status = BK2461_STATUS_DROP;
			  }
		  }else{
			  bk2461_buf[bk2461_write_ptr%BK2461_PACKAGE_MAX] = *ch;
			  bk2461_write_ptr++;
			  
		  }
		  break;
	  case BK2461_STATUS_CHECK:
	  	  if(*ch == BK2461_HEAD1){
		      dec->status =  BK2461_STATUS_IDLE2;
		  }else if(*ch == BK2461_END){
		  	  if(bk2461_buf[1] == (bk2461_write_ptr - 2)){
			      dec->status =  BK2461_STATUS_OK;
				  ret = 1;
			  }else{
		         bk2461_buf[bk2461_write_ptr%BK2461_PACKAGE_MAX] = *ch;
				 bk2461_write_ptr++;
			     dec->status =  BK2461_STATUS_START;
			  }
		  }else{
		  	  bk2461_buf[bk2461_write_ptr%BK2461_PACKAGE_MAX] = *ch;
			  bk2461_write_ptr++;
			  dec->status =  BK2461_STATUS_START;
		  }
	      break;
	   default:
	       dec->status =  BK2461_STATUS_IDLE2;
		   break;
	};
	if(dec->status == BK2461_STATUS_DROP)
	    dec->status = BK2461_STATUS_IDLE1;
    if(dec->status == BK2461_STATUS_OK)
	    dec->status = BK2461_STATUS_IDLE1;
	return ret;
}

void bk2461_pack(unsigned char *out,unsigned char *in,unsigned char size)
{
	unsigned char i;

	out[0] = BK2461_NEW_HEAD;

	for(i = 1;i < size+1;i++){
		out[i] = in[i-1];
	}

	out[size+1] = BK2461_NEW_END;
}

void bk2461_convert(unsigned char *buf)
{
	short tmp;
	
	tmp = buf[2] << 8 | buf[3];
	bk2461_pitch = ((float)tmp / 2048.0f) - 1.0f;
	bk2461_pitch = -bk2461_pitch;

	tmp = buf[4] << 8 | buf[5];
	bk2461_thr = ((float)tmp / 2048.0f) - 1.0f;

	tmp = buf[6] << 8 | buf[7];
	bk2461_yaw  = ((float)tmp / 2048.0f) - 1.0f;

	tmp = buf[8] << 8 | buf[9];
	bk2461_roll  = ((float)tmp / 2048.0f) - 1.0f;

	bk2461_mode = (float)buf[10];	
}

bool bk2461_open()
{
#ifdef BK2461_UART_PATH
	bk2461_fd = open(BK2461_UART_PATH, O_RDWR | O_NONBLOCK | O_NDELAY | O_NOCTTY);
#endif
	if(bk2461_fd < 0){
		printf("bk2461 init failed\r\n");
		return false;
	}else{
		hal_linux_uart_cfg(bk2461_fd,B38400);
		bk2461_init(&bk2461_decoder);
	}
	
	return true;
}

bool bk2461_read(float * rc)
{
	bool ret = false;
	int num;
	uint8_t i;
	uint8_t rx_buf[15];

	num = read(bk2461_fd,rx_buf,15);
	if(num > 0){
		for(i = 0;i < num;i++){
			if(bk2461_purse_char(&bk2461_decoder,&rx_buf[i])){
				bk2461_convert(bk2461_buf);
				rc[0] = bk2461_roll;
				rc[1] = bk2461_pitch;
				rc[2] = bk2461_yaw;
				rc[3] = bk2461_thr;
				rc[4] = bk2461_mode;
				ret = true;
			}
		}
	}

	return ret;
}

bool bk2461_write(float * motor)
{
	int num;
	unsigned char out[10],in[8];
	unsigned char i;
	unsigned short tmp;

	for(i = 0;i < 8;i+=2){
		tmp = (unsigned short)((motor[i/2] + 0.0001f) * 1000);
		in[i]   = tmp / 128;
		in[i+1] = tmp % 128;
	}

    bk2461_pack(out,in,8);
	num = write(bk2461_fd,out,10);
	if(num == 10){
		return true;
	}else{
		return false;
	}
}

