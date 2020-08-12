#include <fcntl.h>
#include <unistd.h>
#include <termios.h>

#include "app_debug.h"

#include "param.h"
#include "hal_linux.h"
#include "lib_math.h"

#define DEBUG_ID DEBUG_ID_HAL

//#define FLOW_PMW3901_UART "/dev/ttyS0"

static int flow_pmw3901_fd = -1;

typedef enum {
	FLOW_DECODE_START = 0,
	FLOW_DECODE_LEN,
	FLOW_DECODE_DATA,
	FLOW_DECODE_SUM,
	FLOW_DECODE_SQUAL,
	FLOW_DECODE_END,
} flow_decode_state_t;

typedef struct{
	int16_t flow_x;
	int16_t flow_y;
	uint8_t qual;
}flow_uart_info_s;

typedef struct {
	flow_decode_state_t decode_state;
	uint8_t             sum;
	uint8_t             cnt;	
	flow_uart_info_s    flow;
	uint8_t             buf[5];
}flow_decoder_t;

static flow_decoder_t flow_decoder;
static int16_t flow_uart_raw_vel[2];
static uint8_t flow_qual;

#ifdef FLOW_PMW3901_UART
static void flow_decoder_init(flow_decoder_t *dec)
{
	dec->decode_state = FLOW_DECODE_START;
	dec->cnt = 0;
	dec->sum = 0;
}
#endif

static void calc_flow_info(flow_decoder_t *dec)
{
	flow_uart_raw_vel[0] = -(int16_t)((dec->buf[1]<<8)|(dec->buf[0]));
	flow_uart_raw_vel[1] = -(int16_t)((dec->buf[3]<<8)|(dec->buf[2]));
	flow_qual = dec->buf[4];
}

static bool flow_parse_byte(flow_decoder_t *dec,uint8_t byte)
{
	bool ret = false;
	switch(dec->decode_state){
		case FLOW_DECODE_START:
			if(byte == 0xFE){
				dec->decode_state = FLOW_DECODE_LEN;
			}
			break;
		case FLOW_DECODE_LEN:	
			if(byte == 0x04){
				dec->decode_state = FLOW_DECODE_DATA;
			}	
			break;
		case FLOW_DECODE_DATA:
			dec->buf[dec->cnt] = byte;
			dec->cnt++;
			dec->sum += byte;
			if(dec->cnt == 4){
				dec->decode_state = FLOW_DECODE_SUM;
			}
			break;
		case FLOW_DECODE_SUM:
			if(byte == dec->sum){
				dec->decode_state = FLOW_DECODE_SQUAL;
			}else{
				dec->decode_state = FLOW_DECODE_START;
				dec->cnt = 0;
				dec->sum = 0;
			}
			break;
		case FLOW_DECODE_SQUAL:
			dec->buf[dec->cnt] = byte;
			dec->decode_state = FLOW_DECODE_END;
			break;
		case FLOW_DECODE_END:
			if(byte == 0xAA){
				dec->decode_state = FLOW_DECODE_START;
				ret = true;
			}
			dec->decode_state = FLOW_DECODE_START;
			dec->cnt = 0;
			dec->sum = 0;
			break;
	}
	return ret;
}

bool flow_pmw3901_open()
{
#ifdef FLOW_PMW3901_UART
	flow_pmw3901_fd = open(FLOW_PMW3901_UART, O_RDWR | O_NONBLOCK | O_NDELAY | O_NOCTTY);
	if(flow_pmw3901_fd < 0){
		DEBUG(DEBUG_ID,"open flow pmw3901 failed (%s)", FLOW_PMW3901_UART);
		return false;
	}else{
		hal_linux_uart_cfg(flow_pmw3901_fd,B19200);
		flow_decoder_init(&flow_decoder);
		INFO(DEBUG_ID,"use flow pmw3901");
	}
	return true;
#else
	return false;
#endif
}

bool flow_pmw3901_read(float dt,float vel[2],float *quality,uint16_t * ver)
{
	bool ret = false;
	int num;
	uint8_t i;
	uint8_t rx_buf[10];	
	num = read(flow_pmw3901_fd,rx_buf,10);
	if(num > 0){
		for(i = 0;i < num;i++){
			if(flow_parse_byte(&flow_decoder,rx_buf[i])){
				calc_flow_info(&flow_decoder);
				vel[0] = flow_uart_raw_vel[0] / 5.7f;
				vel[1] = flow_uart_raw_vel[1] / 5.7f;
				*quality = flow_qual;
				ret = true;
			}
		}
	}
	return ret;
}

