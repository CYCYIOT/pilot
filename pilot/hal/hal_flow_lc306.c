#include <fcntl.h>
#include <unistd.h>
#include <termios.h>

#include "app_debug.h"

#include "param.h"
#include "hal_linux.h"
#include "lib_math.h"

#define DEBUG_ID DEBUG_ID_HAL

//#define FLOW_LC306_UART "/dev/ttyS2"

static int flow_lc306_fd = -1;

#define SENSOR_IIC_ADDR 0xDC

static uint8_t tab_focus[4] = {0x96,0x26,0xbc,0x50};
static uint8_t Sensor_cfg[] = {
//地址, 数据
0x12, 0x80, 
0x11, 0x30, 
0x1b, 0x06, 
0x6b, 0x43, 
0x12, 0x20, 
0x3a, 0x00, 
0x15, 0x02, 
0x62, 0x81, 
0x08, 0xa0, 
0x06, 0x68, 
0x2b, 0x20, 
0x92, 0x25, 
0x27, 0x97, 
0x17, 0x01, 
0x18, 0x79, 
0x19, 0x00, 
0x1a, 0xa0, 
0x03, 0x00, 
0x13, 0x00, 
0x01, 0x13, 
0x02, 0x20, 
0x87, 0x16, 
0x8c, 0x01, 
0x8d, 0xcc, 
0x13, 0x07, 
0x33, 0x10, 
0x34, 0x1d, 
0x35, 0x46, 
0x36, 0x40, 
0x37, 0xa4, 
0x38, 0x7c, 
0x65, 0x46, 
0x66, 0x46, 
0x6e, 0x20, 
0x9b, 0xa4, 
0x9c, 0x7c, 
0xbc, 0x0c, 
0xbd, 0xa4, 
0xbe, 0x7c, 
0x20, 0x09, 
0x09, 0x03, 
0x72, 0x2f, 
0x73, 0x2f, 
0x74, 0xa7, 
0x75, 0x12, 
0x79, 0x8d, 
0x7a, 0x00, 
0x7e, 0xfa, 
0x70, 0x0f, 
0x7c, 0x84, 
0x7d, 0xba, 
0x5b, 0xc2, 
0x76, 0x90, 
0x7b, 0x55, 
0x71, 0x46, 
0x77, 0xdd, 
0x13, 0x0f, 
0x8a, 0x10, 
0x8b, 0x20, 
0x8e, 0x21, 
0x8f, 0x40, 
0x94, 0x41, 
0x95, 0x7e, 
0x96, 0x7f, 
0x97, 0xf3, 
0x13, 0x07, 
0x24, 0x58, 
0x97, 0x48, 
0x25, 0x08, 
0x94, 0xb5, 
0x95, 0xc0, 
0x80, 0xf4, 
0x81, 0xe0, 
0x82, 0x1b, 
0x83, 0x37, 
0x84, 0x39, 
0x85, 0x58, 
0x86, 0xff, 
0x89, 0x15, 
0x8a, 0xb8, 
0x8b, 0x99, 
0x39, 0x98, 
0x3f, 0x98, 
0x90, 0xa0, 
0x91, 0xe0, 
0x40, 0x20, 
0x41, 0x28, 
0x42, 0x26, 
0x43, 0x25, 
0x44, 0x1f, 
0x45, 0x1a, 
0x46, 0x16, 
0x47, 0x12, 
0x48, 0x0f, 
0x49, 0x0d, 
0x4b, 0x0b, 
0x4c, 0x0a, 
0x4e, 0x08, 
0x4f, 0x06, 
0x50, 0x06, 
0x5a, 0x56, 
0x51, 0x1b, 
0x52, 0x04, 
0x53, 0x4a, 
0x54, 0x26, 
0x57, 0x75, 
0x58, 0x2b, 
0x5a, 0xd6, 
0x51, 0x28, 
0x52, 0x1e, 
0x53, 0x9e, 
0x54, 0x70, 
0x57, 0x50, 
0x58, 0x07, 
0x5c, 0x28, 
0xb0, 0xe0, 
0xb1, 0xc0, 
0xb2, 0xb0, 
0xb3, 0x4f, 
0xb4, 0x63, 
0xb4, 0xe3, 
0xb1, 0xf0, 
0xb2, 0xa0, 
0x55, 0x00, 
0x56, 0x40, 
0x96, 0x50, 
0x9a, 0x30, 
0x6a, 0x81, 
0x23, 0x33, 
0xa0, 0xd0, 
0xa1, 0x31, 
0xa6, 0x04, 
0xa2, 0x0f, 
0xa3, 0x2b, 
0xa4, 0x0f, 
0xa5, 0x2b, 
0xa7, 0x9a, 
0xa8, 0x1c, 
0xa9, 0x11, 
0xaa, 0x16, 
0xab, 0x16, 
0xac, 0x3c, 
0xad, 0xf0, 
0xae, 0x57, 
0xc6, 0xaa, 
0xd2, 0x78, 
0xd0, 0xb4, 
0xd1, 0x00, 
0xc8, 0x10, 
0xc9, 0x12, 
0xd3, 0x09, 
0xd4, 0x2a, 
0xee, 0x4c, 
0x7e, 0xfa, 
0x74, 0xa7, 
0x78, 0x4e, 
0x60, 0xe7, 
0x61, 0xc8, 
0x6d, 0x70, 
0x1e, 0x39, 
0x98, 0x1a
};

typedef enum {
	FLOW_DECODE_START = 0,
	FLOW_DECODE_LEN,
	FLOW_DECODE_DATA,
	FLOW_DECODE_SUM,
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
	uint8_t             buf[10];
}flow_decoder_t;

static flow_decoder_t flow_decoder;
static int16_t flow_uart_raw_vel[2];
static uint8_t flow_qual;
static uint8_t flow_ver;

#ifdef FLOW_LC306_UART
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
	flow_qual = dec->buf[8];
	flow_ver = dec->buf[9];
}

static bool flow_parse_byte(flow_decoder_t *dec,uint8_t byte)
{
	bool ret = false;
	switch(dec->decode_state){
		case FLOW_DECODE_START:
			//printf("FLOW_DECODE_START \r\n");
			if(byte == 0xFE){
				dec->decode_state = FLOW_DECODE_LEN;
			}
			break;
		case FLOW_DECODE_LEN:	
			//printf("FLOW_DECODE_LEN \r\n");
			if(byte == 0x0A){
				dec->decode_state = FLOW_DECODE_DATA;
			}	
			break;
		case FLOW_DECODE_DATA:
			dec->buf[dec->cnt] = byte;
			dec->cnt++;
			dec->sum += byte;
			if(dec->cnt == 10){
				dec->decode_state = FLOW_DECODE_SUM;
			}
			break;
		case FLOW_DECODE_SUM:
			//printf("FLOW_DECODE_SUM \r\n");
			dec->decode_state = FLOW_DECODE_END;
			break;
		case FLOW_DECODE_END:
			//printf("FLOW_DECODE_END \r\n");
			if(byte == 0x55){
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

static void SensorConfig_UartSend(uint8_t dat)
{
   write(flow_lc306_fd,&dat,1);
}

void SensorConfig_Init()
{
	uint16_t i;
	uint16_t len;
	uint8_t recv[3];
	uint8_t read_dat;
	int num;
	int recv_cnt;
				
	len = sizeof(Sensor_cfg);
	
	//0xAA指令
	SensorConfig_UartSend(0xAA);

	//0xAB指令			
	SensorConfig_UartSend(0xAB);		 
	SensorConfig_UartSend(tab_focus[0]);		
	SensorConfig_UartSend(tab_focus[1]);
	SensorConfig_UartSend(tab_focus[2]);
	SensorConfig_UartSend(tab_focus[3]);
	SensorConfig_UartSend(tab_focus[0]^tab_focus[1]^tab_focus[2]^tab_focus[3]); 		 
		 
	//如接收不到模块返回的三个数，可延时10ms从0xAA指令开始重新配置
	recv_cnt = 0;
	while(recv_cnt<3)
	{
		num = read(flow_lc306_fd,&read_dat,1);
		if(num > 0){
			recv[recv_cnt] = read_dat;
			recv_cnt++;	
		}
	}	
			
	if(((recv[0]^recv[1]) == recv[2]) & (recv[1] == 0x00)){
		INFO(DEBUG_ID,"flow lc306 AB Command ok (%02x,%02x,%02x)",recv[0],recv[1],recv[2]);
	}else{
		INFO(DEBUG_ID,"flow lc306 AB Command fail");
	}
			
	//0xBB指令							
	for(i=0; i<len;i+=2 )
	{
		SensorConfig_UartSend(0xBB);		 
		SensorConfig_UartSend(SENSOR_IIC_ADDR); 	
		SensorConfig_UartSend(Sensor_cfg[i]);
		SensorConfig_UartSend(Sensor_cfg[i+1]);
		SensorConfig_UartSend(SENSOR_IIC_ADDR^Sensor_cfg[i]^Sensor_cfg[i+1]);
		 
		//如接收不到模块返回的三个数，可延时1ms重新发送0xBB指令
		recv_cnt = 0;
		while(recv_cnt<3)  
		{
			num = read(flow_lc306_fd,&read_dat,1);
			if(num > 0){
				recv[recv_cnt] = read_dat;
				recv_cnt++;	
			}						
		}	
				
		if(((recv[0]^recv[1]) == recv[2]) & (recv[1] == 0x00)){
			//INFO(DEBUG_ID,"flow lc306 BB Command ok (%02x,%02x,%02x)",recv[0],recv[1],recv[2]);
		}else{
			INFO(DEBUG_ID,"flow lc306 BB Command fail");						 
		}
	}
	 
	SensorConfig_UartSend(0xDD);	
	
	INFO(DEBUG_ID,"flow lc306 Configuration success");	
}

bool flow_lc306_open()
{
#ifdef FLOW_LC306_UART
	flow_lc306_fd = open(FLOW_LC306_UART, O_RDWR | O_NONBLOCK | O_NDELAY | O_NOCTTY);
	if(flow_lc306_fd < 0){
		DEBUG(DEBUG_ID,"open flow lc306 failed (%s)", FLOW_LC306_UART);
		return false;
	}else{
		hal_linux_uart_cfg(flow_lc306_fd,B19200);
		flow_decoder_init(&flow_decoder);
		SensorConfig_Init();
		INFO(DEBUG_ID,"use flow lc306");
	}
	return true;
#else
	return false;
#endif
}

bool flow_lc306_read(float dt,float vel[2],float *quality,uint16_t * ver)
{
	bool ret = false;
	int num;
	uint8_t i;
	uint8_t rx_buf[15];	
	num = read(flow_lc306_fd,rx_buf,15);
	if(num > 0){
		for(i = 0;i < num;i++){
			//printf("DAT:%02x (%d,%d) \r\n",rx_buf[i],i,num);
			if(flow_parse_byte(&flow_decoder,rx_buf[i])){
				calc_flow_info(&flow_decoder);
				vel[0] = flow_uart_raw_vel[0] / 135.1f;
				vel[1] = flow_uart_raw_vel[1] / 135.1f;
				*quality = flow_qual;
				*ver = flow_ver;
				ret = true;
			}
		}
	}
	return ret;
}

