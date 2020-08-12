#include <fcntl.h>
#include <unistd.h>
#include <termios.h>

#include "app_debug.h"
#include "param.h"
#include "hal_linux.h"
#include "hal_tof_vl53l1x.h"
#define DEBUG_ID DEBUG_ID_HAL

#define VL53L1X_UART_PATH "/dev/ttyS2"

int vl53l1x_fd = -1;

#define VL53L1X_STATUS_VALID				0
#define VL53L1X_STATUS_SIGMA_FAIL			1
#define VL53L1X_STATUS_SIGNAL_FAIL			2
#define VL53L1X_STATUS_OUTOFBOUNDS_FAIL		4
#define VL53L1X_STATUS_HARDWARE_FAIL		5
#define VL53L1X_STATUS_WRAP_TARGET_FAIL		7
#define VL53L1X_STATUS_PROCESSING_FAIL		8
#define VL53L1X_STATUS_RANGE_INVALID		14

#define VL53L1X_DECODE_MAGIC		0x5A

#define VL53L1X_DECODE_STEP_MAGIC0	0
#define VL53L1X_DECODE_STEP_MAGIC1	1
#define VL53L1X_DECODE_STEP_TYPE	2
#define VL53L1X_DECODE_STEP_LEN		3
#define VL53L1X_DECODE_STEP_DATA1	4
#define VL53L1X_DECODE_STEP_DATA2	5
#define VL53L1X_DECODE_STEP_DATA3	6
#define VL53L1X_DECODE_STEP_CHECK	7
#define VL53L1X_DECODE_STEP_OK		8
#define VL53L1X_DECODE_STEP_FAIL	9

uint8_t vl53l1x_decode_data1 = 0;
uint8_t vl53l1x_decode_data2 = 0;
uint8_t vl53l1x_decode_data3 = 0;
uint8_t vl53l1x_decode_step = VL53L1X_DECODE_STEP_MAGIC0;
uint16_t vl53l1x_decode_check = 0;

uint8_t vl53l1x_data_status;
uint8_t vl53l1x_data_time;
uint8_t vl53l1x_data_mode;
uint16_t vl53l1x_data_distance;

bool vl53l1x_open()
{
#ifdef VL53L1X_UART_PATH
	vl53l1x_fd = open(VL53L1X_UART_PATH, O_RDWR | O_NONBLOCK | O_NDELAY | O_NOCTTY);
	if(vl53l1x_fd < 0){
		DEBUG(DEBUG_ID,"open vl53l1x_fd failed (%s)", VL53L1X_UART_PATH);
		return false;
	}else{
		hal_linux_uart_cfg(vl53l1x_fd,B115200);
		INFO(DEBUG_ID,"use vl53l1x tof");
		return true;
	}
#endif

	return false;
}

uint8_t vl53l1x_get_status()
{
	return vl53l1x_data_status;
}

void vl53l1x_decode(uint8_t data)
{
	switch(vl53l1x_decode_step){
		case VL53L1X_DECODE_STEP_MAGIC0:
			if(data == VL53L1X_DECODE_MAGIC){
				vl53l1x_decode_check = data;
				vl53l1x_decode_step = VL53L1X_DECODE_STEP_MAGIC1;
			}
			break;
		case VL53L1X_DECODE_STEP_MAGIC1:
			if(data == VL53L1X_DECODE_MAGIC){
				vl53l1x_decode_check += data;
				vl53l1x_decode_step = VL53L1X_DECODE_STEP_TYPE;
			}else{
				vl53l1x_decode_step = VL53L1X_DECODE_STEP_FAIL;
			}
			break;
		case VL53L1X_DECODE_STEP_TYPE:
			if(data == 0x15){
				vl53l1x_decode_check += data;
				vl53l1x_decode_step = VL53L1X_DECODE_STEP_LEN;
			}else{
				vl53l1x_decode_step = VL53L1X_DECODE_STEP_FAIL;
			}			
			break;
		case VL53L1X_DECODE_STEP_LEN:
			if(data == 0x3){
				vl53l1x_decode_check += data;
				vl53l1x_decode_step = VL53L1X_DECODE_STEP_DATA1;
			}else{
				vl53l1x_decode_step = VL53L1X_DECODE_STEP_FAIL;
			}
			break;
		case VL53L1X_DECODE_STEP_DATA1:
			vl53l1x_decode_data1 = data;
			vl53l1x_decode_check += data;
			vl53l1x_decode_step = VL53L1X_DECODE_STEP_DATA2;
			break;
		case VL53L1X_DECODE_STEP_DATA2:
			vl53l1x_decode_data2 = data;
			vl53l1x_decode_check += data;
			vl53l1x_decode_step = VL53L1X_DECODE_STEP_DATA3;
			break;
		case VL53L1X_DECODE_STEP_DATA3:
			vl53l1x_decode_data3 = data;
			vl53l1x_decode_check += data;
			vl53l1x_decode_step = VL53L1X_DECODE_STEP_CHECK;
			break;
		case VL53L1X_DECODE_STEP_CHECK:
			if((vl53l1x_decode_check & 0xFF) == data){
				vl53l1x_decode_step = VL53L1X_DECODE_STEP_OK;
			}else{
				vl53l1x_decode_step = VL53L1X_DECODE_STEP_FAIL;
			}
			break;
	}

	if(vl53l1x_decode_step == VL53L1X_DECODE_STEP_FAIL){
		vl53l1x_decode_step = VL53L1X_DECODE_STEP_MAGIC0;
	}
}

bool vl53l1x_read(float dt,float * dist)
{
#if 1
	bool check = false;
	uint8_t data[8];
	int len;

	len = read(vl53l1x_fd,data,8);
	if(len > 0){
		//printf("vl53l1x_read:%d %02x %02x %02x %02x %02x %02x %02x %02x\r\n",len,data[0],data[1],data[2],data[3],data[4],data[5],data[6],data[7]);
		int i = 0;
		for(i = 0 ; i < len ; i++){
			vl53l1x_decode(data[i]);

			if(vl53l1x_decode_step == VL53L1X_DECODE_STEP_OK){
				vl53l1x_decode_step = VL53L1X_DECODE_STEP_MAGIC0;
				vl53l1x_data_status = (vl53l1x_decode_data3 >> 4) & 0xF;
				vl53l1x_data_time = (vl53l1x_decode_data3 >> 2) & 0x3;
				vl53l1x_data_mode = vl53l1x_decode_data3 & 0x3;
				vl53l1x_data_distance = (vl53l1x_decode_data1 << 8) | vl53l1x_decode_data2;

				if(vl53l1x_data_status == VL53L1X_STATUS_VALID){
					*dist = (float)vl53l1x_data_distance / 1000;
					check = true;
				}

				//printf("dist:%d status:%d time:%d mode:%d \r\n",vl53l1x_data_distance,vl53l1x_data_status,vl53l1x_data_time,vl53l1x_data_mode);
			}
		}
	}
	

  return check;
#endif
}

