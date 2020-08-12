#include <fcntl.h>
#include <unistd.h>
#include <sys/msg.h>

#include "app_control.h"
#include "app_debug.h"

#include "hal_linux.h"

#define DEBUG_ID DEBUG_ID_HAL

#if 1
typedef struct {
    long     msg_type;
    uint16_t flow_ver;
    float    flow_x;
    float    flow_y;
    uint8_t  quality;
    uint8_t  image_brightness;
    uint8_t  image_score;
    int8_t   image_grade;
    float time_loop;
    float time_run;
} msg_rec_s;
#else
typedef struct 
{
	long msg_type;
	float flow_x;
	float flow_y;
	unsigned char quality;
	unsigned short flow_ver;
}msg_rec_s;
#endif

int msg_id = -1;
msg_rec_s flow_data;

float flow_time_loop;
float flow_time_run;

float flow_linux_get_loop()
{
	return flow_time_loop;
}

float flow_linux_get_run()
{
	return flow_time_run;
}

bool flow_linux_open()
{
	msg_id = msgget((key_t)1234, 0666 | IPC_CREAT);
	if(msg_id == -1){
		DEBUG(DEBUG_ID,"create msg failed for flow_linux");
		return false;
	}else{
		INFO(DEBUG_ID,"use flow_linux");
		return true;
	}	
}

bool flow_linux_read(float dt,float vel[2],float *quality,uint16_t * ver)
{
	if(msgrcv(msg_id, (void*)&flow_data, sizeof(flow_data),0,IPC_NOWAIT) == -1){
		return false;
	}else{

#if 0
		static uint8_t mode_tmp = CONTROL_MODE_STOP;
		uint8_t mode_test = control_get_mode();
		if(mode_tmp != mode_test){
			mode_tmp = mode_test;
			if(mode_tmp == CONTROL_MODE_TAKEOFF) {
				hal_linux_send_signal(1);
			}else if(mode_tmp == CONTROL_MODE_STOP){
				hal_linux_send_signal(0);
			}
		}
#endif
	
		vel[0] = flow_data.flow_x;
		vel[1] = flow_data.flow_y;	
		*ver = flow_data.flow_ver;
		*quality = (float)flow_data.quality / 100.0f;

		flow_time_loop = flow_data.time_loop;
		flow_time_run = flow_data.time_run;
			
		return true;
	}	
}

