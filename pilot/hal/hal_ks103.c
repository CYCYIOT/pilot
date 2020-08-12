#include <fcntl.h>
#include <unistd.h>

#include "app_debug.h"
#include "param.h"

#define DEBUG_ID DEBUG_ID_HAL

#define KS103_PATH "/dev/ks103"

int ks103_fd = -1;

#define KS103_UPDATE_RATE 10 // Hz

typedef struct{
	uint16_t distance;
}ks103_report_s;

bool ks103_open()
{
	ks103_fd = open(KS103_PATH, O_RDONLY);
	if(ks103_fd < 0){
		DEBUG(DEBUG_ID,"open ks103 failed (%s)",KS103_PATH);
		return false;
	}else{
		INFO(DEBUG_ID,"use ks103 sonar");
	}

	return true;
}

bool ks103_read(float dt,float * dist)
{
	static float ks103_timer = 0;
	ks103_report_s report;

	ks103_timer += dt;
	if(ks103_timer > (1.0f / KS103_UPDATE_RATE)){
		ks103_timer = 0.0f;
	}else{
		return false;
	}

	if(read(ks103_fd, &report, sizeof(report)) > 0){
		*dist = (float)report.distance / 10 / 100;
		//printf("%3.3f\r\n",*dist);		
		return true;
	}else{
		return false;
	}
}

