#include <fcntl.h>
#include <unistd.h>

#include "app_debug.h"
#include "param.h"

#define DEBUG_ID DEBUG_ID_HAL

#define IST8310_PATH "/dev/ist8310"

#define IST8310_UPDATE_RATE 200// Hz

typedef struct{
	int16_t x;
	int16_t y;
	int16_t z;
	int16_t temp;
}ist8310_report_s;

int ist8310_fd = -1;



bool ist8310_open(void)
{
	ist8310_fd = open(IST8310_PATH, O_RDONLY);
	if(ist8310_fd < 0){
		DEBUG(DEBUG_ID,"open ist8310 failed (%s)", IST8310_PATH);
		return false;
	}else{
		INFO(DEBUG_ID,"use ist8310 mag");
	}
	
	return true;
}

bool ist8310_read(float dt,float mag[3])
{
	ist8310_report_s report;
	static float ist8310_timer = 0;

	ist8310_timer += dt;
	
	if(ist8310_timer > (1.0f / IST8310_UPDATE_RATE)){
		ist8310_timer = 0.0f;
	}else{
		return false;
	}
	
	if(read(ist8310_fd, &report, sizeof(report)) > 0){
		mag[0] = -(float)report.y * 3.0f;
		mag[1] = -(float)report.z * 3.0f ;
		mag[2] = -(float)report.x * 3.0f ;
		return true;
	}else{
		return false;
	}
}

