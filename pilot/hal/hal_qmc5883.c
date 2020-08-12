#include <fcntl.h>
#include <unistd.h>

#include "app_debug.h"
#include "param.h"

#define DEBUG_ID DEBUG_ID_HAL

#define QMC5883_PATH "/dev/qmc5883"

#define QMC5883_UPDATE_RATE 100// Hz

typedef struct{
	int16_t x;
	int16_t y;
	int16_t z;
}qmc5883_report_s;

int qmc5883_fd = -1;

float qmc5883_mag_scale[3] = {0};

bool qmc5883_open()
{
	qmc5883_fd = open(QMC5883_PATH, O_RDONLY);
	if(qmc5883_fd < 0){
		DEBUG(DEBUG_ID,"open qmc5883 failed (%s)", QMC5883_PATH);
		return false;
	}else{
		INFO(DEBUG_ID,"use qmc5883 mag");
	}
	
	return true;
}

bool qmc5883_read(float dt,float mag[3])
{
	qmc5883_report_s report;
	static float qmc5883_timer = 0;

	qmc5883_timer += dt;
	if(qmc5883_timer > (1.0f / QMC5883_UPDATE_RATE)){
		qmc5883_timer = 0.0f;
	}else{
		return false;
	}
	
	if(read(qmc5883_fd, &report, sizeof(report)) > 0){
		mag[0] = (float)report.x / 12.0f;
		mag[1] = (float)report.y / 12.0f;
		mag[2] = (float)report.z / 12.0f;

		return true;
	}else{
		return false;
	}
}

