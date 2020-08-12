#include <fcntl.h>
#include <unistd.h>

#include "app_debug.h"
#include "param.h"

#define DEBUG_ID DEBUG_ID_HAL

#define MMC5883_PATH "/dev/mmc5883"

#define MMC5883_UPDATE_RATE 100// Hz
#define OPT_CONVERT(REG)		 ((float)((REG) >=32 ? (32 - (REG)) : (REG)) * 0.006)


typedef struct{
	int16_t x;
	int16_t y;
	int16_t z;
	uint8_t  reg[2];
}mmc5883_report_s;

int mmc5883_fd = -1;

float mmc5883_mag_scale[3] = {0};
float fOtpMatrix[3] = {1.0,1.0,1.35};


bool mmc5883_open(void)
{
	mmc5883_fd = open(MMC5883_PATH, O_RDONLY);
	if(mmc5883_fd < 0){
		DEBUG(DEBUG_ID,"open mmc5883 failed (%s)", MMC5883_PATH);
		return false;
	}else{
		INFO(DEBUG_ID,"use mmc5883 mag");
	}
	
	return true;
}

bool mmc5883_read(float dt,float mag[3])
{
	mmc5883_report_s report;
	static float mmc5883_timer = 0;

	mmc5883_timer += dt;
	if(mmc5883_timer > (1.0f / MMC5883_UPDATE_RATE)){
		mmc5883_timer = 0.0f;
	}else{
		return false;
	}
	
	if(read(mmc5883_fd, &report, sizeof(report)) > 0){
		mag[1] = (float)report.x / 4.096f;
		mag[0] = (float)report.y / 4.096f;
		mag[2] = (float)report.z / 4.096f;

		return true;
	}else{
		return false;
	}
}

