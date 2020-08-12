#include <fcntl.h>
#include <unistd.h>

#include "app_debug.h"
#include "param.h"

#define DEBUG_ID DEBUG_ID_HAL

#define LIS3MDL_PATH "/dev/lis3mdl"

typedef struct{
	int16_t x;
	int16_t y;
	int16_t z;
}lis3mdl_report_s;

int lis3mdl_fd = -1;

#define LIS3MDL_SCALE_TO_MGAUSS 0.29f
float lis3mdl_mag_scale[3] = {0};


bool lis3mdl_open()
{
	lis3mdl_fd = open(LIS3MDL_PATH, O_RDONLY);
	if(lis3mdl_fd < 0){
		DEBUG(DEBUG_ID,"open lis3mdl failed (%s)", LIS3MDL_PATH);
		return false;
	}else{
		INFO(DEBUG_ID,"use mmc5883 mag");
	}
	
	return true;
}

bool lis3mdl_read(float dt,float mag[3])
{
	lis3mdl_report_s report;
	
	if(read(lis3mdl_fd, &report, sizeof(report)) > 0){
		mag[0] = report.x *  LIS3MDL_SCALE_TO_MGAUSS;
		mag[1] = report.y *  LIS3MDL_SCALE_TO_MGAUSS;
		mag[2] = report.z *  LIS3MDL_SCALE_TO_MGAUSS;

		// printf("%3.3f %3.3f %3.3f\r\n",dat->x,dat->y,dat->z);
		return true;
	}else{
		return false;
	}
}

