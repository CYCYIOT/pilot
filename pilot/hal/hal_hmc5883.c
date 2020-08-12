#include <fcntl.h>

#include "app_debug.h"

#include "lib_math.h"
#include "param.h"

#define DEBUG_ID DEBUG_ID_HAL

#define HMC5883_PATH "/dev/hmc5883"

typedef struct{
	int16_t x;
	int16_t y;
	int16_t z;
}hmc5883_report_s;

int hmc5883_fd = -1;

float hmc5883_gain_multiple = (1.0f / 1090) * 1000;
float hmc5883_mag_scale[3] = {0};
int hmc5883_mag_scale_init = 30;
int hmc5883_mag_scale_good = 0;
#define IS_CALIBRATION_VALUE_VALID(val) (val > 0.7f && val < 1.35f)

bool hmc5883_init()
{
	int ret = 0;
	hmc5883_report_s report;
	float cal[3] = {0};
    uint16_t expected_x = 766;
    uint16_t expected_yz = 713;

	while(hmc5883_mag_scale_init > 0){
		ret = read(hmc5883_fd, &report, sizeof(report));
		if(ret >= 1){
			cal[0] = fabsf(expected_x / (float)report.x);
			cal[1] = fabsf(expected_yz / (float)report.y);
			cal[2] = fabsf(expected_yz / (float)report.z);
			
			if (IS_CALIBRATION_VALUE_VALID(cal[0]) &&
				IS_CALIBRATION_VALUE_VALID(cal[1]) &&
				IS_CALIBRATION_VALUE_VALID(cal[2])) {
				hmc5883_mag_scale_good++;
			
				hmc5883_mag_scale[0] += cal[0];
				hmc5883_mag_scale[1] += cal[1];
				hmc5883_mag_scale[2] += cal[2];
			}
		
			//printf("%3.3f %3.3f %3.3f (%d)\r\n",cal[0],cal[1],cal[2],hmc5883_mag_scale_good);
		}else{
			INFO(DEBUG_ID,"hmc5883 get data failed:%d",ret);
		}
		hmc5883_mag_scale_init--;
		usleep(10 * 1000);
	}

    if (hmc5883_mag_scale_good >= 5) {
        hmc5883_mag_scale[0] = hmc5883_mag_scale[0] / hmc5883_mag_scale_good;
        hmc5883_mag_scale[1] = hmc5883_mag_scale[1] / hmc5883_mag_scale_good;
        hmc5883_mag_scale[2] = hmc5883_mag_scale[2] / hmc5883_mag_scale_good;
    } else {
        hmc5883_mag_scale[0] = 1.0;
        hmc5883_mag_scale[1] = 1.0;
        hmc5883_mag_scale[2] = 1.0;
    }

	INFO(DEBUG_ID,"hmc5883 scale:%3.3f %3.3f %3.3f",hmc5883_mag_scale[0],hmc5883_mag_scale[1],hmc5883_mag_scale[2]);

	return true;
}

bool hmc5883_open()
{
	hmc5883_fd = open(HMC5883_PATH, O_RDONLY);
	if(hmc5883_fd < 0){
		DEBUG(DEBUG_ID,"open hmc5883 failed (%s)", HMC5883_PATH);
		return false;
	}else{
		hmc5883_init();
		INFO(DEBUG_ID,"use hmc5883 mag");
	}
	
	return true;
}

bool hmc5883_read(float dt,float mag[3])
{
	hmc5883_report_s report;
	
	if(read(hmc5883_fd, &report, sizeof(report)) > 0){
		mag[0] = report.x * hmc5883_mag_scale[0] * hmc5883_gain_multiple;
		mag[1] = report.y * hmc5883_mag_scale[1] * hmc5883_gain_multiple;
		mag[2] = report.z * hmc5883_mag_scale[2] * hmc5883_gain_multiple;

		// printf("%3.3f %3.3f %3.3f\r\n",dat->x,dat->y,dat->z);
		return true;
	}else{
		return false;
	}
}

