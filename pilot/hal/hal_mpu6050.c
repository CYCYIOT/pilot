#include <fcntl.h>
#include <unistd.h>

#include "app_debug.h"
#include "hal_mpu6050.h"
#include "param.h"

#define DEBUG_ID DEBUG_ID_HAL

#define MPU6050_PATH "/dev/mpu6050"

typedef struct{
	int16_t acc[3];
	int16_t temp;
	int16_t gyro[3];
}mpu6050_report_s;

int mpu6050_fd = -1;


#define ACC_SCALE  0.002387768f //(float)((2 * 8) / 65536.0)*GRAVITY
#define GYRO_SCALE 0.001065185f //(float)((2 * 2000.0) / 65536.0)/57.3

bool mpu6050_open()
{
	mpu6050_fd = open(MPU6050_PATH, O_RDONLY);
	if(mpu6050_fd < 0){
		DEBUG(DEBUG_ID,"open mpu6050 failed (%s)", MPU6050_PATH);
		return false;
	}else{
		INFO(DEBUG_ID,"use mpu6050 imu");
	}

	return true;
}

bool mpu6050_read(float dt,float acc[3],float gyro[3],float * temp)
{
	uint8_t i;
	mpu6050_report_s report;
	
	if(read(mpu6050_fd, &report, sizeof(report)) > 0){	
		for(i = 0;i < 3;i++){
			acc[i]	= (float)report.acc[i]* ACC_SCALE;
			gyro[i] = (float)report.gyro[i]* GYRO_SCALE; 
		}
		*temp = 36.53f + (float)(report.temp) / 340.0f;
		return true;
	}else{
		return false;
	}
}

