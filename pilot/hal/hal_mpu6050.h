#ifndef _HAL_MPU6050_H_
#define _HAL_MPU6050_H_

#include <stdbool.h>

bool mpu6050_open();
bool mpu6050_read(float dt,float acc[3],float gyro[3],float * temp);

#endif

