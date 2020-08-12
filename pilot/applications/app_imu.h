#ifndef _APP_IMU_H_
#define _APP_IMU_H_

#include <stdint.h>
#include <stdbool.h>

void imu_init(void);
void imu_param_init(void);
void imu_update(float dt);

void imu_set_acc_cal(float offset[3],float scale[3]);
void imu_get_acc(float acc[3]);
void imu_get_acc_filted(float acc[3]);
void imu_get_acc_raw(float acc[3]);
void imu_get_gyro(float gyro[3]);
void imu_get_gyro_bias(float gyro_b[3]);
bool imu_get_update();
uint8_t imu_get_acc_status();
uint8_t imu_get_gyro_status();
uint8_t imu_get_acc_calib_status();
void imu_set_acc_calib(bool start);
float imu_get_temp();
float imu_get_error_time();

#endif
