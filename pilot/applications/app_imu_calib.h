#ifndef _APP_IMU_CALIB_ACC_H_
#define _APP_IMU_CALIB_ACC_H_

#define IMU_CALIB_ACC_STATUS_NULL	0
#define IMU_CALIB_ACC_STATUS_EXIST	1
#define IMU_CALIB_ACC_STATUS_START	2
#define IMU_CALIB_ACC_STATUS_FAILED	3

#define IMU_CALIB_ACC_STEP_LEVEL	0
#define IMU_CALIB_ACC_STEP_BACK		1
#define IMU_CALIB_ACC_STEP_UP		2
#define IMU_CALIB_ACC_STEP_DOWN		3
#define IMU_CALIB_ACC_STEP_RIGHT	4
#define IMU_CALIB_ACC_STEP_LEFT		5
#define IMU_CALIB_ACC_STEP_DONE		6

void imu_calib_acc_confirm_step(void);
void imu_calib_acc_start(void);
uint8_t imu_calib_acc_get_status(void);
uint8_t imu_calib_acc_get_step(void);
uint8_t imu_calib_acc_get_progress();

void imu_calib_acc_init(void);
void imu_calib_acc_param_init();
void imu_calib_acc_update(void);
void imu_calib_acc_set_status_null(void);


#endif
