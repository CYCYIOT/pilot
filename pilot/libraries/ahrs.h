#ifndef _AHRS_H_
#define _AHRS_H_

#include <stdbool.h>

#define AHRS_MODE_DISARM	0
#define AHRS_MODE_NORMAL	1
#define AHRS_MODE_FLIP		2
#define AHRS_MODE_ACTION	3

typedef struct{
	bool inited;
	float gyro_err_int[3];
	float mag_err_int[3];
	float q[4];
	float gyro[3];
	float acc_raw[3];
	float gyro_raw[3];
	float mag_raw[3];
	bool  acc_updated;
	bool  gyro_updated;
	bool  mag_updated;
}ahrs_estimator;

void ahrs_param_init();
void ahrs_init(ahrs_estimator *est,float *acc,float *mag);
void ahrs_update(float dt,ahrs_estimator *est,bool armed);

void ahrs_apply_acc(ahrs_estimator *est,float acc[3]);
void ahrs_apply_gyro(ahrs_estimator *est,float gyro[3]);
void ahrs_apply_mag(ahrs_estimator *est,float mag[3]);

void ahrs_init_from_euler(ahrs_estimator *est,float att[3]);
void ahrs_qua2dcm(float *q,float d[3][3]);
void ahrs_qua2euler(float *q,float att[3]);
void ahrs_euler2qua(float att[3],float *q);
void ahrs_euler2dcm(float att[3],float d[3][3]);

void ahrs_set_flip_mode(bool enable);

float ahrs_get_rp_p();
float ahrs_get_rp_i();
float ahrs_get_check_time();
uint8_t ahrs_get_mode();

#endif

