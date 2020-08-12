#ifndef _APP_SYSTEM_H_
#define _APP_SYSTEM_H_

#include "lib_math.h"

typedef enum{
	NORMAL = 0,
	NEED_CAL,
	CALIBRATING,
	CALIBRATED,
	
}sensor_status_e;


typedef struct{
	sensor_status_e status;
}sensor_status_s;

typedef struct{
	bool armed;
	uint8_t status;
	bool att_limit;
	bool imu_temp_limit;
	bool batt_low;
}system_status_s;

#define SYSTEM_STATUS_OK	0
#define SYSTEM_STATUS_INIT	1
#define SYSTEM_STATUS_ERR	2

void system_init(void);
void system_param_init(void);
void system_update(float dt);
bool system_get_armed(void);
bool system_set_armed(bool armed);
void system_get_armpos(float armpos[2]);
uint8_t system_get_status();
float system_get_awlink_heart_rate();
void system_set_awlink_heart_count();
bool system_get_awlink_online();
bool system_get_run();
void system_set_run(bool run);

float system_get_cpu_free();
float system_get_mem_free();

bool system_get_batt_low();
bool system_get_imu_temp_limit();
bool system_get_att_limit();

#endif
