#ifndef _APP_MISSION_H_
#define _APP_MISSION_H_

#include "lib_math.h"

#define MISSION_RUN_STATUS_STOP			0
#define MISSION_RUN_STATUS_RUN			1
#define MISSION_RUN_STATUS_PAUSE		2

#define MISSION_TYPE_CONTROL	1

#pragma pack(1)
typedef struct mission_control_s {
	uint8_t mode;
	float param1;
	float param2;
	float param3;
	float param4;
	float param5;	
	float param6;	
	float param7;	
	float param8;	
}mission_control_s;
#pragma pack()

void mission_init(void);
void mission_update(float dt);

void mission_save(void);
void mission_clear(void);

void mission_add_item(uint16_t count,uint8_t type,uint8_t len,uint8_t * data);
int mission_get_item_type(uint16_t count);
bool mission_get_item_data(uint16_t count,uint8_t * data,uint8_t len);
uint16_t mission_get_total(void);

void mission_set_done();
float mission_get_time();
uint16_t mission_get_run_count();
uint8_t mission_get_run_type();
uint8_t mission_get_run_status();
void mission_set_run(uint8_t status);
void mission_clear_time();

#endif

