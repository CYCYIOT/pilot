#ifndef _APP_RC_H_
#define _APP_RC_H_

#include <stdbool.h>
#include <stdint.h>

#define RC_MODE_HIGH	0
#define RC_MODE_MEDIUM	1
#define RC_MODE_LOW		2

typedef struct{
	float roll;
	float pitch;
	float yaw;
	float thr;
	float roll_raw;
	float pitch_raw;
	float yaw_raw;
	float thr_raw;
	uint8_t mode;
}rc_s;

void rc_init(void);
void rc_param_init();
void rc_update(float dt);
bool rc_get_update();

void rc_get(rc_s * rc_tmp);
void rc_remote_get(float rc[5]);
void rc_awlink_get(float rc[4]);
void rc_awlink_set_rc(float roll,float pitch,float yaw,float thr);
void rc_awlink_set_rc_h();

void rc_set_headfree(bool enable);
void rc_set_headfree_yaw(float yaw);
bool rc_get_headfree();

void rc_set_mode(uint8_t mode);
uint8_t rc_get_mode();

#endif
