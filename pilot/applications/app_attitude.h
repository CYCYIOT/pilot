#ifndef _APP_ATTITUDE_H_
#define _APP_ATTITUDE_H_

#include <stdbool.h>

typedef struct{
	float att[3];
	float rate[3];
	float rate_f[3];
	float acc[3];
	float r[3][3];
	bool valid;
}att_s;

void attitude_init(void);
void attitude_reset();
void attitude_param_init(void);
void attitude_update(float dt);

void attitude_get(att_s *dat);
float attitude_get_att_roll();
float attitude_get_att_pitch();
float attitude_get_att_yaw();
float attitude_get_rate_roll();
float attitude_get_rate_pitch();
float attitude_get_rate_yaw();
float attitude_get_rate_roll_f();
float attitude_get_rate_pitch_f();
float attitude_get_rate_yaw_f();
bool attitude_get_valid(void);
void attitude_get_gyro_bias(float bias[3]);

#endif

