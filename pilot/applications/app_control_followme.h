#ifndef _APP_CONTROL_FOLLOWME_H_
#define _APP_CONTROL_FOLLOWME_H_

#include <stdbool.h>

typedef struct{
	double lat;
	double lon;
	int16_t yaw;
	float alt;
	float accuracy;
	float vel;
	uint32_t count;
}control_followme_s;

bool control_followme_check();
bool control_followme_mode_init(float param1,float param2);
void control_followme_param_init();
void control_followme_update(float dt,rc_s * rc);
void control_followme_set_target(double lat,double lon,float alt,float accuracy,int16_t yaw,float vel);
void control_followme_get_target(double * lat,double * lon,float * x,float * y,float * accuracy,float * yaw,float * vel,uint32_t * count);
void control_followme_get_diff_xy(float * x,float * y);
void control_followme_get_debug(float * mode,float * distance,float * tx,float * ty,float * tr,float * tp,float * tyaw);
void control_followme_set_target_accuracy(float accuracy,float distance);

#endif

