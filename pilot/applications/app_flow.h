#ifndef _APP_FLOW_H_
#define _APP_FLOW_H_
#include <stdbool.h>

void flow_param_init(void);
void flow_init(void);
void flow_update(float dt);

void flow_get_vel(float vel[2]);
void flow_get_vel_raw(float vel[2]);
void flow2_get_vel_raw(float vel[2]);
void flow3_get_vel_raw(float vel[2]);
void flow_get_vel_scale(float vel[2]);
void flow_get_gyro(float gyro[2]);
uint8_t flow_get_status();
bool flow_get_update();
float flow_get_quality();
float flow_get_quality_f();
uint16_t flow_get_version();
float flow_get_error_time();
float flow_get_dtg();
void flow_set_dtg(float min,float max);

#endif
