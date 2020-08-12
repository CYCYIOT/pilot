#ifndef _APP_RANGEFINDER_H_
#define _APP_RANGEFINDER_H_

void rangefinder_param_init(void);
void rangefinder_init(void);
void rangefinder_update(float dt);

bool rangefinder_get_update();
float rangefinder_get_vel();
uint8_t rangefinder_get_status();
float rangefinder_get_range();
float rangefinder_get_range_f();
float rangefinder_get_error_time();
float rangefinder_get_rotate();

#endif
