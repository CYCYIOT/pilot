#ifndef _APP_BARO_H_
#define _APP_BARO_H_

void baro_init(void);
void baro_update(float dt);

bool baro_get_update();
float baro_get_pres();
float baro_get_alt();
float baro_get_alt_raw();
float baro_get_alt_f();
float baro_get_vel();
float baro_get_vel_f();
float baro_get_baro_vel(int i);
float baro_get_baro_alt_bias();
float baro_get_baro_alt_vel_zero();
void baro_set_baro_vel_enable(bool enable);

float baro_get_temp();
uint8_t baro_get_status();
float baro_get_error_time();

#endif
