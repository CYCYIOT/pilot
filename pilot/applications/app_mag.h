#ifndef _APP_MAG_H_
#define _APP_MAG_H_

void mag_init(void);
void mag_update(float dt);

void mag_set_cal_offset(float offset[3]);
void mag_get_val(float mag[3]);
bool mag_get_update();
uint8_t mag_get_status();
uint8_t mag_get_calib_progress();
void mag_set_calib(bool start);
void mag_param_init(void);
bool mag_calib_status_check(float dt);
bool mag_calib_check_result();
bool get_mag_calib_status();
float mag_get_error_time();


#endif
