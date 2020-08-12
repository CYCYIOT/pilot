#ifndef _APP_MAG_CALIB_H_
#define _APP_MAG_CALIB_H_

#include <stdbool.h>

enum calibration_status{
	calibrated = 0,
	calibrating,
	need_calibrate,
	start_calibration,
};


typedef struct{
	enum calibration_status status;
	int                cal_counter;
}calibrator;



void mag_calib_start(void);
bool mag_calib_update(float dt,float mag[3]);
void mag_calib_end(void);
void mag_calib_get_offset(float offset[3]);
void mag_calib_define_param(void);
bool mag_calib_check(float mag[3]);
uint8_t mag_callib_get_progress ();


#endif
