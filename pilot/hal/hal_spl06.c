#include <fcntl.h>

#include "app_debug.h"

#include "lib_math.h"
#include "param.h"

#define DEBUG_ID DEBUG_ID_HAL

#define SPL06_PATH "/dev/spl06"

typedef struct
{
    int16_t c0;
    int16_t c1;
    int32_t c00;
    int32_t c10;
    int16_t c01;
    int16_t c11;
    int16_t c20;
    int16_t c21;
    int16_t c30;       
    uint8_t id;
    int32_t kP;    
    int32_t kT;
} spl06_calib_data;

struct spl06_report_s{
	spl06_calib_data calib;
	int32_t rawPre;
	int32_t rawTemp;
};

#define SPL06_UPDATE_RATE	0.040f

static int spl06_fd = -1;

void spl06_convert(struct spl06_report_s *rp,float * pressure,float *temp)
{
    float fTCompensate;
	float fTsc, fPsc;
	float qua2, qua3;
	float fPCompensate;

	fTsc = rp->rawTemp / (float)rp->calib.kT;
	fPsc = rp->rawPre / (float)rp->calib.kP;

	fTCompensate = rp->calib.c0 * 0.5 + rp->calib.c1 * fTsc;

	qua2 = rp->calib.c10 + fPsc * (rp->calib.c20 + fPsc* rp->calib.c30);
	qua3 = fTsc * fPsc * (rp->calib.c11 + fPsc * rp->calib.c21);
	fPCompensate = rp->calib.c00 + fPsc * qua2 + fTsc * rp->calib.c01 + qua3;

	*pressure = fPCompensate / 100.0f;
	*temp = fTCompensate;
}

float spl06_altitude_convert(float Press, float Ref_P)
{
	return 44330 * (1 - powf(((float)Press / (float)Ref_P),(1/5.255)));
}

bool spl06_open()
{
	spl06_fd = open(SPL06_PATH,O_RDONLY);
	if(spl06_fd < 0){
		DEBUG(DEBUG_ID,"open spl06 failed (%s)", SPL06_PATH);
		return false;
	}else{
		INFO(DEBUG_ID,"use spl06 baro");
	}

	return true;
}
static float pressure_ref = 0.0f;
float spl06_timer = 0.0f;
float spl06_init_timer = 0.0f;

bool spl06_read(float dt,float * pres,float * alt,float * temp)
{
	struct spl06_report_s report_spl06; 
	float pressure_tmp;
	float temp_tmp;
	float alt_tmp;
	int ret = 0;

	spl06_timer += dt;
	spl06_init_timer += dt;

	if(spl06_timer > SPL06_UPDATE_RATE){
		spl06_timer = 0.0f;
		ret = read(spl06_fd,&report_spl06,sizeof(report_spl06));
		if(ret > 0){
				spl06_convert(&report_spl06,&pressure_tmp,&temp_tmp);
				//printf("%3.3f %3.3f %d %d\r\n",pressure_tmp,temp_tmp,report_spl06.rawPre,report_spl06.rawTemp);
				if(spl06_init_timer > 1.0f && pressure_ref == 0.0f){
					//printf("baro init :%f\n",pressure_tmp);
					//printf("C0:%d\r\n",report_spl06.calib.c0);
					//printf("C1:%d\r\n",report_spl06.calib.c1);
					//printf("C00:%d\r\n",report_spl06.calib.c00);
					//printf("C10:%d\r\n",report_spl06.calib.c10);
					//printf("C01:%d\r\n",report_spl06.calib.c01);
					//printf("C11:%d\r\n",report_spl06.calib.c11);
					//printf("C20:%d\r\n",report_spl06.calib.c20);
					//printf("C21:%d\r\n",report_spl06.calib.c21);
					//printf("C30:%d\r\n",report_spl06.calib.c30);
					//printf("KP:%d\r\n",report_spl06.calib.kP);
					//printf("KT:%d\r\n",report_spl06.calib.kT);
					pressure_ref = pressure_tmp;
				}

				if(pressure_ref != 0){
					alt_tmp = spl06_altitude_convert(pressure_tmp,pressure_ref);
					//printf("%3.3f %3.3f %3.3f\r\n",alt_tmp,pressure_tmp,temp_tmp);
					*alt = alt_tmp;
					*pres = pressure_tmp;
					*temp = temp_tmp;

					return true;
				}
				return false;
		}else{
			return false;
		}
	}
	return false;
}

