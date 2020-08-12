#ifndef _HAL_H_
#define _HAL_H_

#include <stdbool.h>
#include <stdint.h>
#include <fcntl.h>
#include <unistd.h>
#include "ubx.h"
#include "hal_stm8s.h"
#include "hal_linux.h"


int hal_init(void);
bool hal_set_motor(float *motor,uint8_t num);
bool hal_get_imu(float dt,float acc[3],float gyro[3],float * temp);
bool hal_get_baro(float dt,float * pres,float * alt,float * temp);
bool hal_get_rangefinder(float dt,float * dist);
bool hal_get_mag(float dt,float mag[3]);
bool hal_get_rc(float dt,float rc[5]);
bool hal_get_batt(float dt,int *batt_status);
bool hal_get_flow(float dt,float vel[2],float *quality,uint16_t * ver);
bool hal_get_flow2(float dt,float vel[2],float *quality,uint16_t * ver);
bool hal_get_flow3(float dt,float vel[2],float *quality,uint16_t * ver);
bool hal_get_aruco(float dt,float vel[2],float *quality,uint16_t * ver);
bool hal_get_gps(float dt,gps_info_s *dat);
void hal_set_led_status(uint8_t status);
bool hal_get_tof();
bool hal_get_cpu_free(float dt,float * cpu_free);
bool hal_get_mem_free(float dt,float * mem_free);
#define    debug_t(str, arg...)  do{\
           time_t timep;\
           struct tm *p;\
           char time_str[40];\
           memset(time_str,'\0',40);\
           time(&timep);\
           p=localtime(&timep);\
           sprintf(time_str,"[%d.%02d.%02d %02d:%02d:%02d]",(1900+p->tm_year),(1+p->tm_mon),p->tm_mday,p->tm_hour,p->tm_min,p->tm_sec);\
           FILE *debug_fp = fopen("/tmp/debug", "a");\
           if (NULL != debug_fp){\
           fprintf(debug_fp, "%s L%d in %s, ",time_str, __LINE__, __FILE__);\
           fprintf(debug_fp, str, ##arg);\
           fflush(debug_fp);\
           fclose(debug_fp);\
           }\
}while(0)



#endif
