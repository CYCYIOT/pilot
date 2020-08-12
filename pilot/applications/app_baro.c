#include "app_debug.h"
#include "app_sensors.h"
#include "app_nav.h"
#include "app_pos_control.h"

#include "lib_math.h"
#include "hal.h"
#include "param.h"

#define DEBUG_ID DEBUG_ID_BARO
#define BARO_VEL_BUF_LEN  100

#define BARO_ALT_FILTER	0.70f
#define BARO_VEL_FILTER	0.80f

//#define BARO_VEL_ENABLE  1
#define BARO_VEL_FRONT   -0.01f //-0.03f 
#define BARO_VEL_BACK     0.01f // 0.05f 
#define BARO_VEL_RIGHT   -0.03f //-0.20f
#define BARO_VEL_LEFT    -0.03f //-0.11f

#define    tof(str, arg...)  do{\
           time_t timep;\
           struct tm *p;\
           char time_str[40];\
           memset(time_str,'\0',40);\
           time(&timep);\
           p=localtime(&timep);\
           sprintf(time_str,"[%d.%02d.%02d %02d:%02d:%02d]",(1900+p->tm_year),(1+p->tm_mon),p->tm_mday,p->tm_hour,p->tm_min,p->tm_sec);\
           FILE *debug_fp = fopen("/tmp/tof", "a");\
           if (NULL != debug_fp){\
           fprintf(debug_fp, "%s L%d in %s, ",time_str, __LINE__, __FILE__);\
           fprintf(debug_fp, str, ##arg);\
           fflush(debug_fp);\
           fclose(debug_fp);\
           }\
}while(0)

float baro_pres;
float baro_ground_alt;
float baro_alt;
float baro_alt_raw;
float baro_alt_f;
float baro_alt_f_old;
float baro_vel;
float baro_vel_f;
float baro_temp;
bool baro_update_val;
bool baro_init_check = false;

float baro_error_timeout = 0.5f;
float baro_error_time = 0.0f;

static float baro_vel_buffer[2][BARO_VEL_BUF_LEN + 5];
static float baro_delay = 0.05f;

static int baro_vel_r_ptr = 0;
static int baro_vel_w_ptr = 0; 
static float baro_alt_bias = 0;
static float baro_alt_vel_zero = 0;
static bool baro_vel_enable = false;
float baro_get_error_time()
{
	return baro_error_time;
}

uint8_t baro_get_status()
{
	if(baro_error_time >= baro_error_timeout){
		return SENSOR_STATUS_FAIL;
	}else if(baro_init_check == false){
		return SENSOR_STATUS_INIT;
	}else{
		return SENSOR_STATUS_OK;
	}
}

bool baro_get_update()
{
	return baro_update_val;
}

float baro_get_pres()
{
	return baro_pres;
}

float baro_get_alt()
{
	return baro_alt;
}

float baro_get_alt_raw()
{
	return baro_alt_raw;
}

float baro_get_alt_f()
{
	return baro_alt_f;
}

float baro_get_vel()
{
	return baro_vel;
}

float baro_get_vel_f()
{
	return baro_vel_f;
}

float baro_get_temp()
{
	return baro_temp;
}

float baro_get_baro_vel(int i)
{
	return baro_vel_buffer[i][baro_vel_r_ptr % BARO_VEL_BUF_LEN];
}

float baro_get_baro_alt_bias()
{
	return baro_alt_bias;
}

float baro_get_baro_alt_vel_zero()
{
	return baro_alt_vel_zero;
}

void baro_set_baro_vel_enable(bool enable)
{
	baro_vel_enable = enable;
}

void baro_init(void)
{
	INFO(DEBUG_ID,"init");

	baro_ground_alt = 0.0f;
	baro_update_val = 0;
}

void baro_update(float dt)
{
	static float baro_dt = 0.0f;
	nav_s pos; 
	float vel_grd_tmp[2];
	float vel_coef[2];
	
	nav_get_pos(&pos);
	baro_dt += dt;

	if(pos.vel_valid == true){
		baro_vel_buffer[0][baro_vel_w_ptr % BARO_VEL_BUF_LEN] = pos.vel_grd[0];
		baro_vel_buffer[1][baro_vel_w_ptr % BARO_VEL_BUF_LEN] = pos.vel_grd[1];
		baro_vel_w_ptr++;
	}

	if(hal_get_baro(dt,&baro_pres,&baro_alt_raw,&baro_temp) == true){
 	if(baro_init_check == false){
			baro_ground_alt = baro_alt_raw;
			baro_init_check = true;
			INFO(DEBUG_ID,"ground:%3.3f",baro_ground_alt);
		}
		baro_alt_raw -= baro_ground_alt;
		if(baro_vel_enable)
                          {
			baro_vel_r_ptr = baro_vel_w_ptr - (baro_delay * MAIN_LOOP_HZ);
			if(baro_vel_r_ptr < 0){
				baro_vel_r_ptr = 0;
			}
			vel_grd_tmp[0] = baro_vel_buffer[0][baro_vel_r_ptr % BARO_VEL_BUF_LEN];
			vel_grd_tmp[1] = baro_vel_buffer[1][baro_vel_r_ptr % BARO_VEL_BUF_LEN];
			vel_coef[0] = vel_grd_tmp[0] > 0 ? BARO_VEL_FRONT : BARO_VEL_BACK;
			vel_coef[1] = vel_grd_tmp[1] > 0 ? BARO_VEL_RIGHT : BARO_VEL_LEFT;
			
 			baro_alt_vel_zero = pos_control_get_pos_ned_z_target();
			baro_alt_bias = vel_coef[0] * sq(vel_grd_tmp[0]) + vel_coef[1] * sq(vel_grd_tmp[1]);
			baro_alt_bias = complementary_filter(baro_alt_vel_zero - baro_alt_raw,baro_alt_bias,0.7f);
			baro_alt_bias = constrain_float(baro_alt_bias,-1.5f,1.5f);

       // tof("baro_alt_vel_zero = %3.3f\n",baro_alt_vel_zero);
       // tof("baro_alt_bias = %3.3f\n",baro_alt_bias);
		}else{
			baro_alt_bias *= 0.95f;
		}
		
#ifdef BARO_VEL_ENABLE
		baro_alt = baro_alt_raw + baro_alt_bias;
#else
		baro_alt = baro_alt_raw;
#endif 
		
		baro_alt_f = complementary_filter(baro_alt_f,baro_alt,BARO_ALT_FILTER);
		baro_vel = (baro_alt_f - baro_alt_f_old) / baro_dt;
		baro_vel_f = complementary_filter(baro_vel_f,baro_vel,BARO_VEL_FILTER);
		baro_alt_f_old = baro_alt_f;
		baro_error_time = 0.0f;
		baro_update_val = true;
		baro_dt = 0.0f;
//       tof("baro_vel_f =%3.3f baro_alt_f = %3.3f\n baro_alt = %3.3f\n",baro_vel_f,baro_alt_f,baro_alt_raw);
	}else{
		baro_error_time += dt;
		baro_update_val = false;
	}
	DEBUG_HZ(DEBUG_ID,5,dt,"alt:%+3.3f pres:%3.3f temp:%3.3f pos_z:%3.3f",baro_alt,baro_pres,baro_temp,pos.pos_ned[2]);
}


