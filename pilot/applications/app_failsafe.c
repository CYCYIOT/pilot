#include "awlink_stream.h"

#include "app_motor.h"
#include "app_rc.h"
#include "app_param.h"
#include "app_system.h"
#include "app_control.h"
#include "app_failsafe.h"
#include "app_debug.h"
#include "app_imu.h"
#include "app_batt.h"
#include "app_nav.h"
#include "hal.h"
#include "lib_math.h"
#include "hal_stm8s.h"
#include "pilot_steam_control.h"
#define DEBUG_ID DEBUG_ID_FAILSAFE

#define NORMAL_RC_RATE           5
#define MOTOR_LOCKED_VALUE       100
#define FS_TIME_OUT              5000

#ifdef M                             //减速组
#define MOTOR_LOCKED             7
#define COLLISION_VAL            6
#define LOCKED_ON                1
#else                               //其他
#define MOTOR_LOCKED             10
#define COLLISION_VAL            10
#endif

typedef struct{
	uint8_t mode1;
	float mode1_param1;
	float mode1_param2;
	uint8_t mode2;
	float mode2_param1;
	float mode2_param2;
	uint8_t mode3;
	float mode3_param1;
	float mode3_param2;
}fs_s;

fs_s fs_modes[FS_MODE_MAX];
uint32_t fs_mode;

float fs_rc_timeout;

static float motor_locked_check_rate = 0.2f; 
static float motor_locked_check_time = 0.0f;
static uint8_t motor_locked_cnt[4] = {0};
static uint8_t motor_val[8] = {0};
static uint8_t motor_locked_check_flag = 0;
static float motor_output_thres_val = 0.9f;
static float c_acc_val;

static float collision_detection_time = 0.0f;
static float collision_detection_rate = 0.02f;

static bool fs_att_limit_enable = true;
static bool fs_collision_enable = true;
static bool fs_motor_locked_enable = true;
static bool fs_collision_file_enable = true;
static int  fs_co_fi_count = 0;

void failsafe_set_motor_locked_enable(bool enable)
{
	fs_motor_locked_enable = enable;
}

void failsafe_set_att_limit_enable(bool enable)
{
	fs_att_limit_enable = enable;
}

void failsafe_set_collision_file_enable(bool enable)
{
    fs_collision_file_enable = enable;
	fs_co_fi_count=0;
}

void failsafe_set_collision_enable(bool enable)
{
	fs_collision_enable = enable;
}

uint32_t failsafe_get_mode()
{	
	return fs_mode;
}

uint8_t *failsafe_get_motor_val()
{
	return motor_val;
}

uint8_t *failsafe_get_motor_locked_cnt()
{
	return motor_locked_cnt;
}

uint8_t failsafe_get_collision_val()
{
	return c_acc_val;
}

void failsafe_mode_update(uint8_t mode)
{
	if(control_set_mode(fs_modes[mode].mode1,fs_modes[mode].mode1_param1,fs_modes[mode].mode1_param2) == false){
		if(control_set_mode(fs_modes[mode].mode2,fs_modes[mode].mode2_param1,fs_modes[mode].mode2_param2) == false){
			if(control_set_mode(fs_modes[mode].mode3,fs_modes[mode].mode3_param1,fs_modes[mode].mode3_param2) == false){
				INFO(DEBUG_ID,"failsafe mode(%d) update failed",mode);
			}
		}
	}
}

bool failsafe_check_mode_set(uint8_t mode)
{
	if((fs_mode & (1 << mode)) > 0){
		return true;
	}else{
		return false;
	}
}

bool failsafe_mode_update_clear(uint8_t mode)
{
	int count = 0;

	if(failsafe_check_mode_set(mode) == true){
		fs_mode &= ~(1 << mode);

		for(count = (mode + 1) ; count < FS_MODE_MAX ; count++){
			if(failsafe_check_mode_set(count) == true){
				return false;
			}
		}

		for(count = mode ; count > 0 ; count--){
			if(failsafe_check_mode_set(count) == true){
				failsafe_mode_update(count);
				return false;
			}
		}

		return true;
	}

	return false;
}

bool failsafe_mode_update_set(uint8_t mode)
{
	int count = 0;

	if(failsafe_check_mode_set(mode) == false){
		fs_mode |= (1 << mode);
		
		for(count = (mode + 1) ; count < FS_MODE_MAX ; count++){
			if(failsafe_check_mode_set(count) == true){
				return false;
			}
		}

		failsafe_mode_update(mode);

		return true;
	}
	
	return false;
}

void rc_timeout_check(float dt)
{
	static float rc_timeout_sum = 0.0f;
	
	if(rc_get_update() == false){
		rc_timeout_sum += dt;
	}else{
		rc_timeout_sum = 0.0f;
	}

	if(rc_timeout_sum >= fs_rc_timeout){
		if(failsafe_mode_update_set(FS_MODE_RC_TIMEOUT) == true){
			ERR(DEBUG_ID,"FS_MODE_RC_TIMEOUT");
		}
	}
}

void low_batt_check(float dt)
{	
	if(system_get_batt_low() == true){
		if(failsafe_mode_update_set(FS_MODE_BAT_LOW) == true){
			ERR(DEBUG_ID,"LOW_BATT");
		}
	}
}

void high_temp_check(float dt)
{	
	if(system_get_imu_temp_limit() == true){
		if(failsafe_mode_update_set(FS_MODE_HIGH_TEMP) == true){				
			ERR(DEBUG_ID,"HIGH TEMP");
		}
	}
}

void att_limit_check(float dt)
{
	if(fs_att_limit_enable == false){
		return;
	}

	if(system_get_att_limit() == true){
		if(failsafe_mode_update_set(FS_MODE_ATT_LIMIT) == true){
			ERR(DEBUG_ID,"ATT LIMIT");
		}
	}				
}

void collision_check(float dt)
{
	nav_s nav;
	uint8_t mode;

	if(fs_collision_enable == false){
		return;
	}
    if(fs_collision_file_enable == false){
       if(fs_co_fi_count++ > FS_TIME_OUT){
          fs_co_fi_count=0;
		  fs_collision_file_enable=true;
	   }
		return ;	 
	}
	
	mode = control_get_mode();
	collision_detection_time += dt;
	if(collision_detection_time > collision_detection_rate){
		collision_detection_time = 0.0f;
		if(mode != CONTROL_MODE_TAKEOFF && mode != CONTROL_MODE_THROWN){
			nav_get_pos(&nav);
			c_acc_val = pythagorous_v3f(nav.acc_ned);
			if((c_acc_val > COLLISION_VAL)){
				if(failsafe_mode_update_set(FS_MODE_COLLISION) == true){
					ERR(DEBUG_ID,"COLLISION");
					debug_t("COLLISION\n");
				}
			}
		}	
	}
}

void motor_locked_check(float dt)
{
	uint8_t i;
	float motor_output[4];

    if(control_get_mode() == CONTROL_MODE_TAKEOFF || control_get_mode() == CONTROL_MODE_LAND || control_get_mode() == CONTROL_MODE_THROWN){
		//debug_t("motor locked test\n");
		return;
    	}

		
	motor_locked_check_time += dt;
	if(motor_locked_check_time > motor_locked_check_rate){				
		stm8s_read(motor_val,6);
		//debug_t("motor_val[0] = %d motor_val[1] = %d motor_val[2] = %d motor_val[3] = %d  motor_val[4] = %d  \n",motor_val[0],motor_val[1],motor_val[2],motor_val[3],motor_val[4]);
		motor_locked_check_time = 0.0f;
		motor_output_get(motor_output);
	//	debug_t("motor_output[0] = %f motor_output[1] = %f motor_output[2] = %f motor_output[3] = %f\n",motor_output[0],motor_output[1],motor_output[2],motor_output[3]);
		for(i = 0;i < 4;i++){
			if(motor_output[i] > motor_output_thres_val){				
				motor_locked_check_flag = 0;
				break;	
			}else{
				motor_locked_check_flag = 1;
			}
		}
		if(motor_locked_check_flag == 1){		
			for(i = 2;i < 5;i++){
				if(motor_val[i] > MOTOR_LOCKED_VALUE){
					motor_locked_cnt[i - 2] += 1;				
				}else{
					motor_locked_cnt[i - 2] = 0;
				}
			}
		}else{
			motor_locked_cnt[0] = 0;
			motor_locked_cnt[1] = 0;
			motor_locked_cnt[2] = 0;
			motor_locked_cnt[3] = 0;		
		}					
		for(i = 0;i < 4;i++){
			if(motor_locked_cnt[i] == MOTOR_LOCKED){
				
				if(failsafe_mode_update_set(FS_MODE_MOTOR_LOCKED) == true){
					ERR(DEBUG_ID,"MOTOR_LOCKED");
				}
				debug_t("locked\n");
				motor_locked_cnt[i] = 0;
			}
		}

	}
	
}

void failsafe_init(void)
{
	int count = 0;
	
	INFO(DEBUG_ID,"init");
	
	fs_mode = 0;
	
	for(count = 0 ; count < FS_MODE_MAX ; count++){
		fs_modes[count].mode1 = CONTROL_MODE_STOP;
		fs_modes[count].mode1_param1 = 0.0f;
		fs_modes[count].mode1_param2 = 0.0f;
		fs_modes[count].mode2 = CONTROL_MODE_STOP;
		fs_modes[count].mode2_param1 = 0.0f;
		fs_modes[count].mode2_param2 = 0.0f;
		fs_modes[count].mode3 = CONTROL_MODE_STOP;
		fs_modes[count].mode3_param1 = 0.0f;
		fs_modes[count].mode3_param2 = 0.0f;
	}

	fs_modes[FS_MODE_RC_TIMEOUT].mode1 = CONTROL_MODE_RTH;
	fs_modes[FS_MODE_RC_TIMEOUT].mode2 = CONTROL_MODE_LAND;

	fs_modes[FS_MODE_BAT_LOW].mode1 = CONTROL_MODE_RTH;
	fs_modes[FS_MODE_BAT_LOW].mode2 = CONTROL_MODE_LAND;

	fs_modes[FS_MODE_HIGH_TEMP].mode1 = CONTROL_MODE_LAND;
}

void failsafe_param_init(void)
{
	param_set_var(FS__RC_TIMEOUT_NAME	,&fs_rc_timeout);  //0.5f
}

void failsafe_reset(float dt)
{
	uint8_t i;
	for(i = 0; i < 4;i++){
		motor_val[i] = 0;
		motor_locked_cnt[i] = 0;
	}
	
	fs_mode = 0;
}

void failsafe_update(float dt)
{
  // printf("COLLISION_VAL = %d \n",COLLISION_VAL);
	if(system_get_armed() == true){
		att_limit_check(dt);
		low_batt_check(dt);
		rc_timeout_check(dt);
		//high_temp_check(dt);
#ifdef LOCKED_ON
		motor_locked_check(dt);	
#endif
       if(get_collision_flag()== true){
		collision_check(dt);
       }
	}else{		
		failsafe_reset(dt);
	}

	DEBUG_HZ(DEBUG_ID,1,dt,"%d",fs_mode);
}

