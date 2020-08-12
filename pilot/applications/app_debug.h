#ifndef _APP_DEBUG_H_
#define _APP_DEBUG_H_

#include <stdbool.h>
#include <stdio.h>
#include <stdint.h>

#define DEBUG_NAME_LENGTH	15

#define DEBUG_LEVEL_ERR		0
#define DEBUG_LEVEL_INFO	1
#define DEBUG_LEVEL_DEBUG	2

enum DEBUG_ID{
    DEBUG_ID_MIN = 0,
	DEBUG_ID_IMU,
	DEBUG_ID_MAG,
	DEBUG_ID_BARO,
	DEBUG_ID_RANGEFINDER,
	DEBUG_ID_LOG,
	DEBUG_ID_PARAM,
	DEBUG_ID_CONTROL,
	DEBUG_ID_SYSTEM,
	DEBUG_ID_MOTOR,
	DEBUG_ID_RC,
	DEBUG_ID_ATTITUDE,
	DEBUG_ID_NAV,
	DEBUG_ID_NAV_BARO,
	DEBUG_ID_NAV_FLOW,
	DEBUG_ID_NAV_GPS,
	DEBUG_ID_NAV_RF,
	DEBUG_ID_GPS,
	DEBUG_ID_FAILSAFE,
	DEBUG_ID_HAL,
	DEBUG_ID_NETWORK,
	DEBUG_ID_BATT,
	DEBUG_ID_FLOW,
	DEBUG_ID_LINK,
	DEBUG_ID_MAIN,
	DEBUG_ID_MISSION,
	DEBUG_ID_MAX,	
};

#define DEBUG_ID_MIN_NAME			"NULL"
#define DEBUG_ID_IMU_NAME			"IMU"
#define DEBUG_ID_MAG_NAME			"MAG"
#define DEBUG_ID_BARO_NAME			"BARO"
#define DEBUG_ID_RANGEFINDER_NAME	"RF"
#define DEBUG_ID_LOG_NAME			"LOG"
#define DEBUG_ID_MOTOR_NAME			"MOTOR"
#define DEBUG_ID_RC_NAME			"RC"
#define DEBUG_ID_PARAM_NAME			"PARAM"
#define DEBUG_ID_CONTROL_NAME		"CON"
#define DEBUG_ID_SYSTEM_NAME		"SYS"
#define DEBUG_ID_ATTITUDE_NAME		"ATT"
#define DEBUG_ID_NAV_NAME			"NAV"
#define DEBUG_ID_NAV_BARO_NAME		"NAV_BARO"
#define DEBUG_ID_NAV_FLOW_NAME		"NAV_FLOW"
#define DEBUG_ID_NAV_GPS_NAME		"NAV_GPS"
#define DEBUG_ID_NAV_RF_NAME		"NAV_RF"
#define DEBUG_ID_GPS_NAME			"GPS"
#define DEBUG_ID_FAILSAFE_NAME		"FS"
#define DEBUG_ID_HAL_NAME			"HAL"
#define DEBUG_ID_NETWORK_NAME		"NET"
#define DEBUG_ID_BATT_NAME			"BATT"
#define DEBUG_ID_FLOW_NAME			"FLOW"
#define DEBUG_ID_LINK_NAME			"LINK"
#define DEBUG_ID_MAIN_NAME			"MAIN"
#define DEBUG_ID_MISSION_NAME		"MISS"

typedef struct{
	char name[DEBUG_NAME_LENGTH];
}debug_s;

extern uint8_t debug_level;
extern uint16_t debug_module;
extern debug_s debug_list[DEBUG_ID_MAX];

#define DEBUG_BUF_LEN 64

#define DEBUG_HZ(module,hz,dt,format,...)\
	do {\
		if(debug_module == module && debug_level >= DEBUG_LEVEL_DEBUG){\
			static float time = 0;\
			time += dt;\
			if(time >= (1.0f/hz)){\
				time -= (1.0f/hz);\
				printf("[%s][%s]"format"\r\n","DEBUG",debug_list[module].name,##__VA_ARGS__ );\
			}\
		}\
	} while (0)

#define DEBUG(module,format,...)\
	do {\
		if(debug_module == module && debug_level >= DEBUG_LEVEL_DEBUG){\
			printf("[%s][%s]"format"\r\n","DEBUG",debug_list[module].name,##__VA_ARGS__ );\
		}\
	} while (0)

#define INFO(module,format,...)\
	do {\
		if(debug_level >= DEBUG_LEVEL_INFO){ \
			char buf[DEBUG_BUF_LEN]; \
			snprintf(buf,DEBUG_BUF_LEN,"[%s]"format"\r\n",debug_list[module].name,##__VA_ARGS__);\
			debug_output_info(buf); \
		} \
	} while (0)

#define ERR(module,format,...)\
	do {\
		char buf[DEBUG_BUF_LEN]; \
		snprintf(buf,DEBUG_BUF_LEN,"[%s]"format"\r\n",debug_list[module].name,##__VA_ARGS__);\
		debug_output_err(buf); \
	} while (0)

void debug_init();
void debug_level_set(uint8_t level);
void debug_module_set(char * module);
void debug_module_show();
void debug_status_show();
void debug_output_info(char * buf);
void debug_output_err(char * buf);

#endif
