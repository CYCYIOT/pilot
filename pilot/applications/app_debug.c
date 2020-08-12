#include <string.h>

#include "app_awlink.h"
#include "app_debug.h"
#include "app_log.h"

uint8_t debug_level = DEBUG_LEVEL_INFO;
uint16_t debug_module = DEBUG_ID_MIN;

debug_s debug_list[DEBUG_ID_MAX];

void debug_output_info(char * buf)
{
	log_msg(DEBUG_LEVEL_INFO,(uint8_t *)buf);
	awlink_msg(DEBUG_LEVEL_INFO,(uint8_t *)buf);
	
	printf("[INFO]%s",buf);
}

void debug_output_err(char * buf)
{
	log_msg(DEBUG_LEVEL_ERR,(uint8_t *)buf);
	awlink_msg(DEBUG_LEVEL_ERR,(uint8_t *)buf);
	
	printf("[ERR]%s",buf);
}

void debug_status_show()
{
	printf("debug status:%d %d[%s]\r\n",debug_level,debug_module,debug_list[debug_module].name);
}

void debug_module_show()
{
	uint16_t count;

	printf("modules:\r\n");
	for(count = 0 ; count < DEBUG_ID_MAX ; count++){
		printf("[%d][%s]\r\n",count,debug_list[count].name);
	}
}

void debug_level_set(uint8_t level)
{
	debug_level = level;
}

void debug_module_set(char * module)
{
	uint16_t count;

	debug_level_set(DEBUG_LEVEL_DEBUG);
	for(count = 0 ; count < DEBUG_ID_MAX ; count++){
		if(strncmp(debug_list[count].name,module,strlen(module)) == 0){
			debug_module = count;
			printf("debug module:[%d][%s]\r\n",debug_module,debug_list[debug_module].name);
		}
	}
}

void debug_init()
{
	debug_level = DEBUG_LEVEL_INFO;
	debug_module = DEBUG_ID_MIN;

	strncpy(debug_list[DEBUG_ID_MIN].name,DEBUG_ID_MIN_NAME,DEBUG_NAME_LENGTH);
	strncpy(debug_list[DEBUG_ID_IMU].name,DEBUG_ID_IMU_NAME,DEBUG_NAME_LENGTH);
	strncpy(debug_list[DEBUG_ID_MAG].name,DEBUG_ID_MAG_NAME,DEBUG_NAME_LENGTH);
	strncpy(debug_list[DEBUG_ID_BARO].name,DEBUG_ID_BARO_NAME,DEBUG_NAME_LENGTH);
	strncpy(debug_list[DEBUG_ID_RANGEFINDER].name,DEBUG_ID_RANGEFINDER_NAME,DEBUG_NAME_LENGTH);
	strncpy(debug_list[DEBUG_ID_LOG].name,DEBUG_ID_LOG_NAME,DEBUG_NAME_LENGTH);
	strncpy(debug_list[DEBUG_ID_MOTOR].name,DEBUG_ID_MOTOR_NAME,DEBUG_NAME_LENGTH);
	strncpy(debug_list[DEBUG_ID_RC].name,DEBUG_ID_RC_NAME,DEBUG_NAME_LENGTH);
	strncpy(debug_list[DEBUG_ID_PARAM].name,DEBUG_ID_PARAM_NAME,DEBUG_NAME_LENGTH);
	strncpy(debug_list[DEBUG_ID_CONTROL].name,DEBUG_ID_CONTROL_NAME,DEBUG_NAME_LENGTH);
	strncpy(debug_list[DEBUG_ID_SYSTEM].name,DEBUG_ID_SYSTEM_NAME,DEBUG_NAME_LENGTH);
	strncpy(debug_list[DEBUG_ID_ATTITUDE].name,DEBUG_ID_ATTITUDE_NAME,DEBUG_NAME_LENGTH);
	strncpy(debug_list[DEBUG_ID_NAV].name,DEBUG_ID_NAV_NAME,DEBUG_NAME_LENGTH);
	strncpy(debug_list[DEBUG_ID_NAV_BARO].name,DEBUG_ID_NAV_BARO_NAME,DEBUG_NAME_LENGTH);
	strncpy(debug_list[DEBUG_ID_NAV_FLOW].name,DEBUG_ID_NAV_FLOW_NAME,DEBUG_NAME_LENGTH);
	strncpy(debug_list[DEBUG_ID_NAV_GPS].name,DEBUG_ID_NAV_GPS_NAME,DEBUG_NAME_LENGTH);
	strncpy(debug_list[DEBUG_ID_NAV_RF].name,DEBUG_ID_NAV_RF_NAME,DEBUG_NAME_LENGTH);
	strncpy(debug_list[DEBUG_ID_GPS].name,DEBUG_ID_GPS_NAME,DEBUG_NAME_LENGTH);
	strncpy(debug_list[DEBUG_ID_FAILSAFE].name,DEBUG_ID_FAILSAFE_NAME,DEBUG_NAME_LENGTH);
	strncpy(debug_list[DEBUG_ID_HAL].name,DEBUG_ID_HAL_NAME,DEBUG_NAME_LENGTH);
	strncpy(debug_list[DEBUG_ID_NETWORK].name,DEBUG_ID_NETWORK_NAME,DEBUG_NAME_LENGTH);
	strncpy(debug_list[DEBUG_ID_BATT].name,DEBUG_ID_BATT_NAME,DEBUG_NAME_LENGTH);
	strncpy(debug_list[DEBUG_ID_FLOW].name,DEBUG_ID_FLOW_NAME,DEBUG_NAME_LENGTH);
	strncpy(debug_list[DEBUG_ID_LINK].name,DEBUG_ID_LINK_NAME,DEBUG_NAME_LENGTH);
	strncpy(debug_list[DEBUG_ID_MAIN].name,DEBUG_ID_MAIN_NAME,DEBUG_NAME_LENGTH);
	strncpy(debug_list[DEBUG_ID_MISSION].name,DEBUG_ID_MISSION_NAME,DEBUG_NAME_LENGTH);
}

