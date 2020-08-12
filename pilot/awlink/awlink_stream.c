#include "awlink_encode.h"

#include "awlink_item_status.h"
#include "awlink_item_system.h"
#include "awlink_item_parameter.h"
#include "awlink_item_log.h"
#include "awlink_item_mission.h"
#include "awlink_item_file.h"

void awlink_stream_set_rate(awlink_s * link,uint8_t id,uint8_t rate)
{
	if(rate == 0){
		link->stream.rate[id] = 0.0f;
	}else{
		link->stream.rate[id] = 1.0f / (float)rate;
	}
}

void  awlink_stream_stop(awlink_s * link)
{
	uint8_t i = 0;
	
	for(i = 0 ; i < AWLINK_ITEM_STATUS_NUM ; i++){
		awlink_stream_set_rate(link,i,0);
	}	
}

void awlink_stream_init(awlink_s * link)
{
	awlink_stream_stop(link);
}

void awlink_stream_handle(awlink_s * link,uint8_t id)
{
	switch(id){
		case AWLINK_ITEM_STATUS_BASIC_INFO:	
			awlink_encode_status_base_info(link);		
			break;
		case AWLINK_ITEM_STATUS_GPS_INFO:
			awlink_encode_status_gps_info(link);
			break;
		case AWLINK_ITEM_STATUS_SENSOR_INFO:
			awlink_encode_status_sensor_info(link);
			break;
		case AWLINK_ITEM_STATUS_SENSOR_CALIB_INFO:
			awlink_encode_status_sensor_calib_info(link);
			break;
		case AWLINK_ITEM_STATUS_USER1_INFO:
			awlink_encode_status_user1_info(link);
			break;
		case AWLINK_ITEM_STATUS_USER2_INFO:
			awlink_encode_status_user2_info(link);
			break;
		case AWLINK_ITEM_STATUS_USER3_INFO:
			awlink_encode_status_user3_info(link);
			break;
		case AWLINK_ITEM_STATUS_USER4_INFO:
			awlink_encode_status_user4_info(link);
			break;
		case AWLINK_ITEM_STATUS_MISSION_INFO:
			awlink_encode_status_mission_info(link);
			break;
		case AWLINK_ITEM_STATUS_MP_INFO:
			awlink_encode_status_mp_info(link);
			break;	}
}

void awlink_stream_update(float dt,awlink_s * link)
{
	uint8_t i;

	for(i = 0;i < AWLINK_ITEM_STATUS_NUM;i++){
		link->stream.time[i] += dt;
		if(link->stream.rate[i] != 0 && link->stream.time[i] >= link->stream.rate[i]){
			link->stream.time[i] = 0.0f;
			awlink_stream_handle(link,i);
		}
	}

	awlink_stream_log(link);
	awlink_stream_parameter(link);
	awlink_stream_mission(link);
	awlink_stream_file(dt,link);
}

