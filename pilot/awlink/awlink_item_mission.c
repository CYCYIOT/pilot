#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "app_debug.h"
#include "app_system.h"
#include "app_mission.h"

#include "awlink_protocol.h"
#include "awlink_encode.h"
#include "awlink_item_system.h"

#define DEBUG_ID DEBUG_ID_LINK

#define AWLINK_ITEM_MISSION_CONTROL_CLEAR	0
#define AWLINK_ITEM_MISSION_CONTROL_SAVE	1

typedef struct PACKED{
	uint8_t control;
}awlink_mission_control_s;

typedef struct PACKED{
	uint16_t count;
}awlink_mission_get_s;

typedef struct PACKED{
	uint16_t total;
	uint16_t count;
	uint8_t type;
	uint8_t length;
	uint8_t data[100];
}awlink_mission_item_s;

void awlink_encode_mission_item(awlink_s * link,uint16_t count)
{
	awlink_msg_s msg;
	awlink_mission_item_s item;
	uint8_t type = 0;
	uint8_t len = 0;

	type = mission_get_item_type(count);
	if(type > 0){
		switch(type){
			case MISSION_TYPE_CONTROL:
				len = sizeof(mission_control_s);
				break;
		}

		if(mission_get_item_data(count,item.data,len) == true){

			item.total = mission_get_total();
			item.count = count;
			item.type = type;
			item.length = len;
			
			msg.item_id = AWLINK_ITEM_MISSION;
			msg.subitem_id = AWLINK_ITEM_MISSION_ITEM;
			msg.data = (uint8_t *)&item;
			msg.length = sizeof(item);
			
			DEBUG(DEBUG_ID,"mission item (%d,%d)%d",item.total,item.count,item.type);
			
			awlink_encode(link,&msg,false,true);
		}
	}
}

void awlink_decode_mission_get(awlink_s * link,awlink_msg_s * msg_rev)
{
	awlink_mission_get_s * data;
	data = (awlink_mission_get_s *)msg_rev->data;
	
	if(data->count == 0xFFFF){
		link->mission.total = mission_get_total();
		link->mission.stream_run = true;
		link->mission.count = 1;
		DEBUG(DEBUG_ID,"get all mission:%d",link->mission.total);
	}else{
		awlink_encode_mission_item(link,data->count);
	}
}

void awlink_decode_mission_item(awlink_s * link,awlink_msg_s * msg_rev)
{
	awlink_mission_item_s * data;
	data = (awlink_mission_item_s *)msg_rev->data;
	
	DEBUG(DEBUG_ID,"add mission item:%d %d len:%d",data->count,data->type,data->length);
	mission_add_item(data->count,data->type,data->length,data->data);
}

void awlink_decode_mission_control(awlink_s * link,awlink_msg_s * msg_rev)
{
	awlink_mission_control_s * data;
	data = (awlink_mission_control_s *)msg_rev->data;

	if(system_get_armed() == false){
		if(data->control == AWLINK_ITEM_MISSION_CONTROL_SAVE){
			DEBUG(DEBUG_ID,"save mission");
			mission_save();
		}else if(data->control == AWLINK_ITEM_MISSION_CONTROL_CLEAR){
			DEBUG(DEBUG_ID,"clear mission");
			mission_clear();
		}
	}
}

void awlink_handle_mission(awlink_s * link,awlink_msg_s * msg)
{
	switch(msg->subitem_id){
		case AWLINK_ITEM_MISSION_CONTROL:
			awlink_decode_mission_control(link,msg);
			break;
		case AWLINK_ITEM_MISSION_GET:
			awlink_decode_mission_get(link,msg);
			break;
		case AWLINK_ITEM_MISSION_ITEM:
			awlink_decode_mission_item(link,msg);
			break;
	}
}

void awlink_stream_mission(awlink_s * link)
{
	if(link->mission.stream_run == true){
		if(link->mission.count <= link->mission.total){
			awlink_encode_mission_item(link,link->mission.count);
			link->mission.count++;
		}else if(link->mission.count >= link->mission.total){
			link->mission.stream_run = false;
			link->mission.count = 1;
		}
	}
}

