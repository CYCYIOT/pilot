#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "app_param.h"
#include "app_debug.h"
#include "app_system.h"

#include "awlink_protocol.h"
#include "awlink_encode.h"

#define DEBUG_ID DEBUG_ID_LINK

typedef struct PACKED{
	uint8_t control;
}awlink_param_control_s;
	
typedef struct PACKED{
	uint16_t id;
	char name[PARAM_NAME_LENGTH];
}awlink_param_clear_s;

typedef struct PACKED{
	uint16_t id;
	char name[PARAM_NAME_LENGTH];
}awlink_param_get_s;

typedef struct PACKED{
	uint16_t id_max;
	uint16_t id;
	char name[PARAM_NAME_LENGTH];
	float value;
}awlink_param_item_s;

void awlink_encode_param_item(awlink_s * link,uint16_t id_max,uint16_t id,char name[PARAM_NAME_LENGTH],float val)
{
	awlink_msg_s msg;
	awlink_param_item_s data;

	data.id_max = id_max;
	data.id = id;
	data.value = val;
	strncpy(data.name,name,PARAM_NAME_LENGTH);

	msg.item_id = AWLINK_ITEM_PARAM;
	msg.subitem_id = AWLINK_ITEM_PARAM_ITEM;
	msg.data = (uint8_t *)&data;
	msg.length = sizeof(data);

	awlink_encode(link,&msg,false,true);
}

void awlink_decode_param_clear(awlink_s * link,awlink_msg_s * msg_rev)
{
/*
	awlink_param_clear_s * data;
	data = (awlink_param_clear_s *)msg_rev->data;
*/
}

void awlink_decode_param_get(awlink_s * link,awlink_msg_s * msg_rev)
{
	awlink_param_get_s * data;
	data = (awlink_param_get_s *)msg_rev->data;
   
	if(data->id == 0xFFFF){
		link->param.total = param_get_total();
		link->param.stream_run = true;
		link->param.count = 1;
		DEBUG(DEBUG_ID,"get all param:%d",link->param.total);
	}else{
		char name[PARAM_NAME_LENGTH];
		float val;
		
		if(param_get_bynum(data->id,name,&val) == true){
			DEBUG(DEBUG_ID,"get param:(%s)(%5.5f) ok",name,val);
			awlink_encode_param_item(link,link->param.total,data->id,name,val);
		}		
	}
}

void awlink_decode_param_item(awlink_s * link,awlink_msg_s * msg_rev)
{
	awlink_param_item_s * data;
	data = (awlink_param_item_s *)msg_rev->data;

	if(param_set(data->name,data->value) == true){
		DEBUG(DEBUG_ID,"set param:(%s)(%5.5f) ok",data->name,data->value);
	}else{
		DEBUG(DEBUG_ID,"set param:(%s)(%5.5f) fail",data->name,data->value);	
	}
}

void awlink_decode_param_control(awlink_s * link,awlink_msg_s * msg_rev)
{
	awlink_param_control_s * data;
	data = (awlink_param_control_s *)msg_rev->data;

	if(data->control == 0){
		if(system_get_armed() == false){
			param_save();
		}
	}
}

void awlink_handle_parameter(awlink_s * link,awlink_msg_s * msg)
{
	switch(msg->subitem_id){
		case AWLINK_ITEM_PARAM_CLEAR:
			awlink_decode_param_clear(link,msg);
			break;
		case AWLINK_ITEM_PARAM_GET:
			awlink_decode_param_get(link,msg);
			break;
		case AWLINK_ITEM_PARAM_ITEM:	
			awlink_decode_param_item(link,msg);
			break;
		case AWLINK_ITEM_PARAM_CONTROL:	
			awlink_decode_param_control(link,msg);
			break;
	}
}

void awlink_stream_parameter(awlink_s * link)
{
	if(link->param.stream_run == true){
		if(link->param.count <= link->param.total){
			char name[PARAM_NAME_LENGTH];
			float val;
			if(param_get_bynum(link->param.count,name,&val) == true){
				awlink_encode_param_item(link,link->param.total,link->param.count,name,val);
			}
			link->param.count++;
		}else if(link->param.count >= link->param.total){
			link->param.stream_run = false;
			link->param.count = 1;
		}
	}
}

