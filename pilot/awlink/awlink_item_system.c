#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "awlink_protocol.h"
#include "awlink_encode.h"

#include "app_flow.h"
#include "app_system.h"
#include "app_param.h"

typedef struct{
	uint8_t ack;
	uint8_t item_id;
	uint8_t subitem_id;
}awlink_system_ack_s;

typedef struct{
	uint8_t version[8];
}awlink_system_awlink_ver_s;

typedef struct{
	uint8_t version[20];
}awlink_system_awpilot_ver_s;

typedef struct{
	uint8_t heart;
}awlink_system_heart_s;

typedef struct{
	uint8_t level;
	uint8_t data[64];
}awlink_system_msg_s;

void awlink_encode_system_msg(awlink_s * link,uint8_t level,uint8_t * buf)
{
	awlink_msg_s msg;
	awlink_system_msg_s data;

	data.level = level;
	memset(data.data,0,64);
	snprintf((char *)data.data,64,(char *)buf);

	msg.item_id = AWLINK_ITEM_SYSTEM;
	msg.subitem_id = AWLINK_ITEM_SYSTEM_MSG;
	msg.data = (uint8_t *)&data;
	msg.length = sizeof(data);

	awlink_encode(link,&msg,false,false);
}

void awlink_encode_system_aruco_ack(awlink_s * link,uint8_t ack,uint8_t i_id,uint8_t si_id)
{
    awlink_msg_s msg;
	awlink_system_ack_s data;

	data.ack = ack;
	data.item_id = i_id;
	data.subitem_id = si_id;

	msg.item_id = AWLINK_ITEM_STATUS;
	msg.subitem_id = AWLINK_ITEM_STATUS_ARUCO_OK;
	msg.data = (uint8_t *)&data;
	msg.length = sizeof(data);

	awlink_encode(link,&msg,false,false);



}
void awlink_encode_system_ack(awlink_s * link,uint8_t ack,uint8_t i_id,uint8_t si_id)
{
	awlink_msg_s msg;
	awlink_system_ack_s data;

	data.ack = ack;
	data.item_id = i_id;
	data.subitem_id = si_id;

	msg.item_id = AWLINK_ITEM_SYSTEM;
	msg.subitem_id = AWLINK_ITEM_SYSTEM_ACK;
	msg.data = (uint8_t *)&data;
	msg.length = sizeof(data);

	awlink_encode(link,&msg,false,false);
}

void awlink_encode_system_awlink_ver(awlink_s * link,awlink_msg_s * msg_rev)
{
	awlink_msg_s msg;
	awlink_system_awlink_ver_s data;
	uint16_t ver[4] = {0};
#ifdef VER
	ver[0] = VER;
#endif
	ver[1] = flow_get_version();

	msg.item_id = AWLINK_ITEM_SYSTEM;
	msg.subitem_id = AWLINK_ITEM_SYSTEM_AWLINK_VER;
	msg.data = (uint8_t *)&ver;
	msg.length = sizeof(data);

	awlink_encode(link,&msg,false,false);
}

void awlink_encode_system_awpilot_ver(awlink_s * link,awlink_msg_s * msg_rev)
{
	awlink_msg_s msg;
	awlink_system_awpilot_ver_s data;

#ifdef VER
	snprintf((char *)data.version,20,"%d.%d.%d",VER/100%10,VER/10%10,VER/1%10);
#else
	snprintf((char *)data.version,20,"%d.%d.%d",0,0,0);
#endif

	msg.item_id = AWLINK_ITEM_SYSTEM;
	msg.subitem_id = AWLINK_ITEM_SYSTEM_AWPILOT_VER;
	msg.data = (uint8_t *)&data;
	msg.length = sizeof(data);

	awlink_encode(link,&msg,false,false);
}

void awlink_encode_system_heart(awlink_s * link,uint8_t i)
{
	awlink_msg_s msg;
	awlink_system_heart_s data;

	data.heart= i;

	msg.item_id = AWLINK_ITEM_SYSTEM;
	msg.subitem_id = AWLINK_ITEM_SYSTEM_HEART;
	msg.data = (uint8_t *)&data;
	msg.length = sizeof(data);

	awlink_encode(link,&msg,false,false);
}

void awlink_decode_system_heart(awlink_s * link,awlink_msg_s * msg_rev)
{
	//awlink_system_heart_s * data;
	//data = (awlink_system_heart_s *)msg_rev->data;
	
	system_set_awlink_heart_count();
}

void awlink_handle_system(awlink_s * link,awlink_msg_s * msg)
{
	switch(msg->subitem_id){
		case AWLINK_ITEM_SYSTEM_ACK:
			break;
		case AWLINK_ITEM_SYSTEM_AWLINK_VER:
			awlink_encode_system_awlink_ver(link,msg);
			break;
		case AWLINK_ITEM_SYSTEM_AWPILOT_VER:
			awlink_encode_system_awpilot_ver(link,msg);
			break;
		case AWLINK_ITEM_SYSTEM_HEART:
			awlink_decode_system_heart(link,msg);
			break;
	}
}


