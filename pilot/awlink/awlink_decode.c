#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include "awlink_decode.h"
#include "awlink_protocol.h"
#include "awlink_crc.h"

#include "awlink_item_system.h"
#include "awlink_item_status.h"
#include "awlink_item_control.h"
#include "awlink_item_mission.h"
#include "awlink_item_parameter.h"
#include "awlink_item_log.h"
#include "awlink_item_file.h"

void awlink_handle(awlink_s * link,awlink_msg_s * msg)
{
	switch(msg->item_id){
		case AWLINK_ITEM_SYSTEM:
			awlink_handle_system(link,msg);
			break;
		case AWLINK_ITEM_STATUS:
			awlink_handle_status(link,msg);
			break;
		case AWLINK_ITEM_CONTROL:
			awlink_handle_control(link,msg);
			break;
		case AWLINK_ITEM_MISSION:
			awlink_handle_mission(link,msg);
			break;
		case AWLINK_ITEM_PARAM:
			awlink_handle_parameter(link,msg);
			break;
		case AWLINK_ITEM_LOG:
			awlink_handle_log(link,msg);
			break;	
		case AWLINK_ITEM_FILE:
			awlink_handle_file(link,msg);
			break;	
	}
}

bool awlink_decode_step(awlink_decode_s * decoder,uint8_t * data)
{
	bool ret = false;

	switch(decoder->awlink_parse_step){
		case AWLINK_PARSE_STEP_MAGIC:
			if(*data == AWLINK_MAGIC){
				decoder->awlink_parse_checksum = awlink_crc16_init();
				decoder->awlink_parse_step = AWLINK_PARSE_STEP_LENGTH;
				decoder->awlink_package.checksum = *data;
				ret = true;
			}
			break;
		case AWLINK_PARSE_STEP_LENGTH:
			decoder->awlink_parse_step = AWLINK_PARSE_STEP_ID_SRC;
			decoder->awlink_parse_checksum = awlink_crc16_update(*data,decoder->awlink_parse_checksum);
			decoder->awlink_package.length = *data;
			ret = true;
			break;
		case AWLINK_PARSE_STEP_ID_SRC:
			decoder->awlink_parse_step = AWLINK_PARSE_STEP_I_ID;
			decoder->awlink_parse_checksum = awlink_crc16_update(*data,decoder->awlink_parse_checksum);
			decoder->awlink_package.id_src = *data;
			ret = true;
			break;
		case AWLINK_PARSE_STEP_I_ID:
			decoder->awlink_parse_step = AWLINK_PARSE_STEP_SI_ID;
			decoder->awlink_parse_checksum = awlink_crc16_update(*data,decoder->awlink_parse_checksum);
			decoder->awlink_package.item_id = *data;
			ret = true;
			break;
		case AWLINK_PARSE_STEP_SI_ID:
			decoder->awlink_parse_step = AWLINK_PARSE_STEP_DATA;
			decoder->awlink_parse_checksum = awlink_crc16_update(*data,decoder->awlink_parse_checksum);
			decoder->awlink_package.subitem_id = *data;
			if(decoder->awlink_package.length == 0){				
				decoder->awlink_parse_step = AWLINK_PARSE_STEP_CHECKSUM1;
			}	
			ret = true;
			break;
		case AWLINK_PARSE_STEP_DATA:
			decoder->awlink_parse_checksum = awlink_crc16_update(*data,decoder->awlink_parse_checksum);
			decoder->awlink_package.data[decoder->awlink_parse_data_count] = *data;
			decoder->awlink_parse_data_count++;
			if(decoder->awlink_parse_data_count >= decoder->awlink_package.length){
				decoder->awlink_parse_step = AWLINK_PARSE_STEP_CHECKSUM1;
				
			}
			ret = true;
			break;
		case AWLINK_PARSE_STEP_CHECKSUM1:
			if(*data == (decoder->awlink_parse_checksum & 0xFF)){
				decoder->awlink_parse_step = AWLINK_PARSE_STEP_CHECKSUM2;
				ret = true;
				
			}	
			break;
		case AWLINK_PARSE_STEP_CHECKSUM2:
			if(*data == ((decoder->awlink_parse_checksum >> 8) & 0xFF)){
				decoder->awlink_parse_step = AWLINK_PARSE_STEP_OK;
				ret = true;
			}
			break;
	}

	return ret;
}

void awlink_package_init(awlink_msg_s * package)
{
	package->magic = 0;
	package->length = 0;
	package->id_src = 0;
	package->item_id = 0;
	package->subitem_id = 0;
	package->data = malloc(250);
	package->checksum = 0;
}

void awlink_decode_init(awlink_decode_s * decoder,uint8_t type)
{
	decoder->type = type;
	decoder->awlink_parse_error = 0;
	decoder->awlink_parse_step = AWLINK_PARSE_STEP_MAGIC;
	decoder->awlink_parse_data_count = 0;
	decoder->awlink_parse_checksum = 0;
	awlink_package_init(&decoder->awlink_package);
}

void awlink_decode(awlink_s * link,awlink_decode_s * decoder,uint8_t * data , int length)
{
	int count = 0;

	if(length == 0){
		return;
	}

	for(count = 0 ; count < length ; count++){
		if(awlink_decode_step(decoder,&data[count]) == false){
			decoder->awlink_parse_error++;
			decoder->awlink_parse_step = AWLINK_PARSE_STEP_MAGIC;
			decoder->awlink_parse_data_count = 0;
			printf("decode error\n");
		}else{
			if(decoder->awlink_parse_step == AWLINK_PARSE_STEP_OK){
				awlink_handle(link,&decoder->awlink_package);
				decoder->awlink_parse_step = AWLINK_PARSE_STEP_MAGIC;
				decoder->awlink_parse_data_count = 0;
			}
		}
	}

}

