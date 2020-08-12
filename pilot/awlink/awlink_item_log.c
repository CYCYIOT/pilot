#include "app_param.h"
#include "app_debug.h"
#include "app_log.h"

#include "awlink.h"
#include "awlink_encode.h"

#define DEBUG_ID DEBUG_ID_LINK

#define LOG_ITEM_DATA_LENGTH 230

typedef struct PACKED{
	uint8_t control;
}awlink_log_control_s;

typedef struct PACKED{
	int32_t id;
}awlink_log_get_s;

typedef struct PACKED{
	int32_t id_max;
	int32_t id;
	uint8_t length;
	uint8_t data[LOG_ITEM_DATA_LENGTH];
}awlink_log_item_s;

int32_t awlink_stream_log_readfile(FILE* fd,uint8_t * buf,uint8_t len);

void awlink_encode_log_item(awlink_s * link,uint32_t id_max,uint32_t id,uint8_t * buf,uint32_t len,bool need_buf)
{
	awlink_msg_s msg;
	awlink_log_item_s data;

	data.id_max = id_max;
	data.id = id;
	data.length = len;
	memcpy(data.data,buf,len);

	msg.item_id = AWLINK_ITEM_LOG;
	msg.subitem_id = AWLINK_ITEM_LOG_ITEM;
	msg.data = (uint8_t *)&data;
	msg.length = sizeof(data);

	awlink_encode(link,&msg,need_buf,true);
}

void awlink_decode_log_control(awlink_s * link,awlink_msg_s * msg_rev)
{
	awlink_log_control_s * data;
	data = (awlink_log_control_s *)msg_rev->data;

	if(data->control == 0){
		log_set_restart();
	}
}

void awlink_decode_log_get(awlink_s * link,awlink_msg_s * msg_rev)
{
	awlink_log_get_s * data;
	data = (awlink_log_get_s *)msg_rev->data;

	if(data->id == 0xFFFFFFFF){
		DEBUG(DEBUG_ID,"send log file");
		if(link->log.fd == NULL && link->log.stream_run == false){
			link->log.fd = fopen(LOG_PATH,"r");
			if(link->log.fd != NULL){
				struct stat st;
				if(stat(LOG_PATH,&st) == 0){
					link->log.num = st.st_size / LOG_ITEM_DATA_LENGTH + 1;
					link->log.stream_run = true;
					link->log.count = 1;
					DEBUG(DEBUG_ID,"open log ok:%s (%d,%jd)",LOG_PATH,link->log.num,st.st_size);
				}else{
				}
			}else{
				INFO(DEBUG_ID,"open log(%s) failed",LOG_PATH);
			}
		}
	}else{
		link->log.item_fd = fopen(LOG_PATH,"r");
		if(link->log.item_fd != NULL){
			uint8_t buf[LOG_ITEM_DATA_LENGTH];
			int32_t seek_ret;
			int32_t ret;
			long offset = (data->id - 1) * LOG_ITEM_DATA_LENGTH;
			memset(buf,0,LOG_ITEM_DATA_LENGTH);
			seek_ret = fseek(link->log.item_fd,offset, SEEK_SET);
			ret = awlink_stream_log_readfile(link->log.item_fd,buf,LOG_ITEM_DATA_LENGTH);
			if(seek_ret == 0 && ret > 0){
				awlink_encode_log_item(link,link->log.num,data->id,buf,ret,false);
			}
			
			fclose(link->log.item_fd);
		}
	}
}

void awlink_handle_log(awlink_s * link,awlink_msg_s * msg)
{
	switch(msg->subitem_id){
		case AWLINK_ITEM_LOG_GET:
			awlink_decode_log_get(link,msg);
			break;
		case AWLINK_ITEM_LOG_CONTROL:
			awlink_decode_log_control(link,msg);
			break;
	}
}

int32_t awlink_stream_log_readfile(FILE* fd,uint8_t * buf,uint8_t len)
{
	if(fd != NULL){
		return fread(buf,sizeof(uint8_t),len,fd);
	}else{
		return -1;
	}
}

void awlink_stream_log(awlink_s * link)
{
	int32_t ret = 1;
	uint8_t buf[LOG_ITEM_DATA_LENGTH];
	uint8_t count = 5;
	
	if(link->log.stream_run == true){
		if(link->log.count <= link->log.num){
			while(count > 0 && ret > 0 && link->log.count <= link->log.num){
				memset(buf,0,LOG_ITEM_DATA_LENGTH);
				ret = awlink_stream_log_readfile(link->log.fd,buf,LOG_ITEM_DATA_LENGTH);
				if(ret > 0){
					//DEBUG(DEBUG_ID,"send log file %d,%d,%d",ret,link->log.num,link->log.count);
					awlink_encode_log_item(link,link->log.num,link->log.count,buf,ret,true);
				}
				link->log.count++;
				count--;
			}
		}else if(link->log.count >= link->log.num){
			fclose(link->log.fd);
			link->log.fd = NULL;
			link->log.stream_run = false;
			link->log.count = 1;
			awlink_encode_log_item(link,link->log.num,-1,buf,ret,false);
			DEBUG(DEBUG_ID,"send log file done");
		}
	}
}

