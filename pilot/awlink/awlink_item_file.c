#include "app_debug.h"
#include "app_system.h"

#include "awlink.h"
#include "awlink_encode.h"
#include "awlink_item_system.h"


#define DEBUG_ID DEBUG_ID_LINK

#define AWLINK_ITEM_FILE_CONTROL_NULL	0
#define AWLINK_ITEM_FILE_CONTROL_STOP	1

#define AWLINK_ITEM_FILE_STATUS_STOP	0
#define AWLINK_ITEM_FILE_STATUS_RUN		1
#define AWLINK_ITEM_FILE_STATUS_OK		2
#define AWLINK_ITEM_FILE_STATUS_FAIL	3

#define FILE_DATA_LEN 200

typedef struct PACKED{
	uint8_t control;
	char path[FILE_PATH_LEN];
	uint16_t total;
}awlink_file_control_s;

typedef struct PACKED{
	uint16_t count;
	uint8_t data[FILE_DATA_LEN];
}awlink_file_item_s;

typedef struct PACKED{
	uint16_t count;
	uint8_t status;
}awlink_file_status_s;

bool awlink_file_write(awlink_file_s * file)
{
	int count = 0;
	file_node_s * node;
	int fd = -1;

	remove(file->path);
	fd = open(file->path,O_RDWR | O_CREAT | O_TRUNC);
	if(fd > 0){
		if(file->start_node.count > 0 && file->start_node.next != NULL){
			node = file->start_node.next;
		
			for(count = 0 ; count < file->start_node.count ; count++){
				if(node->data != NULL){
					write(fd,node->data,FILE_DATA_LEN);
				}
		
				if(node->next != NULL){
					node = node->next;
				}
			}
		}
		close(fd);
		return true;
	}else{
		return false;
	}
}

void awlink_file_clear(awlink_file_s * file)
{
	file_node_s * node;
	file_node_s * node_clear;

	if(file->start_node.count != 0){
		node = file->start_node.next;
		while(node != NULL){
			node_clear = node;
			node = node->next;
			
			if(node_clear->data != NULL){
				free(node_clear->data);
			}

			if(node_clear != NULL){
				free(node_clear);
			}
		}
	}

	file->start_node.count = 0;
	file->start_node.data = NULL;
	file->start_node.prev = &file->start_node;
	file->start_node.next = NULL;
}

void awlink_encode_file_status(awlink_s * link,uint8_t status)
{
	awlink_msg_s msg;
	awlink_file_status_s data;

	data.status = status;
	data.count = link->file.count;

	msg.item_id = AWLINK_ITEM_FILE;
	msg.subitem_id = AWLINK_ITEM_FILE_STATUS;
	msg.data = (uint8_t *)&data;
	msg.length = sizeof(data);

	//DEBUG(DEBUG_ID,"awlink_encode_file_status:%d",status);

	awlink_encode(link,&msg,false,true);
}

void awlink_decode_file_item(awlink_s * link,awlink_msg_s * msg_rev)
{
	int count = 0;
	file_node_s * node;
	awlink_file_item_s * data;
	data = (awlink_file_item_s *)msg_rev->data;

	//INFO(DEBUG_ID,"awlink_decode_file_item:%d,%d,%d",data->count,link->file.count,link->file.start_node.count);

	if(link->file.stream_run == true && link->file.start_node.count > 0 && link->file.start_node.next != NULL){
		node = link->file.start_node.next;

		for(count = 0 ; count < link->file.start_node.count ; count++){
			//INFO(DEBUG_ID,"awlink item:%d,%d",node->count,data->count);
			if(node->count == data->count && node->data != NULL){
				memcpy(node->data,data->data,FILE_DATA_LEN);
				link->file.time = 0.0f;
				link->file.count++;
				//INFO(DEBUG_ID,"awlink item hit:%d",data->count);
				return;
			}

			if(node->next != NULL){
				node = node->next;
			}
		}
	}
}

void awlink_decode_file_control(awlink_s * link,awlink_msg_s * msg_rev)
{
	int count = 0;
	awlink_file_control_s * data;
	data = (awlink_file_control_s *)msg_rev->data;

	//INFO(DEBUG_ID,"awlink_decode_file_control:(%s),%d,%d",data->path,data->total,data->control);

	memcpy(link->file.path,data->path,FILE_PATH_LEN);

	link->file.control= data->control;
	
	link->file.start_node.count = data->total;
	link->file.start_node.data = NULL;
	link->file.start_node.prev = &link->file.start_node;
	link->file.start_node.next = NULL;

	for(count = 0 ; count < link->file.start_node.count ; count++){
		file_node_s * node;
		node = (file_node_s *)malloc(sizeof(file_node_s));
		node->count = count + 1;
		node->data = malloc(FILE_DATA_LEN);		
		
		node->prev = link->file.start_node.prev;
		node->next = NULL;
		link->file.start_node.prev->next = node;
		link->file.start_node.prev = node;
	}

	link->file.time_alive = 0.0f;
	link->file.time = 0.0f;
	link->file.stream_run = true;
	link->file.count = 0;
}

void awlink_handle_file(awlink_s * link,awlink_msg_s * msg)
{
	switch(msg->subitem_id){
		case AWLINK_ITEM_FILE_CONTROL:
			awlink_decode_file_control(link,msg);
			break;
		case AWLINK_ITEM_FILE_ITEM:
			awlink_decode_file_item(link,msg);
			break;
	}
}

void awlink_stream_file(float dt,awlink_s * link)
{
	float file_timeout = 1.0f;
	
	if(link->file.stream_run == true){
		link->file.time += dt;
		link->file.time_alive += dt;

		if(link->file.time_alive >= 0.2f){
			link->file.time_alive = 0.0f;
			awlink_encode_file_status(link,AWLINK_ITEM_FILE_STATUS_RUN);
		}
		
		if(link->file.time >= file_timeout){
			link->file.stream_run = false;
			awlink_file_clear(&link->file);
			awlink_encode_file_status(link,AWLINK_ITEM_FILE_STATUS_FAIL);
			INFO(DEBUG_ID,"awlink file fail");
		}else{
			if(link->file.start_node.count == link->file.count){
				if(awlink_file_write(&link->file) == true){
					INFO(DEBUG_ID,"awlink file done %d,%d",link->file.count,link->file.start_node.count);
					awlink_encode_file_status(link,AWLINK_ITEM_FILE_STATUS_OK);
					if(link->file.control == AWLINK_ITEM_FILE_CONTROL_STOP){
						system_set_run(false);
					}
				}else{
					INFO(DEBUG_ID,"awlink write file failed %d,%d",link->file.count,link->file.start_node.count);
					awlink_encode_file_status(link,AWLINK_ITEM_FILE_STATUS_FAIL);
				}
				link->file.stream_run = false;
				awlink_file_clear(&link->file);
			}
		}
	}	
}

