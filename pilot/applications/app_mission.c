#include <sys/stat.h>

#include "app_mission.h"
#include "app_debug.h"
#include "app_control.h"

#include "hal.h"
#include "param.h"

#include "disk_flush.h"

#define DEBUG_ID DEBUG_ID_MISSION

#define MISSION_FILE_PATH			"/netPrivate/mission"

#define MISSION_MAGIC 0xEE
#define MISSION_DECODE_STEP_MAGIC		1
#define MISSION_DECODE_STEP_LENGTH		2
#define MISSION_DECODE_STEP_TYPE		3
#define MISSION_DECODE_STEP_DATA		4
#define MISSION_DECODE_STEP_CHECKSUM1	5
#define MISSION_DECODE_STEP_CHECKSUM2	6
#define MISSION_DECODE_STEP_OK			7
#define MISSION_DECODE_STEP_FAIL		8

typedef struct mission_node_s{
	struct mission_node_s * prev;
	struct mission_node_s * next;
	uint16_t count;
	uint8_t len;
	uint8_t type;
	uint8_t * data;
}mission_node_s;

typedef struct mission_item_s{
	uint8_t magic;
	uint8_t len;
	uint8_t type;
	uint8_t * data;
	uint16_t crc;
}mission_item_s;

mission_node_s mission_start;

mission_item_s mission_decode_item;
uint8_t mission_decode_step;
uint8_t mission_decode_data_count;
uint16_t mission_decode_crc;

int mission_file_fd = -1;
uint16_t mission_item_count;

bool mission_run_done;
uint8_t mission_run_type;
uint8_t mission_run_status;
uint16_t mission_run_count;

struct timespec mission_time_start;
float mission_time_diff;

void mission_add_node(uint16_t count,mission_item_s * item)
{
	mission_node_s * node = NULL;
	mission_node_s * node_curr = NULL;
	mission_node_s * node_add = NULL;
	bool node_count_check = true;

	if(count > 0){
		node = (mission_node_s *)malloc(sizeof(mission_node_s));
		node->len = item->len;
		node->type = item->type;
		node->count = count;
		node->data = malloc(node->len);
		memcpy(node->data,item->data,node->len);
				
		if(mission_start.count > 0){
			node_curr = mission_start.next;
			while(node_curr != NULL && count >= node_curr->count){
				if(node_curr->count == count){
					node_count_check = false;
					break;
				}
				node_add = node_curr;
				node_curr = node_curr->next;
			}
			
			if(node_count_check == true){
				node->prev = node_add;
				node->next = node_add->next;
			
				if(node->next != NULL){
					node->next->prev = node;
				}
				node_add->next = node;
				
				mission_start.count++;
			}
		}else{
			node->prev = mission_start.prev;
			node->next = NULL;
			mission_start.prev->next = node;
			mission_start.prev = node;
			mission_start.count++;
		}
	}
}

void mission_add_node_last(mission_item_s * item)
{
	mission_node_s * node;
	node = (mission_node_s *)malloc(sizeof(mission_node_s));
	node->len = item->len;
	node->type = item->type;
	node->count = mission_start.count + 1;
	node->data = malloc(node->len);
	memcpy(node->data,item->data,node->len);

	node->prev = mission_start.prev;
	node->next = NULL;
	mission_start.prev->next = node;
	mission_start.prev = node;
	mission_start.count++;
}

void mission_list_debug()
{
	mission_node_s * node;
	mission_control_s * control;

	if(mission_start.count == 0){
		DEBUG(DEBUG_ID,"mission null");
	}else{
		node = mission_start.next;
		DEBUG(DEBUG_ID,"mission list:%d",mission_start.count);
		while(node != NULL){
			DEBUG(DEBUG_ID,"mission %d(%d)",node->count,node->type);

			if(node->data != NULL){
				switch(node->type){
					case MISSION_TYPE_CONTROL:
						control = (mission_control_s *)node->data;
						DEBUG(DEBUG_ID,"control:%d (%3.3f,%3.3f,%3.3f,%3.3f,%3.3f,%3.3f,%3.3f,%3.3f)",control->mode,control->param1,control->param2,control->param3,control->param4,control->param5,control->param6,control->param7,control->param8);
						break;
				}
			}
			
			node = node->next;
		}
		DEBUG(DEBUG_ID,"mission list end");
	}
}

void mission_decode_process(uint8_t * data , mission_item_s * msg)
{
	switch(mission_decode_step){
		case MISSION_DECODE_STEP_MAGIC:
			if(*data == MISSION_MAGIC){
				mission_decode_crc = crc16_init();
				mission_decode_step = MISSION_DECODE_STEP_LENGTH;
			}
			break;
		case MISSION_DECODE_STEP_LENGTH:
			mission_decode_step = MISSION_DECODE_STEP_TYPE;
			mission_decode_crc = crc16_update(*data,mission_decode_crc);
			mission_decode_data_count = 0;
			msg->len = *data;
			msg->data = malloc(msg->len);
			if(msg->data == NULL){
				mission_decode_step = MISSION_DECODE_STEP_FAIL;
			}
			break;
		case MISSION_DECODE_STEP_TYPE:
			mission_decode_step = MISSION_DECODE_STEP_DATA;
			mission_decode_crc = crc16_update(*data,mission_decode_crc);
			msg->type = *data;
			break;
		case MISSION_DECODE_STEP_DATA:
			mission_decode_crc = crc16_update(*data,mission_decode_crc);
			if(msg->data != NULL){
				msg->data[mission_decode_data_count] = *data;
			}
			mission_decode_data_count++;
			if(mission_decode_data_count >= msg->len){
				mission_decode_step = MISSION_DECODE_STEP_CHECKSUM1;
			}
			break;
		case MISSION_DECODE_STEP_CHECKSUM1:
			if(*data == (mission_decode_crc & 0xFF)){
				mission_decode_step = MISSION_DECODE_STEP_CHECKSUM2;
			}else{
				mission_decode_step = MISSION_DECODE_STEP_FAIL;
			}
			break;
		case MISSION_DECODE_STEP_CHECKSUM2:
			if(*data == ((mission_decode_crc >> 8) & 0xFF)){
				mission_decode_step = MISSION_DECODE_STEP_OK;
			}else{
				mission_decode_step = MISSION_DECODE_STEP_FAIL;
			}
			break;
	}
}

void mission_decode(uint8_t * data,int len)
{
	int count;
	
	if(len == 0){
		return ;
	}
	
	for(count = 0 ; count < len ; count++){
		mission_decode_process(&data[count],&mission_decode_item);

		if(mission_decode_step == MISSION_DECODE_STEP_OK){
			mission_decode_step = MISSION_DECODE_STEP_MAGIC;
			mission_add_node_last(&mission_decode_item);
			if(mission_decode_item.data != NULL){
				free(mission_decode_item.data);
			}
		}else if(mission_decode_step == MISSION_DECODE_STEP_FAIL){
			mission_decode_step = MISSION_DECODE_STEP_MAGIC;
			if(mission_decode_item.data != NULL){
				free(mission_decode_item.data);
			}
		}	
	}
}

bool mission_load_file(int fd)
{
	uint8_t data[20];
	int len_sum = 0;
	int len = 0;
	struct stat fileStat;  

	if(fstat(fd, &fileStat) == -1){
		ERR(DEBUG_ID,"fstat failed");
		return false;	
	}  

	while(len_sum < fileStat.st_size){
		len = read(fd,data,20);
		if(len < 0){
			return false;
		}
		
		len_sum += len;
		mission_decode(data,len);
	}

	return true;
}

void mission_save_file(int fd)
{
	uint8_t * buf;
	int count;
	uint16_t crc;
	mission_node_s * node;

	if(mission_start.count > 0){
		node = mission_start.next;
		while(node != NULL){
			crc = crc16_init();
			crc = crc16_update(node->len,crc);
			crc = crc16_update(node->type,crc);
			for(count = 0 ; count < node->len ; count++){
				crc = crc16_update(node->data[count],crc);
			}

			buf = (uint8_t *)malloc(node->len + 5);
			if(buf != NULL){
				buf[0] = MISSION_MAGIC;
				buf[1] = node->len;
				buf[2] = node->type;
				for(count = 0 ; count < node->len ; count++){
					buf[3 + count] = node->data[count];
				}
				buf[3 + node->len] = crc & 0xFF;
				buf[4 + node->len] = (crc >> 8) & 0xFF;
				
				write(fd,buf,node->len + 5);
				free(buf);
			}
			
			node = node->next;
		}
	}
}

void mission_save(void)
{
	if(mission_file_fd < 0){
		mission_file_fd = open(MISSION_FILE_PATH,O_RDWR|O_CREAT|O_TRUNC,0);
	}

	if(mission_file_fd > 0){
		mission_save_file(mission_file_fd);
		close(mission_file_fd);
		data_flush_to_disk();
		mission_file_fd = -1;
		INFO(DEBUG_ID,"mission save done");
	}else{
		ERR(DEBUG_ID,"mission save failed,can't open file");
	}
}

void mission_load(void)
{
	if(mission_file_fd < 0){
		mission_file_fd = open(MISSION_FILE_PATH,O_RDWR,0);
	}
	
	if(mission_file_fd > 0){
		mission_load_file(mission_file_fd);
		close(mission_file_fd);
		mission_file_fd = -1;		
		INFO(DEBUG_ID,"mission load done");
	}else{
		INFO(DEBUG_ID,"mission load failed");
	}

	mission_list_debug();
}

void mission_add_item(uint16_t count,uint8_t type,uint8_t len,uint8_t * data)
{
	mission_item_s item;

	item.type = type;
	item.len = len;
	item.data = data;
	
	mission_add_node(count,&item);
}

bool mission_get_item_data(uint16_t count,uint8_t * data,uint8_t len)
{
	mission_node_s * mission_curr;

	if(count > mission_start.count && count == 0){
		return false;
	}

	mission_curr = mission_start.next;
	while(mission_curr != NULL && mission_curr->count != count){
		mission_curr = mission_curr->next;
	}

	if(mission_curr != NULL && mission_curr->data != NULL){
		if(len >= mission_curr->len){
			memcpy(data,mission_curr->data,mission_curr->len);
			return true;
		}
	}

	return false;
}

int mission_get_item_type(uint16_t count)
{
	mission_node_s * mission_curr;

	if(count > mission_start.count && count == 0){
		return -1;
	}

	mission_curr = mission_start.next;
	while(mission_curr != NULL && mission_curr->count != count){
		mission_curr = mission_curr->next;
	}

	if(mission_curr != NULL){
		return mission_curr->type;
	}else{
		return -1;
	}
}

void mission_clear(void)
{
	mission_node_s * node;
	mission_node_s * node_clear;

	if(mission_start.count != 0){
		node = mission_start.next;
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

	mission_start.type = 0;
	mission_start.len = 0;
	mission_start.count = 0;
	mission_start.data = NULL;
	mission_start.prev = &mission_start;
	mission_start.next = NULL;
}

void mission_set_done()
{
	mission_run_done = true;
}

uint16_t mission_get_total(void)
{
	return mission_start.count;
}

uint8_t mission_get_run_type()
{
	return mission_run_type;
}

uint16_t mission_get_run_count()
{
	return mission_run_count;
}

uint8_t mission_get_run_status()
{
	return mission_run_status;
}

void mission_set_run(uint8_t status)
{
	if(mission_run_status == MISSION_RUN_STATUS_STOP && status == MISSION_RUN_STATUS_RUN){
		mission_run_count = 0;
		mission_run_done = true;
		get_diff_time(&mission_time_start,true);
	}
	
	mission_run_status = status;

	INFO(DEBUG_ID,"status:%d",mission_run_status);
}

void mission_clear_time()
{
	get_diff_time(&mission_time_start,true);
	get_diff_time(&mission_time_start,true);
}

float mission_get_time()
{
	return mission_time_diff;
}

void mission_init(void)
{
	mission_decode_step = MISSION_DECODE_STEP_MAGIC;

	mission_start.type = 0;
	mission_start.len = 0;
	mission_start.count = 0;
	mission_start.data = NULL;
	mission_start.prev = &mission_start;
	mission_start.next = NULL;

	mission_run_status = MISSION_RUN_STATUS_STOP;
	
	mission_load();
	
	INFO(DEBUG_ID,"init");
}

void mission_update(float dt)
{
	mission_control_s control;
	mission_time_diff = get_diff_time(&mission_time_start,false);

	if(mission_run_status == MISSION_RUN_STATUS_RUN && mission_run_done == true){
		mission_run_count++;
		mission_run_done = false;

		if(mission_run_count <= mission_start.count){
			mission_run_type = mission_get_item_type(mission_run_count);
			INFO(DEBUG_ID,"mission:(%d,%d) type:%d",mission_run_count,mission_start.count,mission_run_type);
			switch(mission_run_type){
				case MISSION_TYPE_CONTROL:
					mission_get_item_data(mission_run_count,(uint8_t *)&control,sizeof(mission_control_s));
					//INFO(DEBUG_ID,"control:%d (%3.3f,%3.3f,%3.3f,%3.3f,%3.3f,%3.3f,%3.3f,%3.3f)",control.mode,control.param1,control.param2,control.param3,control.param4,control.param5,control.param6,control.param7,control.param8);
					INFO(DEBUG_ID,"control:%d (%3.3f,%3.3f,%3.3f,%3.3f)",control.mode,control.param1,control.param2,control.param3,control.param4);
					//control_set_mode_long(control.mode,control.param1,control.param2,control.param3,control.param4,control.param5,control.param6,control.param7,control.param8);
					break;
			}
		}else{
			mission_run_status = MISSION_RUN_STATUS_STOP;
			control_set_normal_mode();
			INFO(DEBUG_ID,"mission done:(%d,%d)",mission_run_count,mission_start.count);
		}
	}
}


