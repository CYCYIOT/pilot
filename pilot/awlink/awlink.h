#ifndef _AWLINK_H_
#define _AWLINK_H_

#include <stdio.h>
#include <fcntl.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <fcntl.h>

#include "awlink_protocol.h"

#define DECODE_TYPE_UDP	0
#define DECODE_TYPE_TCP	1

#define FILE_PATH_LEN 80

typedef struct{
	float rate[AWLINK_ITEM_STATUS_NUM];
	float time[AWLINK_ITEM_STATUS_NUM];
}awlink_stream_s;

typedef struct{
	bool stream_run;
	uint16_t total;
	uint16_t count;
}awlink_param_s;

typedef struct{
	bool stream_run;
	uint16_t total;
	uint16_t count;
}awlink_mission_s;

typedef struct{
	bool stream_run;
	int32_t num;
	int32_t count;
	FILE * fd;
	FILE * item_fd;
}awlink_log_s;

typedef struct file_node_s{
	struct file_node_s * prev;
	struct file_node_s * next;
	uint16_t count;
	uint8_t * data;
}file_node_s;

typedef struct{
	file_node_s start_node;
	uint8_t control;
	char path[FILE_PATH_LEN];
	bool stream_run;
	float time;
	float time_alive;
	uint16_t count;
}awlink_file_s;

typedef struct{
	uint8_t magic;
	uint8_t length;
	uint8_t id_src;
	uint8_t item_id;
	uint8_t subitem_id;
	uint8_t * data;
	uint16_t checksum;
}awlink_msg_s;

typedef struct{
	uint8_t type;
	uint16_t awlink_parse_error;
	uint8_t awlink_parse_step;
	uint8_t awlink_parse_data_count;
	uint16_t awlink_parse_checksum;
	awlink_msg_s awlink_package;
}awlink_decode_s;

typedef struct{
	int udp_port;
	int tcp_port;
	
	struct sockaddr_in udp_addr_send;
	struct sockaddr_in udp_addr;
	int udp_socket_fd;
	int udp_addr_len;
	bool udp_link_connect;
	
	int tcp_max_fd;
	int tcp_socket_fd;
	int tcp_client_fd;
	struct sockaddr_in tcp_addr;
	struct sockaddr_in tcp_client;
	struct timeval tcp_timeout;
	fd_set tcp_set;
	
	uint8_t udp_recv_buf[1000];
	uint8_t tcp_recv_buf[1000];

	uint8_t send_buf[256 * 6];
	uint32_t send_buf_count;
	
	awlink_decode_s udp_decoder;
	awlink_decode_s tcp_decoder;
}awlink_network_s;


typedef struct{
	awlink_network_s net;
	awlink_file_s file;
	awlink_log_s log;
	awlink_mission_s mission;
	awlink_param_s param;
	awlink_stream_s stream;

	bool control_enable;
}awlink_s;


#endif

