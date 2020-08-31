#include "app_awlink.h"

#include "awlink_network.h"
#include "awlink_stream.h"
#include "awlink_item_system.h"
#include "awlink_protocol.h"

//#define AWLINK_SUB_LINK 1

awlink_s awlink_main;
#ifdef AWLINK_SUB_LINK
awlink_s awlink_sub;
#endif

void awlink_msg(uint8_t level,uint8_t * buf)
{
	awlink_encode_system_msg(&awlink_main,level,buf);  //¨¨o??2a¨º?
#ifdef AWLINK_SUB_LINK
	awlink_encode_system_msg(&awlink_sub,level,buf);
#endif
}

void awlink_heart()
{
	awlink_encode_system_heart(&awlink_main,0);
#ifdef AWLINK_SUB_LINK
	awlink_encode_system_heart(&awlink_sub,0);
#endif
}

void awlink_link_init(awlink_s * link,int tcp_port,int udp_port)
{
	int count;
	
	link->net.udp_port = udp_port;
	link->net.tcp_port = tcp_port;
	link->net.udp_socket_fd = -1;
	link->net.udp_addr_len = 0;
	link->net.udp_link_connect = false;
	link->net.tcp_max_fd = -1;
	link->net.tcp_socket_fd = -1;
	link->net.tcp_client_fd = -1;
	link->net.send_buf_count = 0;

	link->file.stream_run = false;

	link->log.stream_run = false;
	link->log.num = 0;
	link->log.count = 1;
	link->log.fd = NULL;
	link->log.item_fd = NULL;

	link->mission.stream_run = false;
	link->mission.total = 0;
	link->mission.count = 1;

	link->param.stream_run = false;
	link->param.total = 0;
	link->param.count = 1;

	for(count = 0 ; count < AWLINK_ITEM_STATUS_NUM ; count++){
		link->stream.rate[count] = 0;
		link->stream.time[count] = 0;
	}

	awlink_network_init(&link->net);
	awlink_stream_init(link);
}

void awlink_init()
{
	awlink_main.control_enable = true;
	awlink_link_init(&awlink_main,9697,9696);
	
#ifdef AWLINK_SUB_LINK
	awlink_sub.control_enable = false;
	awlink_link_init(&awlink_sub,9699,9698);
#endif
}

void awlink_exit()
{
	awlink_network_exit(&awlink_main.net);
#ifdef AWLINK_SUB_LINK
	awlink_network_exit(&awlink_sub.net);
#endif
}

void awlink_update(float dt)
{
	awlink_network_update(dt,&awlink_main);
	awlink_stream_update(dt,&awlink_main);

#ifdef AWLINK_SUB_LINK
	awlink_network_update(dt,&awlink_sub);
	awlink_stream_update(dt,&awlink_sub);
#endif
}

void awlink_aruco_send()
{

 awlink_encode_system_aruco_ack(&awlink_main,AWLINK_ACK_OK,AWLINK_ITEM_CONTROL,AWLINK_ITEM_CONTROL_ARUCO_DIST);


}
void awlink_shooting_send_tcp()
{
 awlink_encode_system_shooting(&awlink_main,AWLINK_ACK_OK,AWLINK_ITEM_CONTROL,AWLINK_ITEM_CONTROL_SHOOTING);

}
void recv_uart(uint8_t *recv_buf,int len)
{
  awlink_network_s * net = &awlink_main.net;

  awlink_decode(&awlink_main,&net->udp_decoder,recv_buf,len);
}

