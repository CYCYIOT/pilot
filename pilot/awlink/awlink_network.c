#include "app_system.h"
#include "app_debug.h"

#include "awlink_network.h"
#include "awlink_item_system.h"
#include "awlink_item_status.h"
#include "awlink_item_control.h"
#include "pilot_steam_control.h"
#include "hal_led.h"
#define DEBUG_ID DEBUG_ID_NETWORK
#define LED_BLINK_STOP  1
bool awlink_network_get_connect(awlink_network_s * net)
{
	return net->udp_link_connect;
}

void awlink_socket_recv_udp(awlink_s * link)
{
	awlink_network_s * net = &link->net;
	int len = 0;

	len = recvfrom(net->udp_socket_fd, net->udp_recv_buf, sizeof(net->udp_recv_buf), 0,(struct sockaddr *)&net->udp_addr_send, (socklen_t*)&net->udp_addr_len);	
	if(len > 0){
		if(net->udp_link_connect == false && ntohs(net->udp_addr_send.sin_port) > 0){
			net->udp_link_connect = true;
			
			char addr_p[INET_ADDRSTRLEN];
			inet_ntop(AF_INET,&net->udp_addr_send.sin_addr,addr_p,sizeof(addr_p));
			INFO(DEBUG_ID,"UDP connect:%d(%s)",ntohs(net->udp_addr_send.sin_port),addr_p);
            led_blink(LED_BLINK_STOP);
		}
		
		awlink_decode(link,&net->udp_decoder,net->udp_recv_buf,len);
	}
}

void awlink_network_send_udp(awlink_network_s * net,uint8_t * buff,uint32_t length)
{
	if(net->udp_link_connect == true){
		sendto(net->udp_socket_fd,buff, length, 0,(struct sockaddr *)&net->udp_addr_send,net->udp_addr_len);
	}
}

void awlink_network_send_tcp(awlink_network_s * net,uint8_t * buff,uint32_t length)
{
	if(net->udp_link_connect == true && net->tcp_client_fd > 0){
		//printf("tcp:%d \r\n",length);
		send(net->tcp_client_fd,buff,length,0);
	}
}

void awlink_network_send(awlink_network_s * net,uint8_t * buff,uint32_t length,bool safe)
{
	if(safe == true){
		awlink_network_send_tcp(net,buff,length);
	}else{
		awlink_network_send_udp(net,buff,length);
	}
}

void awlink_network_udp_init(awlink_network_s * net)
{
	int flag = 0;

	if((net->udp_socket_fd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
		char error_info[50];
		snprintf(error_info,50,"UDP:%d socket failed\n",net->udp_port);
		perror(error_info);
		exit(1);
	}
	
	bzero(&net->udp_addr, sizeof(net->udp_addr));
	net->udp_addr.sin_family = AF_INET;
	net->udp_addr.sin_port = htons(net->udp_port);
	net->udp_addr.sin_addr.s_addr = htonl(INADDR_ANY) ;
	flag = fcntl(net->udp_socket_fd , F_GETFL , 0);
	fcntl(net->udp_socket_fd,F_SETFL,flag | O_NONBLOCK);
	
	if(bind(net->udp_socket_fd, (struct sockaddr *)&net->udp_addr, sizeof(net->udp_addr))<0){
		char error_info[50];
		snprintf(error_info,50,"UDP:%d connect failed\n",net->udp_port);
		perror(error_info);
		exit(1);
	}else{
		INFO(DEBUG_ID,"UDP:%d init ok",net->udp_port);	
	}
}

void awlink_socket_recv_tcp(awlink_s * link)
{
	awlink_network_s * net = &link->net;
	int len = -1;
	int ret = -1;

	FD_ZERO(&net->tcp_set);
	FD_SET(net->tcp_socket_fd, &net->tcp_set);
	if(net->tcp_client_fd >= 0){
		FD_SET(net->tcp_client_fd, &net->tcp_set);
	}

	ret = select(net->tcp_max_fd + 1, &net->tcp_set, NULL, NULL, &net->tcp_timeout);
	switch (ret) {	
		case -1:  
			INFO(DEBUG_ID,"TCP:%d select error",net->tcp_port);	
			break;
		case 0:  
			break;  
		default:  
			if(FD_ISSET(net->tcp_socket_fd, &net->tcp_set)){
				bzero(&net->tcp_client, sizeof(net->tcp_client));  
				size_t len = sizeof(net->tcp_client);  
				net->tcp_client_fd = accept(net->tcp_socket_fd,(struct sockaddr *) &net->tcp_client, &len);  
				if (net->tcp_client_fd >= 0) {  
					if(net->tcp_max_fd < net->tcp_client_fd) {  
						net->tcp_max_fd = net->tcp_client_fd;  
					}
					INFO(DEBUG_ID,"TCP connect:%d(%s)",net->tcp_port,inet_ntoa(net->tcp_client.sin_addr));						
				}
				break;
			} else {
				if(net->tcp_client_fd < 0)  
					break;

				if(!FD_ISSET(net->tcp_client_fd, &net->tcp_set)) {	
					break;  
				}  

				len = recv(net->tcp_client_fd, net->tcp_recv_buf, sizeof(net->tcp_recv_buf), 0);	
				if (len < 0) {	
					//INFO(DEBUG_ID,"TCP:%d recv error",net->tcp_port);	
				}else{
					awlink_decode(link,&net->tcp_decoder,net->tcp_recv_buf,len);
				}
			}  
			break;	
	}  
}

void awlink_network_tcp_init(awlink_network_s * net)
{
	int flag;
	int opt = 1;
	
	net->tcp_socket_fd = socket(AF_INET, SOCK_STREAM, 0);
	if(net->tcp_socket_fd < 0){
		char error_info[50];
		snprintf(error_info,50,"TCP:%d socket failed\n",net->udp_port);
		perror(error_info);
		exit(1);
	}
  
    bzero(&net->tcp_addr, sizeof(net->tcp_addr));  
    net->tcp_addr.sin_family = AF_INET;  
    net->tcp_addr.sin_port = htons(net->tcp_port);  
    net->tcp_addr.sin_addr.s_addr = htonl(INADDR_ANY);  
	flag = fcntl(net->tcp_socket_fd , F_GETFL , 0);
	fcntl(net->tcp_socket_fd,F_SETFL,flag | O_NONBLOCK);

	if(setsockopt(net->tcp_socket_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt)) != 0)
	{           
		char error_info[50];
		snprintf(error_info,50,"TCP:%d SO_REUSEADDR failed\n",net->udp_port);
		perror(error_info);
		exit(1);
	}	
	
	if(bind(net->tcp_socket_fd, (struct sockaddr *)&net->tcp_addr, sizeof(net->tcp_addr)) < 0){
		char error_info[50];
		snprintf(error_info,50,"TCP:%d bind failed\n",net->udp_port);
		perror(error_info);
		exit(1);
	}

	if(listen(net->tcp_socket_fd,1) < 0){
		char error_info[50];
		snprintf(error_info,50,"TCP:%d listen failed\n",net->udp_port);
		perror(error_info);
		exit(1);
	}

	net->tcp_max_fd = net->tcp_socket_fd;
	net->tcp_timeout.tv_sec = 0;  
	net->tcp_timeout.tv_usec = 0;

	INFO(DEBUG_ID,"TCP:%d init ok",net->tcp_port);
}

void awlink_network_init(awlink_network_s * net)
{
	INFO(DEBUG_ID,"init");

	awlink_decode_init(&net->udp_decoder,DECODE_TYPE_UDP);
	awlink_decode_init(&net->tcp_decoder,DECODE_TYPE_TCP);
	
	awlink_network_udp_init(net);
	awlink_network_tcp_init(net);
}

void awlink_network_exit(awlink_network_s * net)
{
	INFO(DEBUG_ID,"exit");

	if(net->tcp_client_fd > 0){
		shutdown(net->tcp_client_fd,2);
		close(net->tcp_client_fd);
	}
	
	close(net->udp_socket_fd);
}

void awlink_network_update(float dt,awlink_s * link)
{
	if(system_get_awlink_online() == false && link->net.tcp_client_fd > 0){
		shutdown(link->net.tcp_client_fd,2);	
		link->net.tcp_client_fd = -1;
		link->net.tcp_max_fd = link->net.tcp_socket_fd;
		INFO(DEBUG_ID,"TCP shutdown");
	}

	awlink_socket_recv_udp(link);
	awlink_socket_recv_tcp(link);
}

