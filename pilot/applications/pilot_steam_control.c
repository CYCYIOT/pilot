#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <strings.h>
#include <unistd.h>
#include <signal.h>
#include <limits.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/select.h>
#include <sys/wait.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <ctype.h>
#include <errno.h>
#include <dlfcn.h>

#include <asm/byteorder.h>
#include <time.h>
#include <pthread.h>
#include <sys/un.h>
#include "app_system.h"
#include "app_rc.h"
#include "app_baro.h"
#include "pilot_steam_control.h"
#include "hal_tof_vl53l1x.h"
#include "app_control_takeoff.h"
#include "awlink_item_control.h"
#include "hal_infra_red.h"
#include "hal.h"
#define    tof(str, arg...)  do{\
           time_t timep;\
           struct tm *p;\
           char time_str[40];\
           memset(time_str,'\0',40);\
           time(&timep);\
           p=localtime(&timep);\
           sprintf(time_str,"[%d.%02d.%02d %02d:%02d:%02d]",(1900+p->tm_year),(1+p->tm_mon),p->tm_mday,p->tm_hour,p->tm_min,p->tm_sec);\
           FILE *debug_fp = fopen("/tmp/tof", "a");\
           if (NULL != debug_fp){\
           fprintf(debug_fp, "%s L%d in %s, ",time_str, __LINE__, __FILE__);\
           fprintf(debug_fp, str, ##arg);\
           fflush(debug_fp);\
           fclose(debug_fp);\
           }\
}while(0)

#define STEAM_CONTROL_SERVER  "/tmp/.send_steam_socket"
#define PILOT_AND_IMG_U       "/tmp/.send_img_socket"
#define up   1
#define down 2

int server_sock;
int flag_ac=0;
struct data_control
{
  int id;
  int vel;
};

bool collision_flag=true;
bool wing_protection_flag=true;

bool get_collision_flag()
{	
   return collision_flag;
}
bool get_wing_protecttion_flag()
{
  return  wing_protection_flag;
}
int make_local_server_fd()
{
    int val;
 
    unlink(STEAM_CONTROL_SERVER);

    if(-1 == (server_sock = socket(AF_UNIX, SOCK_DGRAM, 0)) ) 
    {
        tof("Create socket fail. Reason:%s\n", strerror(errno));
        return -1;
    }
    struct sockaddr_un server_addr;
    memset (&(server_addr), '\0', sizeof(struct sockaddr_un));
    server_addr.sun_family = AF_UNIX;  
    memcpy(server_addr.sun_path, STEAM_CONTROL_SERVER, strlen(STEAM_CONTROL_SERVER)); 
    
    val = 1;
    
    if(setsockopt(server_sock, SOL_SOCKET, SO_REUSEADDR, &val, sizeof(val)) < 0) 
    {    
        close(server_sock);
        return -1;
    }
   
    if(-1 == bind(server_sock, (struct sockaddr *)&server_addr, sizeof(struct sockaddr_un))) 
    {    
        tof("Bind socket fail. Reason:%s\n", strerror(errno));
        close(server_sock);
        return -1;
    }

    return 0;    
}

int pilot_send_to_img(int vel){	 
	int connect_fd;
	int ret;
	
	struct data_control send_buff;
	
	static struct sockaddr_un srv_addr;	 	
	// creat unix socket	 	
	connect_fd=socket(AF_UNIX,SOCK_DGRAM,0);	 	
	if(connect_fd<0){	 	
		tof("cannot creat socket");	 
		return -1;	 	
		}	
	srv_addr.sun_family=AF_UNIX;	 
	strcpy(srv_addr.sun_path,PILOT_AND_IMG_U);	 
	//connect server	
//	memset(send_buff,0,1024);	

	send_buff.id=99;
	send_buff.vel=vel;
	ret=(int)sendto(connect_fd,&send_buff,sizeof(send_buff),0,(struct sockaddr*)&srv_addr,sizeof(srv_addr));	
	if (ret<0){	 
		tof("cannot connect server");	 	
		close(connect_fd);	 	
		return -1;	 	
		}	 	 
		 	
	close(connect_fd);	 	
	return 0;	
}

void steam_control_yaw(int yaw)
{


return ;
}
void steam_control_alt(int alt)
{
  float baro_alt_s;
  float roll,pitch,yaw,thr;
  float bf;
  roll=0;
  pitch=0;
  yaw=0;

  baro_alt_s=baro_get_alt_raw();

  if(alt < 0)
  	{
  	// tof("join down alt = %d\n",alt);
     thr=-0.590;
	 while((bf=baro_get_alt_raw()) - baro_alt_s > (float)alt/100)
	 	{
	 	//tof("bf:%3.3f baro_alt_s:%3.3f\n",bf,baro_alt_s);
	 	rc_awlink_set_rc(roll,pitch,yaw,thr);
  	   }
	 }
  else
  	{
  //	tof("join up alt = %d\n",alt);
     thr=0.590;	
     while((bf=baro_get_alt_raw()) - baro_alt_s < (float)alt/100)
      { 
      //tof("bf:%3.3f baro_alt_s:%3.3f\n",bf,baro_alt_s);
	  rc_awlink_set_rc(roll,pitch,yaw,thr);
  	    }
	 }
  
    thr=0;
	rc_awlink_set_rc(roll,pitch,yaw,thr);
	pilot_send_to_img(0);
   //tof("ok===========\n");
	
 return ;
}
void steam_control_tof_althold(int alt)
{

//  float tof_alt_s;
  float roll,pitch,yaw,thr;
  float bf;
  int count=0;
  roll=0;
  pitch=0;
  yaw=0;

 // tof_alt_s=get_tof_data_yaw();
 // debug_t("alt_start = %d tof_alt = %f\n",alt,tof_alt_s);
loop:
  while(fabs(((float)alt/100) - (bf=get_tof_data_yaw())) > 0.05){
  //	count =0;
	if( ((float)alt/100) - bf  > 0){	 
		 thr=0.50;
    }else{			  
	     thr=-0.50;
	}
	 rc_awlink_set_rc(roll,pitch,yaw,thr); 
	 usleep(1000);
		 
  }
 
  if(count++ > 3){
    goto loop;
  }

	thr=0;
	rc_awlink_set_rc(roll,pitch,yaw,thr);
	pilot_send_to_img(0);

}
void steam_control_tof_althold_udp(int alt)
{
  float tof_alt_s;
  float roll,pitch,yaw,thr;

  roll=0;
  pitch=0;
  yaw=0;

  tof_alt_s=get_tof_data_yaw();
  //printf("alt = %d\n",alt);

   if(flag_ac == 0)
   	{
   if(((float)alt/100) - tof_alt_s  > 0.05)
    {    
         flag_ac=up;
         thr=0.590;
         rc_awlink_set_rc(roll,pitch,yaw,thr);
		     
     }
   else
   {
         flag_ac=down;
         thr= -0.590;
         rc_awlink_set_rc(roll,pitch,yaw,thr);
     }
   	}
   else
   	{
     if(flag_ac==up)
     {
       if(fabs(((float)alt/100) - tof_alt_s ) >0.05)
       {    
         thr=0.590;
         rc_awlink_set_rc(roll,pitch,yaw,thr);
		     
       }
	   else
	   	{
	   	flag_ac=0;
		set_tof_althold(0);
	   	  }
       }
	 else
	 {
       if(fabs(((float)alt/100) - tof_alt_s)  > 0.05)
       {    
         thr=-0.590;
         rc_awlink_set_rc(roll,pitch,yaw,thr);
		     
       }
	   else
	   	{
	   	flag_ac=0;
		set_tof_althold(0);
	   	  }
	 
	 	}
       }
	
}
void control_action(struct data_control *buff)
{
   switch(buff->id)
   	{
      case CMD_SAFE_MODE:
	    if(buff->vel == 0)
	    {
		collision_flag=false;
	    	}
		else
		collision_flag=true;
	  	break;
	  case CMD_WING_PROTECTION:
	  	if(buff->vel == 0)
	  	{
		wing_protection_flag=false;
	  		}
		else
		wing_protection_flag=true;
	    break;
	  case CMD_CONTROL_YAW:
	  	steam_control_yaw(buff->vel);
        break;
	  case CMD_CONTROL_ALT:
        if(get_takeoff_flag() == true){
	  	 steam_control_alt(buff->vel);
      	 }
	    else{
         pilot_send_to_img(0);
	    }
	  	break;
	  case CMD_TOF_ALTHOLD:
        if(get_takeoff_flag() == true){
	  	 steam_control_tof_althold(buff->vel);
     	}
	    else{
        pilot_send_to_img(0);
	    }
	  	break;
	  case CMD_TAKEOFF_ALT:
	  	set_takeoff_alt(buff->vel);
      break;
    case CMD_INFRARED_SHOOTING:  
        infra_red_write();
	  	break;
      default:
	  	break;
    }
 	 
}

int pilot_recv_from_img(void *arg)
{ 
	int ret;
	struct data_control rcv_buff;
	socklen_t addrlen; 
	struct sockaddr_un clt_addr;			
	addrlen=sizeof(clt_addr);

	while(system_get_run() == true)
	{
	ret=(int)recvfrom(server_sock,&rcv_buff,sizeof(rcv_buff),0,(struct sockaddr*)&clt_addr,&addrlen);
	if(ret>0)
      { 		
	//	tof("msg = %d\n",rcv_buff.id); 
		control_action(&rcv_buff);
			
		}	
		//tof(" recvfrom fail\n");	
		 
	}
	tof("system run false\n");
	return -1;
}
int pilot_recv_from_udp_update(float arg)
{
    int alt_t;
		  
    if((alt_t = get_tof_althold()) != 0)
       {
          steam_control_tof_althold_udp(alt_t);
           }
	return 0;
}

void pilot_steam_control_init()
{
  make_local_server_fd();
  
}
























