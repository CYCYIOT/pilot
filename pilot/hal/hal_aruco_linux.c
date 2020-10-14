#include <fcntl.h>
#include <unistd.h>
#include <sys/msg.h>
#include <errno.h>
#include "event.h"
#include "app_control.h"
#include "app_debug.h"
#include "app_rc.h"
#include "app_awlink.h"
#include "hal_linux.h"
#include "app_system.h"
#include "pilot_steam_control.h"
#include "app_control_takeoff.h"
#include "awlink_item_control.h"
#include "app_nav.h"
#include "app_attitude.h"
#include "app_control_common.h"
#include "app_att_control.h"
#include "hal_aruco_linux.h"
#include "hal.h"
#include "hal_tof_vl53l1x.h"
#include "hal_arduino.h"
#define DEBUG_ID DEBUG_ID_HAL

typedef struct {
  long    msg_type;  
  uint16_t  id_size; 
  float id_data[9][4];
} msg_rec_sa;


int msg_id_a = -1;
msg_rec_sa aruco_data;


float flow_time_loop;
float flow_time_run;

bool aruco_linux_open()
{
	msg_id_a = msgget((key_t)1235, 0666 | IPC_CREAT);
	if(msg_id_a == -1){
		DEBUG(DEBUG_ID,"create msg failed for aruco_linux");
		return false;
	}else{
		INFO(DEBUG_ID,"use aruco_linux");
		return true;
	}	
}

bool aruco_linux_read(float dt,float vel[2],float *quality,uint16_t * ver)
{
	if(msgrcv(msg_id_a, (void*)&aruco_data, sizeof(aruco_data),0,IPC_NOWAIT) == -1){
		return false;
	}

  //printf("aruco_ver = %d\n",aruco_data.aruco_ver);
 
  return true;

}


static float thr=0;
static int yaw_p=0;
static int x_p=0;
static int y_p=0;
static int flag_aruco=0;
static int cent_count=0;
static int aruco_id=0;
static int action_cmd=-1;
static int x_c=0;
static int y_c=0;

int count0=0;
int count1=0;
int count2=0;
int count3=0;
int count4=0;
int count5=0;
static bool  right_c   = false;
static bool  left_c    = false;
static bool  back_c    = false;
static bool  forward_c = false;
static int   count_time = 0;
static int   count_time_zero = 0;

float get_aruco_thr()
{
  return thr;
}
float get_aruco_x_p()
{
  return x_p;
}
float get_aruco_y_p()
{
  return y_p;
}

float get_aruco_yaw_p()
{
 return yaw_p;
}


#define Power 100
#define Power_zero 0
#define COUNTNUM 10
#define TIME_OUT 60  //6

void set_count_zero()
{
count0=count1=count2=count3=0;

}

void ignore_core(float roll,float pitch)     //忽略特定二维码
{
  if(roll < -9){
    left_c = true;
  }
  if(roll > 9){
    right_c = true;
  }
  if(pitch < 9){
    back_c = true;
  }
  if(pitch > 9){
    forward_c =true;
  }

}

void timeout_set_zero()
{
	count_time=0;
	count_time_zero=0;
}

void start_timeout()
{
 if(count_time++ > TIME_OUT){
	awlink_aruco_send();
	flag_aruco=0;
	printf("time out\n");
	}
}
void start_timeout_forzero()
{
 if(count_time_zero++ > TIME_OUT){
	awlink_aruco_send();
	flag_aruco=x_p=y_p=yaw_p=count_time_zero=0;
	printf("time0 out\n");
	}
}

void set_power(float roll,float pitch,float yaw,int id_aruco,int action)
{

 //ignore_core(roll,pitch);
 
 if(action == ACTION_CENTER){
	x_c=roll;
	y_c=pitch;
 }
 else{
   x_p=roll;
   y_p=pitch;
   yaw_p=yaw;
}
 aruco_id=id_aruco;
 flag_aruco=1;
 action_cmd=action;
 if(action == 1){
 debug_t("start yaw\n");
 }
 timeout_set_zero();

}

void set_power_zero()
{
	x_p=y_p=yaw_p=flag_aruco=0;
	 count2=count3=count4=count5=0;
}
void set_power_zero_only()
{
	x_p=y_p=yaw_p=0;
	timeout_set_zero();
}

#if 0
int pthread_create_aruco_action(){ 
	pthread_attr_t attr; 
	pthread_t heart; 
	pthread_attr_init(&attr); 
	int ret; 
	pthread_attr_setdetachstate(&attr,PTHREAD_CREATE_DETACHED); 
	ret=pthread_create(&heart,&attr,arduino_thread,NULL);
	if(ret < 0) {  
		printf("pthread_create fail\n");  
		return -1;
		}
	pthread_attr_destroy(&attr); 
	return 0;

}
#endif
static int find_relation_code()
{
  int i=0;
  for(i = 0; i< aruco_data.id_size ; i++)
  {
     if(aruco_data.id_data[i][0] >= aruco_id && aruco_data.id_data[i][0] <= aruco_id + 4){
        return i;
	 }
  }
 return -1;
}

static int find_center_code(int num)
{
  int i  = 0;
  int index=0;
  int min_val=sqrt(aruco_data.id_data[0][1]*aruco_data.id_data[0][1] + aruco_data.id_data[0][2] * aruco_data.id_data[0][2]);
  for(i = 1 ; i < num ; i++){
     if(min_val > sqrt(aruco_data.id_data[i][1]*aruco_data.id_data[i][1] + aruco_data.id_data[i][2] * aruco_data.id_data[i][2])){
         min_val = sqrt(aruco_data.id_data[i][1]*aruco_data.id_data[i][1] + aruco_data.id_data[i][2] * aruco_data.id_data[i][2]);
	     index  = i;
	 }
  }

  return index;

}

static int find_target_id(int num)
{
 int i=0;

 for(i = 0;i < num; i++){
  if(aruco_id == aruco_data.id_data[i][0])
      return i;
 }

return -1;

}
void process_around(float around_id)
{
     int i=-1;
  
	if(around_id == aruco_id + 1){				
      if(back_c == true){
       return ;
	  }
	  if(count2++ > COUNTNUM){		//计数10次，连续识别到10次，执行相应的动作
	   y_p=-Power;
	
	  }else{
	  set_power_zero_only();
	  count3=count4=count5=0;
	  right_c=left_c=forward_c=false;
	  }
	 }else if(around_id == aruco_id + 2){
	   if(left_c == true){
        return ;
	   }
	   if(count3++ > COUNTNUM){
		x_p=-Power;
	   }else{
	  set_power_zero_only();
	  count2=count4=count5=0;
	  right_c=back_c=forward_c=false;
	  }
	 }else if(around_id == aruco_id + 3){
       if(forward_c == true){
        return ;
	   }
	   if(count4++ > COUNTNUM){
		y_p=Power;
	   }else{
	   set_power_zero_only();
	   count2=count3=count5=0;
	   right_c=back_c=left_c=false;
	   }
	 }else if(around_id == aruco_id + 4){
	   if(right_c == true){
        return ;
	   }
	   if(count5++ > COUNTNUM){
		x_p=Power;
	   }else{
	   set_power_zero_only();
	   count2=count3=count4=0;
	   forward_c=back_c=left_c=false;
	   }
	 }else{
	  i=find_relation_code();                       //最接近中心二维码非关联二维码，重新寻找
      if(i < 0){
	  start_timeout();
      }else{
        process_around(aruco_data.id_data[i][0]);
	  }
	  
	 }
}
void process_action_def(float target_id)
{
   if(aruco_id == target_id){              //判断是否是目标二维码
	 if(cent_count++ > 2){               // 计数超过 5 次，结束动作
	  flag_aruco=0;                      //判断标志归零
	  cent_count=0;                      //中心计数清零
      count2=count3=count4=count5=0;
      awlink_aruco_send();               //向客户端发送动作完成 ACK
   }		
     set_power_zero_only();                  //第一次识别到目标二维码动力清零       
   }else{
   cent_count=0;
   process_around(target_id);
 }
}
void process_action_yaw(float cal_yaw )
{
 if(count0 > 3){
     yaw_p=-300;
	}
 if(count1 > 3){
     yaw_p=300;
    } 
		
 if(cal_yaw > 170 || cal_yaw <-170 || cal_yaw == 0 || (cal_yaw > -5 && cal_yaw <5)){
      yaw_p=0;
	  flag_aruco=0;
	  count0=count1=0;
      awlink_aruco_send();
	  debug_t("id_size = %d id = %.0f yaw = %.2f ok\n",aruco_data.id_size,aruco_data.id_data[0][0],aruco_data.id_data[0][3]);
	}
 else if(cal_yaw > 5 ){	
  count0++;
 }
 else if(cal_yaw < -5 ){
  count1++;
 }
 
}

void process_1_code_yaw(float target_id)
{
 if(target_id == aruco_id){
   process_action_yaw(aruco_data.id_data[0][3]);
 }else{
   start_timeout();
 }

}
void process_action_center()
{

}
void process_1_code()
{
  switch(action_cmd){
  	case ACTION_DEF:
		process_action_def(aruco_data.id_data[0][0]);
		break;
	case ACTION_YAW:
	    process_1_code_yaw(aruco_data.id_data[0][0]);
		break;
	case ACTION_CENTER:
		process_action_center();
		break;
	default:
		break;
  }

}

void process_def_code()
{
   int i=0;
   int ret =-1;

   ret=find_target_id(aruco_data.id_size);        //先判断是否有目标二维码标记
   if(ret < 0){
     i=find_center_code(aruco_data.id_size);  // 寻找画面中最靠近圆心的二维码标记
     // printf("id = %f \n",aruco_data.id_data[i][0]);
     switch(action_cmd){
  	   case ACTION_DEF:
       process_action_def(aruco_data.id_data[i][0]);
	   break;
	   case ACTION_YAW:                               //没有目标id，不执行校准角度指令，直接超时处理
      // process_action_yaw(aruco_data.id_data[i][3]);
       start_timeout();
	   break;
  	   }
   	}else{
   	  switch(action_cmd){
	  	case ACTION_DEF:
        process_action_def(aruco_data.id_data[ret][0]);
		break;
		case ACTION_YAW:
        process_action_yaw(aruco_data.id_data[ret][3]);
	    break;
   	  	}
	}
}

int aruco_thread(void *arg)
{
 
 while(1)
 {

  if(msgrcv(msg_id_a, (void*)&aruco_data, sizeof(aruco_data),0,0) == -1){  //等待接收aruco识别数据
  	  printf("rcv error\n");
		return false;
	 }

 // printf("id_size = %d id = %.0f yaw = %.2f\n",aruco_data.id_size,aruco_data.id_data[0][0],aruco_data.id_data[0][3]);
  if(flag_aruco == 1){                        //收到指令，启动数据处理                                  

//	debug_t("id_size = %d id = %.0f yaw = %.2f\n",aruco_data.id_size,aruco_data.id_data[0][0],aruco_data.id_data[0][3]);

	switch(aruco_data.id_size){                // 处理二维码个数
     case 0:                                   //识别不到超时处理
	    start_timeout_forzero();
	 	break;
	 case 1:
	 	process_1_code();
	 	break;
	 default:
	 	process_def_code();
	 	break;
      }
   	}
   }
  return 0;
}

