#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <math.h>
#include <sys/un.h>
#include <asm/byteorder.h>
#include <syslog.h>
#include <time.h>

#include "app_debug.h"
#include "param.h"
#include "hal_tof_vl53l1x.h"
#include "vl53l1_api.h"
#include "app_system.h"
#include "app_rc.h"
#include "awlink.h"
#include "awlink_item_control.h"
#define DEBUG_ID DEBUG_ID_HAL

#define TOF_VL53L1X_PATH "/dev/tof_vl53l1x"
VL53L1_Dev_t                   dev;
VL53L1_DEV                     Dev = &dev;
int tof_vl53l1x_fd = -1;
struct tof_st {
unsigned int index;
unsigned char *data;
unsigned int  length;
unsigned int write_flag;
unsigned int read_flag;
unsigned char byte;
unsigned short word;
};
struct tof_st_read{
unsigned char byte;
unsigned short word;
};


int status;
int tof_dist=-100;
int tof_old=0;
bool flag=false;
bool flag_old=false;
int count_old=0;
int vel;
int count_tof=0;
int tof_before_data[20];
int data_count=0;
int tof_data_count=0;
static float tof_yaw_data = 0;

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



#if 0
void gpio()
{
  __hdle gpio_hd;
  user_gpio_set_t gpio_info;
  gpio_info.port=7;
  gpio_info.port_num=8;
  gpio_info.mul_sel=1;
  gpio_info.pull=1;
  gpio_info.drv_level=1;
  gpio_info.data=1;
  
  gpio_hd=Gpio_request(&gpio_info,1);

}
#endif

float get_tof_data_yaw()
{
return tof_yaw_data;
}


/******初始化TOF*******/
void AutonomousLowPowerRangingTest(void)
{

  Dev->comms_speed_khz = 400;
  Dev->comms_type = 1;

 // printf("Autonomous Ranging Test\n");
  status = VL53L1_WaitDeviceBooted(Dev);
  status = VL53L1_DataInit(Dev);
  
  status = VL53L1_StaticInit(Dev);
  status = VL53L1_SetDistanceMode(Dev, VL53L1_DISTANCEMODE_LONG);
  status = VL53L1_SetMeasurementTimingBudgetMicroSeconds(Dev, 50000);
  status = VL53L1_SetInterMeasurementPeriodMilliSeconds(Dev, 100);
  status = VL53L1_StartMeasurement(Dev);
 
  status = VL53L1_WaitMeasurementDataReady(Dev);
 // printf("auto status = %d\n",status);

}
int tof_thread_main(void* paramter)
{
  float dist;
  while(system_get_run()==true)
    {
    if(get_tof_data(&dist) == true)
        tof_dist=dist;

      }
return 0;
}
bool get_tof_data(float * dist)
{

 bool check=false;
#if 1
// sleep(2);
 static VL53L1_RangingMeasurementData_t RangingData;
 status=0;
 if(!status)
   {
 //   printf("join VL53L1_GetRangingMeasurementData\n");
    status = VL53L1_GetRangingMeasurementData(Dev, &RangingData);
  //  printf("status = %d\n",status);
     if(status==0){
       if(RangingData.RangeStatus == 0){
        
          *dist=(float)RangingData.RangeMilliMeter/1000;
           check=true;
                                         }
       printf("data=====%d,%d,%.2f,%.2f\n", RangingData.RangeStatus,RangingData.RangeMilliMeter,(RangingData.SignalRateRtnMegaCps/65536.0),RangingData.AmbientRateRtnMegaCps/65336.0);
		   }
	status = VL53L1_ClearInterruptAndStartMeasurement(Dev);
     }
#endif 
return check;
}

bool tx_tof_data(float *dt, float *dist)
{
    //char buf[10]={0};
    uint16_t word;
   //  float roll,pitch,yaw,thr;
  //  uint8_t byte;
    int dis=0;
  // int flag_t; 
   /***判断数据是否有效***/
   // i2c_read_tof(0x89,NULL,0,&byte,NULL,2);
   // if(byte == 0x09)
   // {
   // roll=pitch=yaw=0;
   // thr=0.590; 
    i2c_read_tof(0x96,NULL,0,NULL,&word,3);
    dis=word * 2011 + 1024;
    dis/=2048;
        
    *dist=(float)dis / 1000;
   // if(dis != 0) 
  //  tof("dis = %d count_tof =%d\r\n",dis,count_tof++);
    tof_yaw_data=*dist;
    return true;
    
}
bool hal_get_tof_fd()
{
  if(tof_vl53l1x_fd > 0)
  	return true;
  else
  	return false;
}
float hal_get_tof_data()
{ 
   float dt;
   float dist;
   tx_tof_data(&dt,&dist);
   return dist;
}
bool tof_vl53l1x_open()
{
   
	tof_vl53l1x_fd = open(TOF_VL53L1X_PATH, O_RDWR);
	if(tof_vl53l1x_fd < 0){
		INFO(DEBUG_ID,"open tof_vl53l1x_fd failed (%s)", TOF_VL53L1X_PATH);
		return false;
	}else{
		
		INFO(DEBUG_ID,"use tof_vl53l1x tof");
		
	}
    AutonomousLowPowerRangingTest();
   
    INFO(DEBUG_ID,"open tof_vl53l1x_fd ok (%s)", TOF_VL53L1X_PATH);
   return true;
}
/**********************
write_flag: 1    writemulti
            2    writebyte 
            3    writeword
***********************/
void i2c_write_tof(unsigned int index,unsigned char *data,unsigned int count,unsigned char byte ,unsigned short word,unsigned int write_flag)
{


   struct tof_st st;
   //int ret=0;
   st.index=index;
   st.data=data;
   st.length=count;
   st.write_flag=write_flag;
   st.byte=byte;
   st.word=word;
   st.read_flag=0;
  // printf("write_index = %02x count = %d\n",index,count);
   write(tof_vl53l1x_fd,&st,sizeof(st));

}

/**********************
read_flag:  1    readmulti
            2    readbyte 
            3    readword
***********************/
void  i2c_read_tof(unsigned int index, unsigned char *data, unsigned int count,uint8_t *byte,uint16_t *word,unsigned int read_flag)
{

  
  struct tof_st_read index_read;
//  int ret;
 //  int i=0;
  unsigned char data_t[256]={0};;
  // unsigned char data[count]={0};
 #if 1
   struct tof_st index_st;
   index_st.index=index;
   index_st.data=NULL;
   index_st.length=count;
   index_st.read_flag=read_flag;
   index_st.byte=0;  
   index_st.word=0;
  // printf("read_index = %02x count = %d read_flag = %d\n",index,count,read_flag); 
   write(tof_vl53l1x_fd,&index_st,sizeof(index_st)); 
#endif   
   switch(read_flag)
        {
            case 1: 
                read(tof_vl53l1x_fd,data_t,count);
            //   printf(" ret=%d \n",ret);
         //      for(i=0;i<count;i++)
          //     printf(" %02x ",data_t[i]);
               memcpy(data,data_t,count);
             //  printf("data_read===================\n");             
              // for(i=0;i<count;i++)
               //printf(" %02x ",data[i]);
                break;
            case 2:
              read(tof_vl53l1x_fd,&index_read,sizeof(index_read));
             *byte=index_read.byte;
            // memcpy(byte,,count);            
            // printf("data_byte=%02x\n",*byte);
           //  printf("index_read.byte=%d\n",index_read.byte);
                break;
            case 3:
             read(tof_vl53l1x_fd,&index_read,sizeof(index_read));
             *word=index_read.word;
             //printf("data_word=%d\n",*word);
                break;
            default:
                break;
        }
}

 









