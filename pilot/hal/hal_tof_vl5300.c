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
#include "app_system.h"
#include "app_rc.h"
#include "awlink.h"
#include "awlink_item_control.h"
#include "hal.h"
#define DEBUG_ID DEBUG_ID_HAL

#define TOF_VL5300_PATH "/dev/vi5300"

static int tof_vl5300_fd = -1;

struct vi5300_st_read {
 int16_t dis;
}; 

bool tx_tof_data_vl5300(float *dt, float *dist)
{
   struct vi5300_st_read data;
   int ret = 0;
   ret = read(tof_vl5300_fd,&data,sizeof(data));
   if(ret < 0){
     debug_t("read vi5300 data failed %d cm\n",data.dis/100);
	 return false;
   }
   *dist = (float)data.dis / 1000;
    
  return true;    
}
bool hal_get_tof_fd_vl5300()
{

  if(tof_vl5300_fd > 0)
  	return true;
  else
  	return false;
}

bool tof_vl5300_open()
{
   
	tof_vl5300_fd = open(TOF_VL5300_PATH, O_RDWR);
	if(tof_vl5300_fd < 0){
		INFO(DEBUG_ID,"open tof_vi5300_fd failed (%s)", TOF_VL5300_PATH);
		debug_t("open tof_vi5300_fd failed (%s) fd = %d\n",TOF_VL5300_PATH,tof_vl5300_fd);
		return false;
	}
	debug_t("open tof_vi5300_fd ok (%s) fd = %d\n",TOF_VL5300_PATH,tof_vl5300_fd);
    INFO(DEBUG_ID,"open tof_vi5300_fd ok (%s)", TOF_VL5300_PATH);
   return true;
}

 


