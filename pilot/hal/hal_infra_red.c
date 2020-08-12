#include <fcntl.h>

#include "app_debug.h"

#include "lib_math.h"
#include "param.h"
#include "hal.h"
#include "hal_infra_red.h"
#define DEBUG_ID DEBUG_ID_HAL

#define INFRA_RED_PATH "/dev/infra_red"
//#define JS100PATH       "/dev/js9331_gpio_ir"

struct infra
{
   int launch;
};

 struct st_t{
	unsigned char code_type;
	unsigned int	code_data;
};
static int fd ;

bool infra_red_open()
{
	
	fd = open(INFRA_RED_PATH,O_RDWR);
	if(fd < 0){
		DEBUG(DEBUG_ID,"open infra_red failed (%s)", INFRA_RED_PATH);
		return false;
	}else{
		INFO(DEBUG_ID,"use infra_red fd = %d",fd);
	}

	return true;
}

bool infra_red_write()
{
  int ret=0;
  struct infra st;
 // struct st_t st;
   // st.code_data = 0xCA010001;  
    //st.code_type = 0x02;  
  st.launch=0;
 ret = write(fd,&st,sizeof(st));
 // debug_t("infra_red test ret = %d\n",ret);
 if(ret == -1){
  perror("write fail");
 }

  return true;
}


