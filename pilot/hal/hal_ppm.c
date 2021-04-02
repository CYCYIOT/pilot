#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include "app_debug.h"
#include "param.h"
#include "hal.h"
#include "hal_stm8s.h"

#define DEBUG_ID DEBUG_ID_HAL

#define PPM_DEV "/dev/ppm_control"

struct ppm_st{
uint16_t ch_buf[10];
};

static int fd;
bool ppm_open()
{
 
 fd = open(PPM_DEV,O_RDWR);
 if(fd < 0){
  DEBUG(DEBUG_ID,"open PPM failed (%s)", PPM_DEV);
  return false;
 }else{
  INFO(DEBUG_ID,"use PPM fd = %d",fd);
 }
 
 return true;

}

void ppm_update()
{
 //int ret;
 uint16_t ppm_val;
// struct ppm_st ppm;
 uint8_t val[22]; 

  stm8s_read(val,22);
 ppm_val = val[0];
 ppm_val <<= 8;
 ppm_val += val[1];
 
#if 0
 ret = read(fd,&ppm,sizeof(ppm));
 if(ret < 0){
  printf("read ppm fail\n");
  return;
 }
 printf("ch1 = %d ch2 = %d ch3 =  %d ch4 = %d \n",ppm.ch_buf[0],ppm.ch_buf[1],ppm.ch_buf[2],ppm.ch_buf[3]);
#endif
}






