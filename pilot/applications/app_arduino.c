#include "app_arduino.h"
#include <stdbool.h>
#include <stdint.h>
#include <math.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <termios.h>
#include "hal_arduino.h"
#include "app_batt.h"
#include "hal_infra_red.h"
#include "pilot_steam_control.h"

static void send_cap()
{ 
  uint8_t buf[1]={0};

  buf[0]=batt_get_cap();
  printf("cap = %d \n",buf[0]);
  arduino_write(buf,1);

}
bool arduino_control(uint8_t id)
{
  bool ret = false;
 
  switch(id){
   case ARDUINO_GET_CAP:
   	send_cap();
   	ret = true;
	break;
   case ARDUINO_INFSHOOT:
    infra_red_write();
	ret = true;
	break;
   case ARDUINO_LED_STOP:
   //	pilot_send_to_img(1);
   	ret=true;
	break;
  }


 return ret;
}

