#ifndef __APP_ARDUINO_H_
#define __APP_ARDUINO_H_

#include "hal_arduino.h"

#define ARDUINO_GET_CAP          0
#define ARDUINO_INFSHOOT         2
#define ARDUINO_LED_STOP         3
 


bool arduino_control(uint8_t id);


#endif
