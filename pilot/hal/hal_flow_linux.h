#ifndef _HAL_FLOW_LINUX_H_
#define _HAL_FLOW_LINUX_H_

bool flow_linux_open();
bool flow_linux_read(float dt,float vel[2],float *quality,uint16_t * ver);

float flow_linux_get_loop();
float flow_linux_get_run();

#endif
