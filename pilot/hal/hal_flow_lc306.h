#ifndef _HAL_FLOW_LC306_H_
#define	_HAL_FLOW_LC306_H_

bool flow_lc306_open();
bool flow_lc306_read(float dt,float vel[2],float *quality,uint16_t * ver);

#endif

