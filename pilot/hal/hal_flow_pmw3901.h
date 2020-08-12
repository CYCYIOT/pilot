#ifndef _HAL_FLOW_PMW3901_H_
#define	_HAL_FLOW_PMW3901_H_

bool flow_pmw3901_open();
bool flow_pmw3901_read(float dt,float vel[2],float *quality,uint16_t * ver);

#endif

