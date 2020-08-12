#ifndef __HAL_BATT_LINUX_H__
#define __HAL_BATT_LINUX_H__

bool batt_linux_open();
bool batt_linux_read(float dt,int *batt_status);

#endif

