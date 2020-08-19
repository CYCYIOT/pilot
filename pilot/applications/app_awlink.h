#ifndef _APP_AWLINK_H_
#define _APP_AWLINK_H_

#include "awlink.h"

void awlink_init();
void awlink_exit();
void awlink_update(float dt);

void awlink_msg(uint8_t level,uint8_t * buf);
void awlink_heart();
void awlink_aruco_send();
void awlink_shooting_send_tcp();

void recv_uart(uint8_t *recv_buf,int len);

#endif

