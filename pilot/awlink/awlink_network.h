#ifndef _APP_NETWORK_H_
#define _APP_NETWORK_H_

#include "awlink_decode.h"

void awlink_network_init(awlink_network_s * net);
void awlink_network_update(float dt,awlink_s * link);
void awlink_network_send(awlink_network_s * net,uint8_t * buff,uint32_t length,bool safe);
bool awlink_network_get_connect(awlink_network_s * net);
void awlink_network_exit(awlink_network_s * net);

#endif

