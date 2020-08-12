#ifndef _APP_AWLINK_DECODE_H_
#define _APP_AWLINK_DECODE_H_

#include "awlink.h"

#include "awlink_protocol.h"

void awlink_decode_init(awlink_decode_s * decoder,uint8_t type);
void awlink_decode(awlink_s * link,awlink_decode_s * decoder,uint8_t * data , int length);

#endif

