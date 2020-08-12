#ifndef _APP_AWLINK_STREAM_H_
#define _APP_AWLINK_STREAM_H_

#include "awlink.h"

void awlink_stream_init(awlink_s * link);
void awlink_stream_update(float dt,awlink_s * link);
void awlink_stream_set_rate(awlink_s * link,uint8_t id,uint8_t rate);
void awlink_stream_stop(awlink_s * link);

#endif

