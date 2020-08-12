#ifndef _APP_AWLINK_ENCODE_H_
#define _APP_AWLINK_ENCODE_H_

#include "awlink.h"
#include "awlink_network.h"

void awlink_encode(awlink_s * link,awlink_msg_s * msg,bool need_buf,bool safe);

#endif

