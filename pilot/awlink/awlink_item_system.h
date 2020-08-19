#ifndef _AWLINK_ITEM_SYSTEM_H_
#define _AWLINK_ITEM_SYSTEM_H_

#include "awlink.h"

void awlink_handle_system(awlink_s * link,awlink_msg_s * msg);
void awlink_encode_system_shooting(awlink_s * link,uint8_t ack,uint8_t i_id,uint8_t si_id);

void awlink_encode_system_aruco_ack(awlink_s * link,uint8_t ack,uint8_t i_id,uint8_t si_id);
void awlink_encode_system_ack(awlink_s * link,uint8_t ack,uint8_t i_id,uint8_t si_id);
void awlink_encode_system_heart(awlink_s * link,uint8_t i);
void awlink_encode_system_msg(awlink_s * link,uint8_t level,uint8_t * buf);

#endif

