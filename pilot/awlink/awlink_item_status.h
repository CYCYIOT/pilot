#ifndef _AWLINK_ITEM_STATUS_H_
#define _AWLINK_ITEM_STATUS_H_

#include "awlink.h"

void awlink_handle_status(awlink_s * link,awlink_msg_s * msg);

void awlink_encode_status_base_info(awlink_s * link);
void awlink_encode_status_gps_info(awlink_s * link);
void awlink_encode_status_sensor_info(awlink_s * link);
void awlink_encode_status_sensor_calib_info(awlink_s * link);
void awlink_encode_status_user1_info(awlink_s * link);
void awlink_encode_status_user2_info(awlink_s * link);
void awlink_encode_status_user3_info(awlink_s * link);
void awlink_encode_status_user4_info(awlink_s * link);
void awlink_encode_status_mission_info(awlink_s * link);
void awlink_encode_status_mp_info(awlink_s * link);

#endif

