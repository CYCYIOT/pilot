#ifndef _AWLINK_ITEM_CONTROL_H_
#define _AWLINK_ITEM_CONTROL_H_

#include "awlink.h"

void awlink_handle_control(awlink_s * link,awlink_msg_s * msg);
int get_tof_flag();
void set_tof_flag(int vel);
int get_tof_althold();
void set_tof_althold(int atl_v);
bool get_file_flag();
void set_file_flag();


#endif

