#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "app_awlink.h"

#include "awlink.h"
#include "awlink_network.h"
#include "awlink_protocol.h"
#include "awlink_crc.h"

void awlink_encode_send(awlink_s * link,uint8_t * data,uint8_t length,bool need_buf,bool safe)
{
	awlink_network_s * net;

	net = &link->net;
	
	if(safe == true){
		memcpy(&net->send_buf[net->send_buf_count],data,length);
		net->send_buf_count += length;
		
		if(need_buf == false || net->send_buf_count >= (256 * 5)){
			awlink_network_send(net,net->send_buf,net->send_buf_count,safe);
			net->send_buf_count = 0;
		}
	}else{
		awlink_network_send(net,data,length,safe);
	}
}

void awlink_encode(awlink_s * link,awlink_msg_s * msg,bool need_buf,bool safe)
{
	uint8_t count = 0;
	uint16_t checksum = 0;
	uint8_t buf[256];
		
	msg->magic = AWLINK_MAGIC;
	msg->id_src = AWLINK_IDSRC_CLIENT;

	checksum = awlink_crc16_init();
	checksum = awlink_crc16_update(msg->length,checksum);
	checksum = awlink_crc16_update(msg->id_src,checksum);
	checksum = awlink_crc16_update(msg->item_id,checksum);
	checksum = awlink_crc16_update(msg->subitem_id,checksum);
	for(count = 0 ; count < msg->length ; count++){
		//printf("awlink_encode:[%d]%x \r\n",count,msg->data[count]);
		checksum = awlink_crc16_update(msg->data[count],checksum);		
	}

	msg->checksum = checksum;

	buf[0] = msg->magic;
	buf[1] = msg->length;
	buf[2] = msg->id_src;
	buf[3] = msg->item_id;
	buf[4] = msg->subitem_id;
	for(count = 0 ; count < msg->length ; count++){
		buf[5+count] = msg->data[count];
	}
	buf[5+count] = msg->checksum & 0xFF;
	buf[6+count] = (msg->checksum >> 8) & 0xFF;

	awlink_encode_send(link,buf,msg->length + 7,need_buf,safe);
}

