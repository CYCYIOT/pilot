#ifndef _APP_AWLINK_CRC_H_
#define _APP_AWLINK_CRC_H_

uint16_t awlink_crc16_init();
uint16_t awlink_crc16_update(uint8_t data,uint16_t crc);

#endif

