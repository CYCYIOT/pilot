#include <stdio.h> 

#include "nmea.h"
#include "lib_math.h"

static int nmea_read_comma(int num,char *str)
{
	int i,j = 0;
	int len = strlen(str);
	for(i = 0;i < len;i++)
	{
		if(str[i] == ','){
			j++;
		}	
		if(j == num){
			return i + 1;
		}	
	}
	return 0;	
}

void decode_nmea_rmc_data(nmea_decoder_t *dec)
{
	int l = 0;
	double m = 0;
	if(dec->raw_data[nmea_read_comma(2,dec->raw_data)] == 'V'){
		dec->gps_nmea_info.valid = false;
	}else{
		dec->gps_nmea_info.valid = true;
		dec->gps_nmea_info.lat = atof(&dec->raw_data[nmea_read_comma(3,dec->raw_data)]);
		l = dec->gps_nmea_info.lat/100;
		m = dec->gps_nmea_info.lat - l * 100;
		dec->gps_nmea_info.lat = l + m/60.0f;
		dec->gps_nmea_info.lon = atof(&dec->raw_data[nmea_read_comma(5,dec->raw_data)]);
		l = dec->gps_nmea_info.lon /100;
		m = dec->gps_nmea_info.lon - l * 100;
		dec->gps_nmea_info.lon = l + m/60.0f;
		dec->gps_nmea_info.vel_m_s = atof(&dec->raw_data[nmea_read_comma(7,dec->raw_data)]) * 0.5144f;
		dec->gps_nmea_info.cog_rad = atof(&dec->raw_data[nmea_read_comma(8,dec->raw_data)]) *DEG_TO_RAD;		
		dec->gps_nmea_info.vel_ned[0] = dec->gps_nmea_info.vel_m_s * cos(dec->gps_nmea_info.cog_rad);
		dec->gps_nmea_info.vel_ned[1] = dec->gps_nmea_info.vel_m_s * sin(dec->gps_nmea_info.cog_rad);
		dec->gps_nmea_info.mag_dec_deg = atof(&dec->raw_data[nmea_read_comma(10,dec->raw_data)]);
	}
}

void decode_nmea_gga_data(nmea_decoder_t *dec)
{	
	if (dec->gps_nmea_info.valid == false){
		return;
	}else{
		dec->gps_nmea_info.satellites_used = atoi(&dec->raw_data[nmea_read_comma(7,dec->raw_data)]);
		dec->gps_nmea_info.eph = atof(&dec->raw_data[nmea_read_comma(8,dec->raw_data)]);
		dec->gps_nmea_info.alt = atof(&dec->raw_data[nmea_read_comma(9,dec->raw_data)]);
	}
}
static bool decode_nmea_gst_data(nmea_decoder_t *dec)
{
	return false;
}
void decode_nmea_gsa_data(nmea_decoder_t *dec)
{
	if (dec->gps_nmea_info.valid == false){
		return;
	}else{
		dec->gps_nmea_info.fix_type = atof(&dec->raw_data[nmea_read_comma(2,dec->raw_data)]);
		dec->gps_nmea_info.pdop = atof(&dec->raw_data[nmea_read_comma(15,dec->raw_data)]);
		dec->gps_nmea_info.eph = atof(&dec->raw_data[nmea_read_comma(16,dec->raw_data)]);
		dec->gps_nmea_info.epv = atof(&dec->raw_data[nmea_read_comma(17,dec->raw_data)]);
	}
}

void decode_nmea_raw_data(nmea_decoder_t *dec)
{
	if(!strncmp (dec->msg,"RMC",3)){
		decode_nmea_rmc_data(dec);
	}else if(!strncmp (dec->msg,"GGA",3)){
		decode_nmea_gga_data(dec);
	}else if(!strncmp (dec->msg,"GST",3)){
		decode_nmea_gst_data(dec);
	}else if(!strncmp (dec->msg,"GSA",3)){
		decode_nmea_gsa_data(dec);
	}else if(!strncmp (dec->msg,"VTG",3)){	
	}else if(!strncmp (dec->msg,"GSV",3)){	
	}else if(!strncmp (dec->msg,"GLL",3)){	
	}
}


void nmea_decoder_init(nmea_decoder_t *dec)
{
	dec->decode_state = NMEA_START;
	dec->crc = 0;
}
uint8_t hex2gps_char(uint8_t hex)
{
	if(hex < 10){
		return (hex + 0x30);
	}else{
		return (hex - 10 + 0x41);
	}
}

void decode_gps_nmea_byte(nmea_decoder_t *dec,uint8_t byte)
{
	switch(dec->decode_state){
		case NMEA_START:
			if(byte == '$'){
				dec->decode_state = NMEA_DATA;
				dec->index = 0;
				dec->msg_cnt = 0;
				dec->raw_data[dec->index++] = byte;
			}	
			break;
		case NMEA_DATA:
			if(byte == '*'){				
				dec->raw_data[dec->index++] = byte;
				dec->decode_state = NMEA_CHECK0;
			}else{	
				dec->msg_cnt++;
				dec->crc ^= byte;
				dec->raw_data[dec->index++] = byte;
				if(dec->msg_cnt > 2 && dec->msg_cnt < 6){
					dec->msg[dec->msg_cnt-3] = byte;
				}				
			}	
			break;
		case NMEA_CHECK0:			
			if(hex2gps_char(dec->crc >> 4) == byte){
				dec->raw_data[dec->index++] = byte;
				dec->decode_state = NMEA_CHECK1;
			}else{
				nmea_decoder_init(dec);
			}
			break;
		case NMEA_CHECK1:
			if(hex2gps_char(dec->crc & 0x0F) == byte){
				dec->raw_data[dec->index++] = byte;
				dec->decode_state = NMEA_END0;
			}else{
				nmea_decoder_init(dec);
			}
			break;
		case NMEA_END0:
			dec->decode_state = NMEA_END1;
			break;
		case NMEA_END1:
			decode_nmea_raw_data(dec);
			nmea_decoder_init(dec);
			break;
		default:
			nmea_decoder_init(dec);
			break;		
	}

}
void nmea_copy(nmea_decoder_t *dec,gps_info_s *dat)
{
	memcpy(dat,&(dec->gps_nmea_info),sizeof(gps_info_s));
}

