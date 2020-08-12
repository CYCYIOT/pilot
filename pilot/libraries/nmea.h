#ifndef GPS_NMEA_H_
#define GPS_NMEA_H_
#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>

#include "ubx.h"



#define GPS_NMEA_BUF_LEN 128
typedef enum{
	NMEA_START = 0,
	NMEA_DATA,
	NMEA_CHECK0,
	NMEA_CHECK1,
	NMEA_END0,
	NMEA_END1
}nmea_decode_state_t;

typedef struct{
	double lat;
	double lon;
	float  pdop;
	float  hdop;
	float  vdop;
	float  alt;
	float  g_spd;
	float  spd_cog_deg;
	float  mag_dec_deg;
	float  vel_n_m_s;
	float  vel_e_m_s;
	int    sat_num;
	uint8_t fix_type;
	char   lat_ns;
	char   mag_dec_ew;
	bool   status;
	bool   valid;	
}gps_nmea_info_s;	


typedef struct{
	nmea_decode_state_t   decode_state;
	uint8_t               index;
	uint8_t               crc;
	uint8_t            	  msg_type;
	char               	  msg[5];
	uint8_t               msg_cnt;
	char               	  raw_data[GPS_NMEA_BUF_LEN];
	gps_info_s       	  gps_nmea_info;  
	bool rmc_valid;
	bool gga_valid;
	bool gsa_valid;
}nmea_decoder_t;


typedef struct
{
	double  time;
	double  lat;
	char    ns;
	double  lon;
	char    er;
	char    quality;
	char    num_sat;
	float   pdop;
	float   hdop;
	float   vdop;
	float   alt;
	char    uAlt;
	float   sep;
	char    uSep;
	char    diffAge;
	char    diffStation;
	char    cs;
} gps_fix_data;

typedef struct
{
	double    time;
	char      status;
	double    lat;
	char      NS;
	double    lon;
	char      EW;
	double    spd;
	double    cog;
	int 	  date;
	char	  mv;
	char	  mvEW;
	char	  posMode;
	char	  navStatus;
	int		  cs;
} gps_rec_data;

void decode_gps_nmea_byte(nmea_decoder_t *dec,uint8_t byte);
void nmea_copy(nmea_decoder_t *dec,gps_info_s *dat);

#endif
