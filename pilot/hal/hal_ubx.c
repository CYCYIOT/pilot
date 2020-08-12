#include <fcntl.h>
#include <unistd.h>
#include <termios.h>

#include "app_debug.h"
#include "app_gps.h"

#include "hal_linux.h"
#include "param.h"
#include "nmea.h"
#include "ubx.h"

#define DEBUG_ID DEBUG_ID_HAL

//#define GPS_UART_PATH "/dev/ttyS2" 
//#define GPS_USING_UBLOX     

static bool gps_existed = false;

ubx_decoder_t ubx_gps;

int buadrate_convert(int buadrate)
{
	switch(buadrate){
		case B9600:
			return 9600;
			break;
		case B38400:
			return  38400;
			break;
		case B115200:
			return  115200;
			break;
		default:
			return  9600;
	};
	return 0;
}

int ubx_config_gps(ubx_decoder_t *dec,int buadrate,int retry)
{
	int i;
	ubx_buf_t		t_buf;

	tcflush(dec->_uart_fd,TCOFLUSH);
	sleep(1);
	hal_linux_uart_cfg(dec->_uart_fd,buadrate);
	gps_existed = false;
	memset(&t_buf.payload_tx_cfg_prt, 0, sizeof(t_buf.payload_tx_cfg_prt));
    t_buf.payload_tx_cfg_prt.portID		= UBX_TX_CFG_PRT_PORTID;
    t_buf.payload_tx_cfg_prt.mode		= UBX_TX_CFG_PRT_MODE;
    t_buf.payload_tx_cfg_prt.baudRate	= buadrate_convert(buadrate);
    t_buf.payload_tx_cfg_prt.inProtoMask	= UBX_TX_CFG_PRT_INPROTOMASK;
    t_buf.payload_tx_cfg_prt.outProtoMask	= UBX_TX_CFG_PRT_OUTPROTOMASK;
	INFO(DEBUG_ID,"[GPS]GPS module buadrate is %d .",t_buf.payload_tx_cfg_prt.baudRate);
	for(i = 0;i < retry;i++){
	    ubx_send_message(dec,UBX_MSG_CFG_PRT, t_buf.raw_gps_data, sizeof(t_buf.payload_tx_cfg_prt));
		if(ubx_wait_for_ack(dec,500000) == 1){
			gps_existed = true;
			break;
		}else{
			INFO(DEBUG_ID,"reconfigure GPS..............");
		}
	}
	if(gps_existed == false){
		INFO(DEBUG_ID,"[GPS]GPS module NOT found!!");
		hal_linux_uart_cfg(dec->_uart_fd,B115200);
		usleep(100000);
		return 0;
	}
	INFO(DEBUG_ID,"[GPS]GPS module found!!");
    memset(&t_buf.payload_tx_cfg_prt, 0, sizeof(t_buf.payload_tx_cfg_prt));
    t_buf.payload_tx_cfg_prt.portID		= UBX_TX_CFG_PRT_PORTID;
    t_buf.payload_tx_cfg_prt.mode		= UBX_TX_CFG_PRT_MODE;
    t_buf.payload_tx_cfg_prt.baudRate	= 115200;
    t_buf.payload_tx_cfg_prt.inProtoMask	= UBX_TX_CFG_PRT_INPROTOMASK;
    t_buf.payload_tx_cfg_prt.outProtoMask	= UBX_TX_CFG_PRT_OUTPROTOMASK;
    ubx_send_message(dec,UBX_MSG_CFG_PRT, t_buf.raw_gps_data, sizeof(t_buf.payload_tx_cfg_prt));
	usleep(100000);
	hal_linux_uart_cfg(dec->_uart_fd,B115200);
	tcflush(dec->_uart_fd,TCIOFLUSH);
	memset(&t_buf.payload_tx_cfg_msg, 0, sizeof(t_buf.payload_tx_cfg_msg));
	t_buf.payload_tx_cfg_msg.msgClass = UBX_CLASS_NAV;
	t_buf.payload_tx_cfg_msg.msgID    = UBX_ID_NAV_PVT;
	t_buf.payload_tx_cfg_msg.rate     = 1;
	ubx_send_message(dec,UBX_MSG_CFG_MSG, t_buf.raw_gps_data, sizeof(t_buf.payload_tx_cfg_msg));
	if(ubx_wait_for_ack(dec,200000) == 0){
		INFO(DEBUG_ID,"[GPS]configure GPS PVT message failed!");
		return 0;
	}

	memset(&t_buf.payload_tx_cfg_nav5, 0, sizeof(t_buf.payload_tx_cfg_nav5));
	t_buf.payload_tx_cfg_nav5.mask		= UBX_TX_CFG_NAV5_MASK;
	t_buf.payload_tx_cfg_nav5.dynModel	= UBX_TX_CFG_NAV5_DYNMODEL;
	t_buf.payload_tx_cfg_nav5.fixMode	= UBX_TX_CFG_NAV5_FIXMODE;
	ubx_send_message(dec,UBX_MSG_CFG_NAV5, t_buf.raw_gps_data, sizeof(t_buf.payload_tx_cfg_nav5));
	if(ubx_wait_for_ack(dec,200000) == 0){
		INFO(DEBUG_ID,"[GPS]configure GPS dynamic mode failed!");
		return 0;
	}

	memset(&t_buf.payload_tx_cfg_rate, 0, sizeof(t_buf.payload_tx_cfg_rate));
	t_buf.payload_tx_cfg_rate.measRate       = 100;    //100ms for 10Hz
	t_buf.payload_tx_cfg_rate.navRate        = UBX_TX_CFG_RATE_NAVRATE;
	t_buf.payload_tx_cfg_rate.timeRef        = UBX_TX_CFG_RATE_TIMEREF;

	ubx_send_message(dec,UBX_MSG_CFG_RATE, t_buf.raw_gps_data, sizeof(t_buf.payload_tx_cfg_rate));
	if(ubx_wait_for_ack(dec,200000) == 0){
		INFO(DEBUG_ID,"Configure GPS output rate failed!");
		return 0;
	}
	INFO(DEBUG_ID,"Configure GPS successfully!");
	return 1;
}

void ubx_printf(void)
{
	int count = 0;
	char buf[256];
	int i;
	count = read(ubx_gps._uart_fd,buf,256);
	
	if(count > 0){
		for(i = 0 ;i < count;i++){
			printf("0x%-2x ",buf[i]);
		}
	}
}

bool ubx_check_nmea(unsigned int timeout)
{
	int count = 0;
	char buf[256];
	int i,j;
	
	for(j = 0;j < (timeout / 50000);j++){
		count = read(ubx_gps._uart_fd,buf,256);
		if(count > 0){
			for(i = 0 ;i < count;i++){
				if(buf[i] == '$' && i < 253){
					if(buf[i+1] == 'G' && buf[i+2] == 'N')
						return true;
				}
			}
		}
		usleep(50000);
	}
	return false;
}


bool ubx_open()
{
#ifdef GPS_USING_UBLOX
	int ubx_uart_fd = -1;
	ubx_uart_fd = open(GPS_UART_PATH, O_RDWR | O_NONBLOCK | O_NDELAY | O_NOCTTY);
	hal_linux_uart_cfg(ubx_uart_fd,B9600);
	ubx_decoder_init(&ubx_gps);
	ubx_decoder_set_uart(&ubx_gps,ubx_uart_fd);
	if(ubx_uart_fd < 0){
		DEBUG(DEBUG_ID,"Can not open dev %s!",GPS_UART_PATH);
		return false;
	}else{
		hal_linux_power_up_gps();

		INFO(DEBUG_ID,"[GPS]configuring gps on %s !",GPS_UART_PATH);
		if(ubx_config_gps(&ubx_gps,B115200,3) == 1){
			return true;
		}else{
			return true;
		}
	}
#else
	return false;
#endif
}


bool ubx_read(float dt,gps_info_s *dat)
{
	int count = 0;
	char buf[256];
	int flag = 0;
	bool ret = false;
	int i;
	count = read(ubx_gps._uart_fd,buf,256);
	dat->valid = 0;
	
	if(count > 0){
		for(i = 0 ;i < count;i++){
			flag =  ubx_parse_byte(&ubx_gps,buf[i]);
			if(flag == 1){
				if(gps_existed == false){
					gps_existed = true;
				}
				ubx_copy(&ubx_gps,dat);
				ret = true;
			}
		}
	}

	dat->module_existed = gps_existed;
	return ret;
}

bool gps_existed_check()
{
	if (gps_existed == false)
		return false;
	else
		return true;
}

