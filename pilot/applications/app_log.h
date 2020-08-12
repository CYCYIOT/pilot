#ifndef _APP_LOG_H_
#define _APP_LOG_H_

#include <stdint.h>

#include "param.h"

#define PACKED __attribute__((__packed__))

#define LOG_PACKET_NAME_LEN					8
#define LOG_PACKET_FORMAT_LEN				24
#define LOG_PACKET_LABELS_LEN				72
#define LOG_PACKET_HEADER_LEN				3
#define LOG_PACKET_DATA_HEADER				uint8_t head1, head2, msg_type; float time;
#define LOG_PACKET_FORMAT_HEADER			uint8_t head1, head2, msg_type;
#define LOG_PACKET_DATA_HEADER_INIT(id)		.head1 = HEAD_BYTE1, .head2 = HEAD_BYTE2, .msg_type = id, .time = get_diff_time(&log_time_start,false)
#define LOG_PACKET_FORMAT_HEADER_INIT(id)	.head1 = HEAD_BYTE1, .head2 = HEAD_BYTE2, .msg_type = id

#define HEAD_BYTE1  0xAB
#define HEAD_BYTE2  0xCD

#define LOG_ALT_MSG			1
#define LOG_ATT_MSG			2
#define LOG_RCI_MSG			3
#define LOG_RCO_MSG			4
#define LOG_IMU_MSG			5
#define LOG_PIDRR_MSG		6
#define LOG_PIDRP_MSG		7
#define LOG_PIDRY_MSG		8
#define LOG_PIDVX_MSG		9
#define LOG_PIDVY_MSG		10
#define LOG_PIDVZ_MSG		11
#define LOG_BARO_MSG		12
#define LOG_SYS_MSG			13
#define LOG_NAV_MSG			14
#define LOG_GPS_MSG			15
#define LOG_BAT_MSG			16
#define LOG_FLOW_MSG		17
#define LOG_MSG_MSG			18
#define LOG_SENS_MSG		19
#define LOG_NAV2_MSG		20
#define LOG_NAV3_MSG		21
#define LOG_FLIP_MSG		22
#define LOG_POSHOLD_MSG     23
#define LOG_RF_MSG     		24
#define LOG_CIR_MSG			25
#define LOG_AHRS_MSG		26
#define LOG_NAV4_MSG		27

#define LOG_FORMAT_MSG	0x80
#define LOG_VER_MSG		0x82

struct PACKED log_RF_s {
	LOG_PACKET_DATA_HEADER
	uint8_t device_status;
	uint8_t status;
	float alt;
	float altf;
	float vel;
	float altt;
	float rotate;
};

struct PACKED log_FLIP_s {
	LOG_PACKET_DATA_HEADER
	float roll;
	float pitch;
	float yaw;
	uint8_t mode;
	float stable_roll;
	float stable_pitch;
};

struct PACKED log_GPS_s {
	LOG_PACKET_DATA_HEADER
	double lat;
	double lon;
	double alt;
	float eph;
	float vel_d_m_s;
	float vel_n_m_s;
	float vel_e_m_s;
	float cog;
	uint8_t satellites_used;
	uint8_t fix_type;
};

struct PACKED log_SYS_s {
	LOG_PACKET_DATA_HEADER
	uint8_t mode;
	uint32_t fs_mode;
	float heart;
	uint8_t m_v[4];
	int m_err;
	float c_val;
};

struct PACKED log_IMU_s {
	LOG_PACKET_DATA_HEADER
	float acc_x;
	float acc_y;
	float acc_z;
	float gyro_x;
	float gyro_y;
	float gyro_z;
	float mag_x;
	float mag_y;
	float mag_z;
	float temp;
};

struct PACKED log_BARO_s {
	LOG_PACKET_DATA_HEADER
	float temp;
	float press;
	float alt;
	float alt_raw;
	float baro_vel_x;
	float baro_vel_y;
	float baro_vel_alt_zero;
	float alt_bias;	
};

struct PACKED log_RCO_s {
	LOG_PACKET_DATA_HEADER
	float roll_out;
	float pitch_out;
	float yaw_out;
	float thr_out;
	float rc1;
	float rc2;
	float rc3;
	float rc4;
};

struct PACKED log_RCI_s {
	LOG_PACKET_DATA_HEADER
	float rc_remote_roll;
	float rc_remote_pitch;
	float rc_remote_yaw;
	float rc_remote_thr;
	float rc_remote_mode;

	float rc_awlink_roll;
	float rc_awlink_pitch;
	float rc_awlink_yaw;
	float rc_awlink_thr;
	
	float rc_roll;
	float rc_pitch;
	float rc_yaw;
	float rc_thr;

	uint8_t mode;
};

struct PACKED log_ATT_s {
	LOG_PACKET_DATA_HEADER
	float roll_des;
	float pitch_des;
	float yaw_des;
	float roll_rate_des;
	float pitch_rate_des;
	float yaw_rate_des;
	float roll_curr;
	float pitch_curr;
	float yaw_curr;
	float roll_rate_curr;
	float pitch_rate_curr;
	float yaw_rate_curr;
	float att_offset_r;
	float att_offset_p;
};

struct PACKED log_AHRS_s {
	LOG_PACKET_DATA_HEADER
	float acc_roll;
	float acc_pitch;
	float acc_sum;
	float nav_acc_roll;
	float nav_acc_pitch;
	float nav_acc_sum;
	float gyro_bias_x;
	float gyro_bias_y;
	float gyro_bias_z;
	float kp;
	float ki;
	float check_time;
	uint8_t mode;
};

struct PACKED log_PID_s {
	LOG_PACKET_DATA_HEADER
	float kp;
	float ki;
	float kd;
	float error;
	float p;
	float i;
	float d;
	float all;
	float allf;
};

struct PACKED log_ALT_s {
	LOG_PACKET_DATA_HEADER
	float pos_des;
	float pos_cur;
	float vel_des;
	float vel_cur;
	float acc_z_des;
	float acc_z_cur;
	float baro_alt_f;
	float baro_alt;
	float baro_vel;
	float rf;
	float rf_vel;
	float acc_z_bias;
};

struct PACKED log_NAV_s {
	LOG_PACKET_DATA_HEADER
	float pos_target_x;
	float pos_target_y;
	float pos_curr_x;
	float pos_curr_y;
	float vel_target_x;
	float vel_target_y;
	float vel_curr_x;
	float vel_curr_y;
	float vel_target_x_b;
	float vel_target_y_b;
	float vel_curr_x_b;
	float vel_curr_y_b;
};

struct PACKED log_NAV2_s {
	LOG_PACKET_DATA_HEADER
	float acc_body_x;
	float acc_body_y;
	float acc_body_z;
	float acc_gnd_x;
	float acc_gnd_y;
	float acc_gnd_z;
	float acc_bias_x;
	float acc_bias_y;
	float acc_bias_z;
	uint8_t pos_valid;
	uint8_t vel_valid;
	uint8_t alt_valid;
};

struct PACKED log_NAV3_s {
	LOG_PACKET_DATA_HEADER
	float flow_pos;
	float flow_vel;
	float flow_bias;
	float fq_weight;
	uint8_t flow_mode;
	float baro_pos;
	float baro_vel;
	float baro_bias;
	uint8_t baro_mode;
	float rf_vel;
	float rf_bias;
	uint8_t rf_mode;
};

struct PACKED log_NAV4_s {
	LOG_PACKET_DATA_HEADER
	float flow_bias0;
	float flow_bias1;
	float flow_bias2;		
	float baro_bias0;
	float baro_bias1;
	float baro_bias2;
	float rf_bias0;
	float rf_bias1;
	float rf_bias2;
	float gps_bias0;
	float gps_bias1;
	float gps_bias2;
};

struct PACKED log_VER_s {
	LOG_PACKET_DATA_HEADER
	char version[64];
};


struct PACKED log_BAT_s {
	LOG_PACKET_DATA_HEADER
	int raw_adc_val;
	int raw_bat_val;
	int vol_origin;
	uint8_t vol;
	uint8_t cap;
        float cap_tmp;
        uint8_t charge;
};

struct PACKED log_FLOW_s {
	LOG_PACKET_DATA_HEADER
	float flow_raw_x;
	float flow_raw_y;
	float flow_scale_x;
	float flow_scale_y;
	float flow_gyro_x;
	float flow_gyro_y;
	float flow_vel_body_x;
	float flow_vel_body_y;
	float flow_dis;
	float flow_qlt;
	float flow_qlt_f;
};

struct PACKED log_SENS_s {
	LOG_PACKET_DATA_HEADER
	float imu;
	float mag;
	float baro;
	float rf;
	float flow;
	float gps;
};

struct PACKED log_MSG_s {
	LOG_PACKET_DATA_HEADER
	uint8_t level;
	uint8_t msg[64];
};

struct PACKED log_POSHOLD_s {
	LOG_PACKET_DATA_HEADER
	uint8_t pos_mode;
	float pos_int_r;
	float pos_int_p;
	float pos_break_time_r;
	float pos_break_time_p;
	uint8_t alt_mode;
	float alt_int_r;
	float alt_int_p;
	float alt_break_time_r;
	float alt_break_time_p;
};

struct PACKED log_CIR_s {
	LOG_PACKET_DATA_HEADER
	float pos_d[2];
	float pos_c[2];
	float vel_d[2];
	float vel_c[2];
	float yaw_body_rate_d;
	float yaw_body_rate_c;
	int status;
};

struct PACKED log_format_s {
	uint8_t type;
	int32_t length;
	char name[LOG_PACKET_NAME_LEN];
	char format[LOG_PACKET_FORMAT_LEN];
	char labels[LOG_PACKET_LABELS_LEN];
};

#define LOG_FORMAT_S(_name, _struct_name, _format, _labels) { \
	.type = LOG_##_name##_MSG, \
	.length = sizeof(struct log_##_struct_name##_s), \
	.name = #_name, \
	.format = _format, \
	.labels = _labels \
}

/*
Format characters in the format string for binary log messages
  b   : int8_t
  B   : uint8_t
  h   : int16_t
  H   : uint16_t
  i   : int32_t
  I   : uint32_t
  f   : float
  d   : double
  Z   : char[64]
  M   : uint8_t flight mode
  q   : int64_t
  Q   : uint64_t
  T   : float Time
 */

static const struct log_format_s log_formats[] = {
	LOG_FORMAT_S(VER,VER,"Z","Version"),
	LOG_FORMAT_S(ALT,ALT,"ffffffffffff","pd,pc,vd,vc,ad,ac,bf,ba,bv,rf,rfv,ab"),
	LOG_FORMAT_S(ATT,ATT,"ffffffffffffff","rd,pd,yd,rrd,prd,yrd,rc,pc,yc,rrc,prc,yrc,aor,aop"),
	LOG_FORMAT_S(AHRS,AHRS,"ffffffffffffB","ar,ap,acc,nar,nap,nacc,gbx,gby,gbz,kp,ki,ct,mode"),
	LOG_FORMAT_S(RCI,RCI,"fffffffffffffB","rrr,rrp,rry,rrt,rrm,rar,rap,ray,rat,rr,rp,ry,rt,mode"),
	LOG_FORMAT_S(RCO,RCO,"ffffffff","r,p,y,t,rc1,rc2,rc3,rc4"),
	LOG_FORMAT_S(IMU,IMU,"ffffffffff","ax,ay,az,gx,gy,gz,mx,my,mz,temp"),
	LOG_FORMAT_S(PIDRR,PID,"fffffffff","kp,ki,kd,error,p,i,d,all,allf"),
	LOG_FORMAT_S(PIDRP,PID,"fffffffff","kp,ki,kd,error,p,i,d,all,allf"),
	LOG_FORMAT_S(PIDRY,PID,"fffffffff","kp,ki,kd,error,p,i,d,all,allf"),
	LOG_FORMAT_S(PIDVX,PID,"fffffffff","kp,ki,kd,error,p,i,d,all,allf"),
	LOG_FORMAT_S(PIDVY,PID,"fffffffff","kp,ki,kd,error,p,i,d,all,allf"),
	LOG_FORMAT_S(PIDVZ,PID,"fffffffff","kp,ki,kd,error,p,i,d,all,allf"),
	LOG_FORMAT_S(BARO,BARO,"ffffffff","temp,press,alt,altr,bvx,bvy,bvaz,abias"),
	LOG_FORMAT_S(SYS,SYS,"MIfBBBBif","mode,fs,heart,mv1,mv2,mv3,mv4,m_err,c_val"),
	LOG_FORMAT_S(NAV,NAV,"ffffffffffff","ptx,pty,pcx,pcy,vtx,vty,vcx,vcy,vtxb,vtyb,vcxb,vcyb"),
	LOG_FORMAT_S(NAV2,NAV2,"fffffffffBBB","abx,aby,abz,agx,agy,agz,bx,by,bz,pv,vv,av"),
	LOG_FORMAT_S(NAV3,NAV3,"ffffBfffBffB","fp,fv,fb,fqw,fm,bp,bv,bb,bm,rv,rb,rm"),
	LOG_FORMAT_S(NAV4,NAV4,"ffffffffffff","fb0,fb1,fb2,bb0,bb1,bb2,rfb0,rfb1,rfb2,gb0,gb1,gb2"),
	LOG_FORMAT_S(GPS,GPS,"dddfffffBB","lat,lon,alt,eph,vd,vn,ve,cog,num,fix"),
	LOG_FORMAT_S(BAT,BAT,"iiiBB","r_adc,r_bat,vol_o,vol,cap"),
	LOG_FORMAT_S(FLOW,FLOW,"fffffffffff","vrx,vry,vsx,vsy,gx,gy,vbx,vby,dtg,qua,quaf"),
	LOG_FORMAT_S(SENS,SENS,"ffffff","imu,mag,baro,rf,flow,gps"),
	LOG_FORMAT_S(FLIP,FLIP,"fffBff","r,p,y,mode,sr,sp"),
	LOG_FORMAT_S(MSG,MSG,"bZ","level,msg"),
	LOG_FORMAT_S(POSHOLD,POSHOLD,"BffffBffff","pm,pir,pip,pbrt,pbpt,am,air,aip,abrt,abpt"),
	LOG_FORMAT_S(RF,RF,"BBfffff","dstatus,status,alt,altf,vel,altt,rotate"),
	LOG_FORMAT_S(CIR,CIR,"ffffffffffi","pdx,pdy,pcx,pcy,vxd,vyd,vxc,vyc,ybrd,ybrc,sta"),
};

static const unsigned log_formats_num = sizeof(log_formats) / sizeof(log_formats[0]);

void log_init();
void log_update(float dt);
void log_print(uint8_t file_num);
void log_msg(uint8_t level,uint8_t * buf);
int log_thread_main(void* paramter);
void log_param_init();
void log_set_restart();


#endif

