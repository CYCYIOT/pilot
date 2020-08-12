#include <fcntl.h>
#include <pthread.h>
#include <signal.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/mount.h>

#include <sys/ipc.h>     
#include <sys/shm.h> 

#include "app_log.h"
#include "app_param.h"
#include "app_pos_control.h"
#include "app_att_control.h"
#include "app_attitude.h"
#include "app_motor.h"
#include "app_system.h"
#include "app_imu.h"
#include "app_mag.h"
#include "app_rangefinder.h"
#include "app_baro.h"
#include "app_rc.h"
#include "app_control.h"
#include "app_nav.h"
#include "app_control_poshold.h"
#include "app_control_followme.h"
#include "app_batt.h"
#include "app_failsafe.h"
#include "app_flow.h"
#include "app_control_common.h"
#include "app_gps.h"
#include "app_debug.h"
#include "app_air_resistance_break.h"
#include "app_control_flip.h"
#include "app_control_circle.h"
#include "app_control_althold.h"
#include "ahrs.h"
#include "ubx.h"
#include "hal_vl53l1x.h"
#include "hal_stm8s.h"
#include "lib_math.h"
#include "lib_inav_baro.h"
#include "lib_inav_flow.h"
#include "lib_inav_rangefinder.h"
#include "lib_inav_gps.h"

#define DEBUG_ID DEBUG_ID_LOG

#define	CARD_DEV_NODE1		"/dev/mmcblk0"
#define	CARD_DEV_NODE2		"/dev/mmcblk0p1"

struct timespec log_time_start;
static bool log_restart = false;
static bool log_need_umount = false;
static bool log_tf_check = true;
static int log_fd = -1;

#define LOG_BUF_SIZE	(50 * 1024)
#define LOG_FILE_SIZE	(8 * 1024 * 1024)

static int32_t log_file_size_count;

static uint8_t log_buf[LOG_BUF_SIZE];
static int32_t log_buf_count = 0;

pthread_mutex_t log_buf_thread_mutex = PTHREAD_MUTEX_INITIALIZER;
static uint8_t log_thread_buf[LOG_BUF_SIZE];
static int32_t log_thread_buf_count = 0;

static float log_run = 0;
static float log_rate_flow = 0;
static float log_rate_sens = 0;
static float log_rate_alt = 0;
static float log_rate_att = 0;
static float log_rate_ahrs = 0;
static float log_rate_pidrr = 0;
static float log_rate_pidrp = 0;
static float log_rate_pidry = 0;
static float log_rate_pidvx = 0;
static float log_rate_pidvy = 0;
static float log_rate_pidvz = 0;
static float log_rate_imu = 0;
static float log_rate_bat = 0;
static float log_rate_nav = 0;
static float log_rate_nav2 = 0;
static float log_rate_nav3 = 0;
static float log_rate_nav4 = 0;
static float log_rate_rci = 0;
static float log_rate_rco = 0;
static float log_rate_baro = 0;
static float log_rate_gps = 0;
static float log_rate_system = 0;
static float log_rate_poshold = 0;
static float log_rate_rangefinder = 0;

void log_output_thread()
{
	if(log_buf_count > 0 && pthread_mutex_trylock(&log_buf_thread_mutex) == 0){
		if((LOG_BUF_SIZE - log_thread_buf_count) > log_buf_count){
			memcpy(&log_thread_buf[log_thread_buf_count],log_buf,log_buf_count);
			log_thread_buf_count += log_buf_count;
			log_buf_count = 0;
		}
		pthread_mutex_unlock(&log_buf_thread_mutex);
	}
}

void log_buf_write(void * data,int16_t size)
{
	if(log_run > 0 && (LOG_BUF_SIZE - log_buf_count) > size){
		memcpy(&log_buf[log_buf_count],data,size);
		log_buf_count += size;
	}
}

inline bool log_time_update(float * time , float hz , float dt)
{
	float rate;

	if(hz <= 0){
		return false;
	}
	
	rate = 1.0f / hz;
	*time += dt;
	if(*time >= rate){
		if(*time <= (1.5f * rate)){
			*time -= rate;
		}else{
			*time = 0;
		}
		return true;
	}else{
		return false;
	}
}

void log_imu(float rate,float dt,bool armed)
{
	static float time = 0;
	if(log_time_update(&time,rate,dt) == true && armed == true){
		float acc_raw[3];
		float gyro[3];
		float mag[3];
		
		struct log_IMU_s pkt = {
			LOG_PACKET_DATA_HEADER_INIT(LOG_IMU_MSG),
		};

		imu_get_acc(acc_raw);
		imu_get_gyro(gyro);
		mag_get_val(mag);
		
		pkt.acc_x = acc_raw[0];
		pkt.acc_y = acc_raw[1];
		pkt.acc_z = acc_raw[2];
		pkt.gyro_x = gyro[0];
		pkt.gyro_y = gyro[1];
		pkt.gyro_z = gyro[2];
		pkt.mag_x = mag[0];
		pkt.mag_y = mag[1];
		pkt.mag_z = mag[2];
		pkt.temp = imu_get_temp();
			
		log_buf_write((void *)&pkt,sizeof(pkt));
	}
}

void log_rco(float rate,float dt,bool armed)
{
	static float time = 0;
	if(log_time_update(&time,rate,dt) == true && armed == true){
		float motor_control[4];
		float motor_output[MAX_NUM_MOTORS];
		
		struct log_RCO_s pkt = {
			LOG_PACKET_DATA_HEADER_INIT(LOG_RCO_MSG),
		};

		motor_control_get(motor_control);
		motor_output_get(motor_output);
		
		pkt.roll_out = motor_control[0];
		pkt.pitch_out = motor_control[1];
		pkt.yaw_out = motor_control[2];
		pkt.thr_out = motor_control[3];
		pkt.rc1	= motor_output[0];
		pkt.rc2	= motor_output[1];
		pkt.rc3	= motor_output[2];
		pkt.rc4	= motor_output[3];
		
		log_buf_write((void *)&pkt,sizeof(pkt));
	}
}

void log_rci(float rate,float dt,bool armed)
{
	static float time = 0;
	if(log_time_update(&time,rate,dt) == true && armed == true){
		struct log_RCI_s pkt = {
			LOG_PACKET_DATA_HEADER_INIT(LOG_RCI_MSG),
		};

		float rc_remote[5];
		float rc_awlink[4];
		rc_s rc;
		
		rc_get(&rc);
		rc_remote_get(rc_remote);
		rc_awlink_get(rc_awlink);
		
		pkt.rc_remote_roll	= rc_remote[0];
		pkt.rc_remote_pitch	= rc_remote[1];
		pkt.rc_remote_yaw	= rc_remote[2];
		pkt.rc_remote_thr	= rc_remote[3];
		pkt.rc_remote_mode	= rc_remote[4];
		
		pkt.rc_awlink_roll	= rc_awlink[0];
		pkt.rc_awlink_pitch	= rc_awlink[1];
		pkt.rc_awlink_yaw	= rc_awlink[2];
		pkt.rc_awlink_thr	= rc_awlink[3];

		pkt.rc_roll			= rc.roll_raw;
		pkt.rc_pitch		= rc.pitch_raw;
		pkt.rc_yaw			= rc.yaw_raw;
		pkt.rc_thr			= rc.thr_raw;

		pkt.mode			= rc.mode;
		
		log_buf_write((void *)&pkt,sizeof(pkt));
	}
}

void log_att(float rate,float dt,bool armed)
{
	static float time = 0;
	static float att_offset[2] = {0};
	if(log_time_update(&time,rate,dt) == true && armed == true){
		struct log_ATT_s pkt = {
			LOG_PACKET_DATA_HEADER_INIT(LOG_ATT_MSG),
		};
		nav_get_att_offset(att_offset);	

		pkt.roll_des		= att_control_get_target_att_roll();
		pkt.pitch_des		= att_control_get_target_att_pitch();
		pkt.yaw_des			= att_control_get_target_att_yaw();
		pkt.roll_rate_des	= att_control_get_target_rate_roll();
		pkt.pitch_rate_des	= att_control_get_target_rate_pitch();
		pkt.yaw_rate_des	= att_control_get_target_rate_yaw();
		
		pkt.roll_curr		= attitude_get_att_roll();
		pkt.pitch_curr		= attitude_get_att_pitch();
		pkt.yaw_curr		= attitude_get_att_yaw();
		pkt.roll_rate_curr	= attitude_get_rate_roll();
		pkt.pitch_rate_curr	= attitude_get_rate_pitch();
		pkt.yaw_rate_curr	= attitude_get_rate_yaw();

		pkt.att_offset_r	= att_offset[0];
		pkt.att_offset_p	= att_offset[1];
		log_buf_write((void *)&pkt,sizeof(pkt));
	}
}

void log_ahrs(float rate,float dt,bool armed)
{
	static float time = 0;
	if(log_time_update(&time,rate,dt) == true && armed == true){
		struct log_AHRS_s pkt = {
			LOG_PACKET_DATA_HEADER_INIT(LOG_AHRS_MSG),
		};
		
		float gyro_bias[3];
		float acc[3];
		inav_estimator est;
		
		nav_get_estimator_data(&est);
		attitude_get_gyro_bias(gyro_bias);
		imu_get_acc_filted(acc);
		
		pkt.acc_roll 		= degrees(-atan2f(acc[1],-acc[2]));
		pkt.acc_pitch 		= degrees(asinf(-acc[0]/(-CONSTANTS_ONE_G)));
		pkt.acc_sum 		= pythagorous_v3f(acc);

		pkt.nav_acc_roll 	= degrees(-atan2f(est.acc_body[1],-est.acc_body[2]));
		pkt.nav_acc_pitch 	= degrees(asinf(-est.acc_body[0]/(-CONSTANTS_ONE_G)));
		pkt.nav_acc_sum 	= nav_get_acc_body_sum();
		
		pkt.gyro_bias_x		= gyro_bias[0];
		pkt.gyro_bias_y		= gyro_bias[1];
		pkt.gyro_bias_z		= gyro_bias[2];

		pkt.kp				= ahrs_get_rp_p();
		pkt.ki				= ahrs_get_rp_i();
		pkt.check_time		= ahrs_get_check_time();
		pkt.mode			= ahrs_get_mode();
		
		log_buf_write((void *)&pkt,sizeof(pkt));
	}
}

void log_baro(float rate,float dt,bool armed)
{
	static float time = 0;
	if(log_time_update(&time,rate,dt) == true && armed == true){
		struct log_BARO_s pkt = {
			LOG_PACKET_DATA_HEADER_INIT(LOG_BARO_MSG),
		};
		
		pkt.alt			= baro_get_alt();
		pkt.press		= baro_get_pres();
		pkt.temp		= baro_get_temp();
		pkt.alt_raw     = baro_get_alt_raw();
		pkt.baro_vel_x  = baro_get_baro_vel(0);
		pkt.baro_vel_y  = baro_get_baro_vel(1);
		pkt.baro_vel_alt_zero=baro_get_baro_alt_vel_zero();
		pkt.alt_bias   = baro_get_baro_alt_bias();	
		
		log_buf_write((void *)&pkt,sizeof(pkt));
	}
}

void log_nav(float rate,float dt,bool armed)
{
	static float time = 0;
	if(log_time_update(&time,rate,dt) == true && armed == true){
		struct log_NAV_s pkt = {
			LOG_PACKET_DATA_HEADER_INIT(LOG_NAV_MSG),
		};
				
		pkt.pos_target_x	= pos_control_get_pos_ned_x_target(); 
		pkt.pos_target_y	= pos_control_get_pos_ned_y_target(); 
		pkt.pos_curr_x		= nav_get_pos_ned_x();
		pkt.pos_curr_y		= nav_get_pos_ned_y();
		pkt.vel_target_x	= pos_control_get_vel_ned_x_target(); 
		pkt.vel_target_y	= pos_control_get_vel_ned_y_target(); 
		pkt.vel_curr_x		= nav_get_vel_ned_x(); 
		pkt.vel_curr_y		= nav_get_vel_ned_y(); 
		pkt.vel_target_x_b	= pos_control_get_vel_grd_x_target(); 
		pkt.vel_target_y_b	= pos_control_get_vel_grd_y_target(); 
		pkt.vel_curr_x_b	= nav_get_vel_grd_x(); 
		pkt.vel_curr_y_b	= nav_get_vel_grd_y(); 
		
		log_buf_write((void *)&pkt,sizeof(pkt));
	}
}

void log_nav2(float rate,float dt,bool armed)
{
	static float time = 0;
	if(log_time_update(&time,rate,dt) == true && armed == true){
		struct log_NAV2_s pkt = {
			LOG_PACKET_DATA_HEADER_INIT(LOG_NAV2_MSG),
		};

		inav_estimator est;	
		nav_get_estimator_data(&est);
		float acc_gnd[2];
		float yaw_cos;
		float yaw_sin;

		yaw_cos = cosf(radians(attitude_get_att_yaw()));
		yaw_sin = sinf(radians(attitude_get_att_yaw()));
		acc_gnd[0] =  est.acc_ned[0] * yaw_cos + est.acc_ned[1] * yaw_sin;
		acc_gnd[1] = -est.acc_ned[0] * yaw_sin + est.acc_ned[1] * yaw_cos;
		
		pkt.acc_body_x	= est.acc_body[0]; 
		pkt.acc_body_y	= est.acc_body[1]; 
		pkt.acc_body_z	= est.acc_body[2]; 
		pkt.acc_gnd_x	= acc_gnd[0]; 
		pkt.acc_gnd_y	= acc_gnd[1]; 
		pkt.acc_gnd_z	= -est.acc_ned[2]; 
		pkt.acc_bias_x	= est.acc_bias_body[0]; 
		pkt.acc_bias_y	= est.acc_bias_body[1]; 
		pkt.acc_bias_z	= est.acc_bias_body[2]; 
		pkt.pos_valid	= est.pos_valid; 
		pkt.vel_valid	= est.vel_valid; 
		pkt.alt_valid	= est.alt_valid; 
			
		log_buf_write((void *)&pkt,sizeof(pkt));
	}
}

void log_nav3(float rate,float dt,bool armed)
{
	static float time = 0;
	if(log_time_update(&time,rate,dt) == true && armed == true){
		struct log_NAV3_s pkt = {
			LOG_PACKET_DATA_HEADER_INIT(LOG_NAV3_MSG),
		};

		float flow_fuse[3];
		float baro_fuse[3];
		float rf_fuse[2];

		inav_flow_get_fuse_val(&flow_fuse[0],&flow_fuse[1],&flow_fuse[2]);
		inav_baro_get_fuse_val(&baro_fuse[0],&baro_fuse[1],&baro_fuse[2]);
		inav_rf_get_fuse_val(&rf_fuse[0],&rf_fuse[1]);
		
		pkt.flow_pos	= flow_fuse[0]; 
		pkt.flow_vel	= flow_fuse[1]; 
		pkt.flow_bias	= flow_fuse[2];
		pkt.fq_weight   = inav_flow_get_fq_weight();
		pkt.flow_mode	= inav_flow_get_fuse_mode(); 
		pkt.baro_pos	= baro_fuse[0]; 
		pkt.baro_vel	= baro_fuse[1]; 
		pkt.baro_bias	= baro_fuse[2];
		pkt.baro_mode	= inav_baro_get_fuse_mode(); 
		pkt.rf_vel		= rf_fuse[0]; 
		pkt.rf_bias		= rf_fuse[1]; 
		pkt.rf_mode		= inav_rf_get_fuse_mode(); 
			
		log_buf_write((void *)&pkt,sizeof(pkt));
	}
}

void log_nav4(float rate,float dt,bool armed)
{
	static float time = 0;
	
	if(log_time_update(&time,rate,dt) == true && armed == true){
		struct log_NAV4_s pkt = {
			LOG_PACKET_DATA_HEADER_INIT(LOG_NAV4_MSG),
		};
		
		float flow_bias[3];
		float baro_bias[3];
		float rf_bias[3];
		float gps_bias[3];
		
		inav_flow_get_bias_value(flow_bias);
		inav_baro_get_bias_value(baro_bias);
		inav_rf_get_bias_value(rf_bias);
		inav_gps_get_bias_value(gps_bias);
		pkt.flow_bias0	= flow_bias[0];
		pkt.flow_bias1	= flow_bias[1];
		pkt.flow_bias2	= flow_bias[2];
		pkt.baro_bias0	= baro_bias[0];
		pkt.baro_bias1	= baro_bias[1];
		pkt.baro_bias2	= baro_bias[2];	
		pkt.rf_bias0	= rf_bias[0];
		pkt.rf_bias1	= rf_bias[1];
		pkt.rf_bias2	= rf_bias[2];
		pkt.gps_bias0	= gps_bias[0];
		pkt.gps_bias1	= gps_bias[1];
		pkt.gps_bias2	= gps_bias[2];	

		log_buf_write((void *)&pkt,sizeof(pkt));
	}
}

void log_alt(float rate,float dt,bool armed)
{
	static float time = 0;
	if(log_time_update(&time,rate,dt) == true && armed == true){
		struct log_ALT_s pkt = {
			LOG_PACKET_DATA_HEADER_INIT(LOG_ALT_MSG),
		};
		
		pkt.pos_des		= pos_control_get_pos_ned_z_target();
		pkt.pos_cur		= nav_get_pos_ned_z();
		pkt.vel_des		= pos_control_get_vel_ned_z_target();
		pkt.vel_cur		= nav_get_vel_ned_z();
		pkt.acc_z_des	= pos_control_get_acc_z_target();
		pkt.acc_z_cur	= nav_get_acc_ned_z();
		pkt.baro_alt_f	= baro_get_alt_f();
		pkt.baro_alt	= baro_get_alt();
		pkt.baro_vel	= baro_get_vel_f();
		pkt.rf			= rangefinder_get_range_f();
		pkt.rf_vel		= rangefinder_get_vel();
		pkt.acc_z_bias	= nav_get_acc_bias_body_z();

		log_buf_write((void *)&pkt,sizeof(pkt));
	}
}

void log_pid_roll(float rate,float dt,bool armed)
{
	static float time = 0;
	if(log_time_update(&time,rate,dt) == true && armed == true){
		pid_s * pid;
		struct log_PID_s pkt = {
			LOG_PACKET_DATA_HEADER_INIT(LOG_PIDRR_MSG),
		};

		pid = att_control_get_pid(0);
		
		pkt.kp		= pid->kp;
		pkt.ki		= pid->ki;
		pkt.kd		= pid->kd;
		pkt.error	= pid->error_now;
		pkt.p		= pid->kp_val;
		pkt.i		= pid->ki_val;
		pkt.d		= pid->kd_val;
		pkt.all		= pid->output;
		pkt.allf	= pid->output_f;

		log_buf_write((void *)&pkt,sizeof(pkt));
	}
}

void log_pid_pitch(float rate,float dt,bool armed)
{
	static float time = 0;
	if(log_time_update(&time,rate,dt) == true && armed == true){
		pid_s * pid;
		struct log_PID_s pkt = {
			LOG_PACKET_DATA_HEADER_INIT(LOG_PIDRP_MSG),
		};

		pid = att_control_get_pid(1);
		
		pkt.kp		= pid->kp;
		pkt.ki		= pid->ki;
		pkt.kd		= pid->kd;
		pkt.error	= pid->error_now;
		pkt.p		= pid->kp_val;
		pkt.i		= pid->ki_val;
		pkt.d		= pid->kd_val;
		pkt.all		= pid->output;
		pkt.allf	= pid->output_f;

		log_buf_write((void *)&pkt,sizeof(pkt));
	}
}

void log_pid_yaw(float rate,float dt,bool armed)
{
	static float time = 0;
	if(log_time_update(&time,rate,dt) == true && armed == true){
		pid_s * pid;
		struct log_PID_s pkt = {
			LOG_PACKET_DATA_HEADER_INIT(LOG_PIDRY_MSG),
		};

		pid = att_control_get_pid(2);
		
		pkt.kp		= pid->kp;
		pkt.ki		= pid->ki;
		pkt.kd		= pid->kd;
		pkt.error	= pid->error_now;
		pkt.p		= pid->kp_val;
		pkt.i		= pid->ki_val;
		pkt.d		= pid->kd_val;
		pkt.all		= pid->output;
		pkt.allf	= pid->output_f;

		log_buf_write((void *)&pkt,sizeof(pkt));
	}
}

void log_pid_x(float rate,float dt,bool armed)
{
	static float time = 0;
	if(log_time_update(&time,rate,dt) == true && armed == true){
		pid_s * pid;
		struct log_PID_s pkt = {
			LOG_PACKET_DATA_HEADER_INIT(LOG_PIDVX_MSG),
		};

		pid = pos_control_get_pid(0);
		
		pkt.kp		= pid->kp;
		pkt.ki		= pid->ki;
		pkt.kd		= pid->kd;
		pkt.error	= pid->error_now;
		pkt.p		= pid->kp_val;
		pkt.i		= pid->ki_val;
		pkt.d		= pid->kd_val;
		pkt.all		= pid->output;
		pkt.allf	= pid->output_f;

		log_buf_write((void *)&pkt,sizeof(pkt));
	}
}

void log_pid_y(float rate,float dt,bool armed)
{
	static float time = 0;
	if(log_time_update(&time,rate,dt) == true && armed == true){
		pid_s * pid;
		struct log_PID_s pkt = {
			LOG_PACKET_DATA_HEADER_INIT(LOG_PIDVY_MSG),
		};

		pid = pos_control_get_pid(1);
		
		pkt.kp		= pid->kp;
		pkt.ki		= pid->ki;
		pkt.kd		= pid->kd;
		pkt.error	= pid->error_now;
		pkt.p		= pid->kp_val;
		pkt.i		= pid->ki_val;
		pkt.d		= pid->kd_val;
		pkt.all		= pid->output;
		pkt.allf	= pid->output_f;

		log_buf_write((void *)&pkt,sizeof(pkt));
	}
}

void log_pid_z(float rate,float dt,bool armed)
{
	static float time = 0;
	if(log_time_update(&time,rate,dt) == true && armed == true){
		pid_s * pid;
		struct log_PID_s pkt = {
			LOG_PACKET_DATA_HEADER_INIT(LOG_PIDVZ_MSG),
		};

		pid = pos_control_get_pid(2);
		
		pkt.kp		= pid->kp;
		pkt.ki		= pid->ki;
		pkt.kd		= pid->kd;
		pkt.error	= pid->error_now;
		pkt.p		= pid->kp_val;
		pkt.i		= pid->ki_val;
		pkt.d		= pid->kd_val;
		pkt.all		= pid->output;
		pkt.allf	= pid->output_f;

		log_buf_write((void *)&pkt,sizeof(pkt));
	}
}

void log_gps(float rate,float dt,bool armed)
{
	static float time = 0;
	if(log_time_update(&time,rate,dt) == true && armed == true){
		struct log_GPS_s pkt = {
			LOG_PACKET_DATA_HEADER_INIT(LOG_GPS_MSG),
		};

		gps_info_s info;
		gps_get_info(&info);
		
		pkt.lat			= info.lat;
		pkt.lon			= info.lon;
	    pkt.alt         = info.alt;
		pkt.eph			= info.eph;
		pkt.vel_n_m_s	= info.vel_ned[0];
		pkt.vel_e_m_s	= info.vel_ned[1];
	    pkt.vel_d_m_s	= info.vel_ned[2];
		pkt.cog			= info.cog_rad;
		pkt.fix_type	= info.fix_type;
		pkt.satellites_used	= info.satellites_used;

		log_buf_write((void *)&pkt,sizeof(pkt));
	}
}

void log_flow(float rate,float dt,bool armed)
{
	static float time = 0;
	if(log_time_update(&time,rate,dt) == true && armed == true){
		struct log_FLOW_s pkt = {
			LOG_PACKET_DATA_HEADER_INIT(LOG_FLOW_MSG),
		};
		
		float flow_gyro[2];
		float flow_vel_raw[2];
		float flow_vel_scale[2];
		float flow_vel[2];
		
		flow_get_gyro(flow_gyro);
		flow_get_vel_raw(flow_vel_raw);
		flow_get_vel_scale(flow_vel_scale);
		flow_get_vel(flow_vel);

		pkt.flow_raw_x		= flow_vel_raw[0];
		pkt.flow_raw_y		= flow_vel_raw[1];
		pkt.flow_scale_x	= flow_vel_scale[0];
		pkt.flow_scale_y	= flow_vel_scale[1];
		pkt.flow_vel_body_x = flow_vel[0];
		pkt.flow_vel_body_y = flow_vel[1];
		pkt.flow_gyro_x 	= -flow_gyro[0];
		pkt.flow_gyro_y 	= flow_gyro[1];
		pkt.flow_dis		= flow_get_dtg();
		pkt.flow_qlt		= flow_get_quality();
		pkt.flow_qlt_f		= flow_get_quality_f();
		
		log_buf_write((void *)&pkt,sizeof(pkt));
	}	
}

void log_msg(uint8_t level,uint8_t * buf)
{
	struct log_MSG_s pkt = {
		LOG_PACKET_DATA_HEADER_INIT(LOG_MSG_MSG),
	};
	
	pkt.level	= level;
	snprintf((char *)pkt.msg,64,(char *)buf);

	log_buf_write((void *)&pkt,sizeof(pkt));
}

void log_sens(float rate,float dt,bool armed)
{
	static float time = 0;
	if(log_time_update(&time,rate,dt) == true && armed == true){
		struct log_SENS_s pkt = {
			LOG_PACKET_DATA_HEADER_INIT(LOG_SENS_MSG),
		};
		
		pkt.imu		= imu_get_error_time();
		pkt.baro	= baro_get_error_time();
		pkt.mag		= mag_get_error_time();
		pkt.rf		= rangefinder_get_error_time();
		pkt.flow	= flow_get_error_time();
		pkt.gps		= gps_get_error_time();

		log_buf_write((void *)&pkt,sizeof(pkt));
	}
}

void log_system(float rate,float dt,bool armed)
{
	static float time = 0;
	if(log_time_update(&time,rate,dt) == true && armed == true){
		struct log_SYS_s pkt = {
			LOG_PACKET_DATA_HEADER_INIT(LOG_SYS_MSG),
		};
		
		uint8_t m_val[4];
		stm8s_get_motor_val(m_val);
		
		pkt.mode	= control_get_mode();
		pkt.fs_mode	= failsafe_get_mode();
		pkt.heart	= system_get_awlink_heart_rate();
		pkt.m_v[0] = m_val[0];
		pkt.m_v[1] = m_val[1];
		pkt.m_v[2] = m_val[2];
		pkt.m_v[3] = m_val[3];
	//	printf("mv[0] = %d mv[1] = %d mv[2] = %d mv[3] = %d\n",m_val[0],m_val[1],m_val[2],m_val[3]);
		pkt.m_err = 0;
		pkt.c_val = failsafe_get_collision_val();
		
		log_buf_write((void *)&pkt,sizeof(pkt));
	}
}

void log_bat(float rate,float dt,bool armed)
{
	static float time = 0;
	if(log_time_update(&time,rate,dt) == true && armed == true){
		struct log_BAT_s pkt = {
			LOG_PACKET_DATA_HEADER_INIT(LOG_BAT_MSG),
		};
		
		pkt.vol_origin = batt_get_origin_vol();
		pkt.vol = batt_get_vol();
		pkt.cap = batt_get_cap();
		pkt.raw_adc_val = batt_get_raw_adc_val();
		pkt.raw_bat_val = batt_get_raw_bat_val();
	    pkt.cap_tmp=batt_get_cap_tmp();
        pkt.charge=batt_get_charge();	
		log_buf_write((void *)&pkt,sizeof(pkt));
	}
}

void log_flip(float rate,float dt,bool armed)
{
	static float time = 0;
	if(log_time_update(&time,rate,dt) == true && armed == true){
		struct log_FLIP_s pkt = {
			LOG_PACKET_DATA_HEADER_INIT(LOG_FLIP_MSG),
		};

		if(control_get_mode() == CONTROL_MODE_FLIP){
			
			pkt.roll = control_flip_get_roll();
			pkt.pitch = control_flip_get_pitch();
			pkt.yaw = control_flip_get_yaw();			
			pkt.mode = control_flip_get_mode();
			pkt.stable_roll = control_flip_stable_get_roll_target();
			pkt.stable_pitch = control_flip_stable_get_pitch_target();
			
			log_buf_write((void *)&pkt,sizeof(pkt));
		}
	}
}

void log_poshold(float rate,float dt,bool armed)
{
	static float time = 0;
	float break_time[2];
	float pos_break1_time[2];
	float angle_integrate[2];
	
	if(log_time_update(&time,rate,dt) == true && armed == true){
		struct log_POSHOLD_s pkt = {
			LOG_PACKET_DATA_HEADER_INIT(LOG_POSHOLD_MSG),
		};
		
		control_althold_get_break_time(break_time);
		air_res_break_get_angle_integrate(angle_integrate);
		control_poshold_get_break1_time(pos_break1_time);
		pkt.pos_mode =control_poshold_get_mode(); 
		pkt.pos_int_r = angle_integrate[0];
		pkt.pos_int_p = angle_integrate[1];
		pkt.pos_break_time_r = pos_break1_time[0];
		pkt.pos_break_time_p = pos_break1_time[1];
		pkt.alt_mode = control_althold_get_status_curr();//althold
		pkt.alt_int_r = angle_integrate[0];
		pkt.alt_int_p = angle_integrate[1];
		pkt.alt_break_time_r = break_time[0];
		pkt.alt_break_time_p = break_time[1];
		
		log_buf_write((void *)&pkt,sizeof(pkt));
	}
}

void log_rf(float rate,float dt,bool armed)
{
	static float time = 0;
	if(log_time_update(&time,rate,dt) == true && armed == true){
		struct log_RF_s pkt = {
			LOG_PACKET_DATA_HEADER_INIT(LOG_RF_MSG),
		};
		
		pkt.device_status = vl53l1x_get_status();
		pkt.status = rangefinder_get_status();
		pkt.alt = rangefinder_get_range();
		pkt.altf =rangefinder_get_range_f();
		pkt.vel = rangefinder_get_vel();
		pkt.altt = control_get_rf_alt_target();
		pkt.rotate = rangefinder_get_rotate();
		
		log_buf_write((void *)&pkt,sizeof(pkt));
	}
}

void log_circle(float rate,float dt,bool armed)
{
	static float time = 0;
	if(log_time_update(&time,rate,dt) == true && armed == true){
		struct log_CIR_s pkt = {
			LOG_PACKET_DATA_HEADER_INIT(LOG_CIR_MSG),
		};
		pkt.pos_d[0] = control_circle_vel_get_pos_sim_x();
		pkt.pos_d[1] = control_circle_vel_get_pos_sim_y();
		pkt.pos_c[0] = control_circle_vel_get_pos_cur_x();
		pkt.pos_c[1] = control_circle_vel_get_pos_cur_y();
		pkt.yaw_body_rate_c = control_circle_vel_get_body_rate_yaw_c();
		pkt.yaw_body_rate_d = control_circle_vel_get_body_rate_yaw_d();
		pkt.vel_c[0] = control_circle_vel_get_vel_x_c();
		pkt.vel_c[1] = control_circle_vel_get_vel_y_c();
		pkt.vel_d[0] = control_circle_vel_get_vel_x_d();
		pkt.vel_d[1] = control_circle_vel_get_vel_y_d();
		pkt.status = control_circle_get_status();
		log_buf_write((void *)&pkt,sizeof(pkt));
	}
}


void log_write_formats()
{
	uint8_t i;
	char buf[128];
	
	struct {
		LOG_PACKET_FORMAT_HEADER
		struct log_format_s body;
	} log_msg_format = {
		LOG_PACKET_FORMAT_HEADER_INIT(LOG_FORMAT_MSG),
	};

	for (i = 0; i < log_formats_num; i++) {
		log_msg_format.body.type = log_formats[i].type;
		log_msg_format.body.length = log_formats[i].length;

		memset(buf,0,128);
		snprintf(buf,128,"%s",log_formats[i].name);
		strncpy(log_msg_format.body.name,buf,LOG_PACKET_NAME_LEN);

		memset(buf,0,128);
		snprintf(buf,128,"%s%s","T",log_formats[i].format);
		strncpy(log_msg_format.body.format,buf,LOG_PACKET_FORMAT_LEN);

		memset(buf,0,128);
		snprintf(buf,128,"%s%s","T,",log_formats[i].labels);
		strncpy(log_msg_format.body.labels,buf,LOG_PACKET_LABELS_LEN);

		log_file_size_count += write(log_fd,(void *)&log_msg_format,sizeof(log_msg_format));
	}
}

void log_write_version()
{
	struct log_VER_s pkt = {
		LOG_PACKET_DATA_HEADER_INIT(LOG_VER_MSG),
	};

#ifdef VER
	snprintf(pkt.version,64,"%d.%d.%d",VER/100%10,VER/10%10,VER/1%10);
#else
	snprintf(pkt.version,64,"%d.%d.%d",0,0,0);
#endif

	log_file_size_count += write(log_fd,(void *)&pkt,sizeof(pkt));
}

void log_param_init()
{
	param_set_var(LOG__RUN_NAME				,&log_run);
	param_set_var(LOG__RATE_FLOW_NAME		,&log_rate_flow);
	param_set_var(LOG__RATE_SENS_NAME		,&log_rate_sens);
	param_set_var(LOG__RATE_ALT_NAME		,&log_rate_alt);
	param_set_var(LOG__RATE_ATT_NAME		,&log_rate_att);
	param_set_var(LOG__RATE_AHRS_NAME		,&log_rate_ahrs);
	param_set_var(LOG__RATE_PIDRR_NAME		,&log_rate_pidrr);
	param_set_var(LOG__RATE_PIDRP_NAME		,&log_rate_pidrp);
	param_set_var(LOG__RATE_PIDRY_NAME		,&log_rate_pidry);
	param_set_var(LOG__RATE_PIDVX_NAME		,&log_rate_pidvx);
	param_set_var(LOG__RATE_PIDVY_NAME		,&log_rate_pidvy);
	param_set_var(LOG__RATE_PIDVZ_NAME		,&log_rate_pidvz);
	param_set_var(LOG__RATE_IMU_NAME		,&log_rate_imu);
	param_set_var(LOG__RATE_BAT_NAME		,&log_rate_bat);
	param_set_var(LOG__RATE_NAV_NAME		,&log_rate_nav);
	param_set_var(LOG__RATE_NAV2_NAME		,&log_rate_nav2);
	param_set_var(LOG__RATE_NAV3_NAME		,&log_rate_nav3);
	param_set_var(LOG__RATE_NAV4_NAME		,&log_rate_nav4);
	param_set_var(LOG__RATE_RCI_NAME		,&log_rate_rci);
	param_set_var(LOG__RATE_RCO_NAME		,&log_rate_rco);
	param_set_var(LOG__RATE_BARO_NAME		,&log_rate_baro);
	param_set_var(LOG__RATE_GPS_NAME		,&log_rate_gps);
	param_set_var(LOG__RATE_SYSTEM_NAME		,&log_rate_system);
	param_set_var(LOG__RATE_POSHOLD_NAME	,&log_rate_poshold);
	param_set_var(LOG__RATE_RF_NAME			,&log_rate_rangefinder);
}

void log_init()
{
	INFO(DEBUG_ID,"init");

	get_diff_time(&log_time_start,true);
}

void log_update(float dt)
{
	bool armed = system_get_armed();

	if(log_run <= 0){
		return;
	}
	
	log_flow(log_rate_flow,dt,armed);
	log_sens(log_rate_sens,dt,armed);
	log_alt(log_rate_alt,dt,armed);
	log_att(log_rate_att,dt,armed);
	log_ahrs(log_rate_ahrs,dt,armed);
	log_pid_roll(log_rate_pidrr,dt,armed);
	log_pid_pitch(log_rate_pidrp,dt,armed);
	log_pid_yaw(log_rate_pidry,dt,armed);
	log_pid_x(log_rate_pidvx,dt,armed);
	log_pid_y(log_rate_pidvy,dt,armed);
	log_pid_z(log_rate_pidvz,dt,armed);
	log_imu(log_rate_imu,dt,armed);
	log_bat(log_rate_bat,dt,true);
	log_nav(log_rate_nav,dt,armed);
	log_nav2(log_rate_nav2,dt,armed);
	log_nav3(log_rate_nav3,dt,armed);
	log_nav4(log_rate_nav4,dt,armed);
	log_rci(log_rate_rci,dt,armed);
	log_rco(log_rate_rco,dt,armed);
	log_baro(log_rate_baro,dt,armed);
	log_gps(log_rate_gps,dt,armed);
	log_system(log_rate_system,dt,armed);
	log_poshold(log_rate_poshold,dt,armed);
	log_rf(log_rate_rangefinder,dt,armed);
	//log_circle(50,dt,armed);
	//log_flip(100,dt,armed);
	
	log_output_thread();
}

void log_set_restart()
{
	log_restart = true;
}

void card_detection(int sig_no)
{
	if(access(CARD_DEV_NODE1,F_OK) != 0){
		//INFO(DEBUG_ID,"card_detection out");
		log_set_restart();
		log_need_umount = true;
		log_tf_check = false;
	}else{
		//INFO(DEBUG_ID,"card_detection in");
		log_tf_check = true;
	}
}

int log_thread_main(void* paramter)
{
	struct timespec prev;
	float dt;
	float file_retry_time = 1.0f;
	float file_sync_time = 0.0f;

#ifdef LOG_TF_CHECK
	signal(SIGUSR2,card_detection);
#endif

	get_diff_time(&prev,true);
	while(log_run > 0 && system_get_run() == true){
		dt = get_diff_time(&prev,true);

		if(log_fd < 0){
			file_retry_time += dt;
			if(file_retry_time >= 1.0f && log_tf_check == true){
				file_retry_time = 0.0f;
				log_fd = open(LOG_PATH,O_RDWR | O_CREAT | O_TRUNC);
				if(log_fd > 0){
					INFO(DEBUG_ID,"log_thread_main ok:%s",LOG_PATH);
					log_write_formats();
					log_write_version();
					log_restart = false;
					log_file_size_count = 0;
				}
			}
		}else{
			pthread_mutex_lock(&log_buf_thread_mutex);
			if(log_thread_buf_count > 0){
				if(log_file_size_count < LOG_FILE_SIZE){
					log_file_size_count += write(log_fd,log_thread_buf,log_thread_buf_count);
					if(log_file_size_count <= 0){
						log_restart = true;
						INFO(DEBUG_ID,"log write failed,restart:%d",log_file_size_count);
					}
				}
				log_thread_buf_count = 0;
			}
			pthread_mutex_unlock(&log_buf_thread_mutex);

			file_sync_time += dt;
			if(file_sync_time >= 0.5f){
				file_sync_time = 0.0f;
				fsync(log_fd);
			}

			if(log_restart == true){
				close(log_fd);
				log_fd = -1;
				if(log_need_umount == true){
					INFO(DEBUG_ID,"card_detection umount");
					umount("/mnt");
					log_need_umount = false;
				}
			}
		}
		
		usleep(20 * 1000);
	}

	//close(log_fd);

	return 0;
}

