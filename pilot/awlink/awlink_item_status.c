#include <string.h>

#include "hal.h"
#include "lib_inav.h"
#include "hal_aruco_linux.h"
#include "app_rc.h"

#include "app_control_takeoff.h"
#include "app_control_poshold.h"
#include "app_log.h"
#include "app_pos_control.h"
#include "app_att_control.h"
#include "app_attitude.h"
#include "app_nav.h"
#include "app_motor.h"
#include "app_system.h"
#include "app_imu.h"
#include "app_mag.h"
#include "app_rangefinder.h"
#include "app_baro.h"
#include "app_flow.h"
#include "app_gps.h"
#include "app_rc.h"
#include "app_control.h"
#include "app_batt.h"
#include "app_debug.h"
#include "app_imu_calib.h"
#include "app_mission.h"
#include "hal_stm8s.h"
#include "awlink_protocol.h"
#include "awlink_encode.h"
#include "awlink_item_control.h"
#include "awlink_item_system.h"
#include "hal_arduino.h"
#include "hal_tof_vl53l1x.h"

#define AWLINK_STATUS_USER_NAME_INFO_LEN 6

typedef struct PACKED {
	float att[3];
	float vel_ned[3];
	float pos_ned[3];
	uint8_t status;
	uint8_t mode;
	uint8_t capacity;
	uint8_t voltage;
	bool  charge;
	bool  headfree;
	bool  armed;
}awlink_status_base_info_s;

typedef struct PACKED {
	double lat;
	double lon;
	uint8_t eph;
	uint8_t satellites;
	uint8_t fix_type;
}awlink_status_gps_info_s;

typedef struct PACKED {
	uint8_t acc;
	uint8_t gyro;
	uint8_t mag;
	uint8_t baro;
	uint8_t gps;
	uint8_t flow;
}awlink_status_sensor_info_s;

typedef struct PACKED {
	float acc[3];
	float gyro[3];
	float imu_temp;
	float flow_x;
	float flow_y;
	float flow_q;
	float baro_alt;
	//float baro_pressure;
	float tof_alt;
	float baro_temp;
}awlink_status_mp_info_s;

typedef struct PACKED {
	char label[AWLINK_STATUS_USER_NAME_INFO_LEN];
	char data1[AWLINK_STATUS_USER_NAME_INFO_LEN];
	char data2[AWLINK_STATUS_USER_NAME_INFO_LEN];
	char data3[AWLINK_STATUS_USER_NAME_INFO_LEN];
	char data4[AWLINK_STATUS_USER_NAME_INFO_LEN];
	char data5[AWLINK_STATUS_USER_NAME_INFO_LEN];
	char data6[AWLINK_STATUS_USER_NAME_INFO_LEN];
	char data7[AWLINK_STATUS_USER_NAME_INFO_LEN];
	char data8[AWLINK_STATUS_USER_NAME_INFO_LEN];
	char data9[AWLINK_STATUS_USER_NAME_INFO_LEN];
}awlink_status_user_name_info_s;

typedef struct PACKED {
	float data1;
	float data2;
	float data3;
	float data4;
	float data5;
	float data6;
	float data7;
	float data8;
	float data9;
}awlink_status_user_info_s;

typedef struct PACKED {
	float data1;
	float data2;
	float data3;
	float data4;
	float data5;
}awlink_aruco_user_info_s;

typedef struct PACKED {
	uint8_t acc;
	uint8_t mag;
}awlink_status_sensor_calib_info_s;

typedef struct PACKED {
	uint8_t status;
	float time;
	uint16_t total;
	uint16_t count;
	uint8_t type;
}awlink_status_mission_info_s;
//static uint8_t motor_val_t[8] = {0};

void awlink_encode_status_base_info(awlink_s * link)
{
	uint8_t mode = 0;
	awlink_msg_s msg;
	awlink_status_base_info_s data;	
	att_s att;
	nav_s pos;

	attitude_get(&att);
	nav_get_pos(&pos);

	v3f_set(data.att,att.att);
	v3f_set(data.pos_ned,pos.pos_ned);
	v3f_set(data.vel_ned,pos.vel_ned);
	
	data.status = 0;
	if(get_takeoff_flag_poshold()== true){
     mode=CONTROL_MODE_TAKEOFF;
	 if(control_get_mode() == CONTROL_MODE_STOP){
     mode=CONTROL_MODE_STOP;
	 set_takeoff_flag_poshold();
	 }
     if((get_pt_flag() == true)){
	  if((get_tof_althold() == 0)){
      mode=control_get_mode();
	  set_pt_flag();
	  set_takeoff_flag_poshold();
	  }
	 } 
	}else{
	 mode = control_get_mode();
	}
	data.mode = mode;
	data.headfree = rc_get_headfree();
	data.capacity = batt_get_cap();
	data.voltage = batt_get_vol();
	data.charge = batt_get_charge();
	data.armed = system_get_armed();
	
	msg.item_id = AWLINK_ITEM_STATUS;
	msg.subitem_id = AWLINK_ITEM_STATUS_BASIC_INFO;
	msg.data = (uint8_t *)&data;
	msg.length = sizeof(data);

	awlink_encode(link,&msg,false,false);
}

void awlink_encode_status_gps_info(awlink_s * link)
{
	awlink_msg_s msg;
	awlink_status_gps_info_s data;
	gps_info_s gps;

	gps_get_info(&gps);
	data.lat = gps.lat;
	data.lon = gps.lon;
	data.eph = gps.eph;
	data.satellites = gps.satellites_used;
	data.fix_type = gps.fix_type;

	msg.item_id = AWLINK_ITEM_STATUS;
	msg.subitem_id = AWLINK_ITEM_STATUS_GPS_INFO;
	msg.data = (uint8_t *)&data;
	msg.length = sizeof(data);

	awlink_encode(link,&msg,false,false);
}

void awlink_encode_status_sensor_info(awlink_s * link)
{
	awlink_msg_s msg;
	awlink_status_sensor_info_s data;

	data.acc = imu_get_acc_status();
	data.gyro = imu_get_gyro_status();
	data.mag = mag_get_status();
	data.baro = baro_get_status();
	data.gps = gps_get_status();
	data.flow = flow_get_status();
	msg.item_id = AWLINK_ITEM_STATUS;
	msg.subitem_id = AWLINK_ITEM_STATUS_SENSOR_INFO;
	msg.data = (uint8_t *)&data;
	msg.length = sizeof(data);

	awlink_encode(link,&msg,false,false);
}

void awlink_encode_status_sensor_calib_info(awlink_s * link)
{
	awlink_msg_s msg;
	awlink_status_sensor_calib_info_s data;
	uint8_t status=0;
	
	status = imu_calib_acc_get_progress();
	data.acc = status;
	status = mag_get_calib_progress();
	data.mag = status;
	
	msg.item_id = AWLINK_ITEM_STATUS;
	msg.subitem_id = AWLINK_ITEM_STATUS_SENSOR_CALIB_INFO;
	msg.data = (uint8_t *)&data;
	msg.length = sizeof(data);

	awlink_encode(link,&msg,false,false);
}

void awlink_encode_status_user1_info(awlink_s * link)
{
	awlink_msg_s msg;
	awlink_aruco_user_info_s data;
    
	data.data1 = 0;
	data.data2 = 0;
	data.data3 = 0;
	data.data4 = 0;
	data.data5 = 0;
	
	msg.item_id = AWLINK_ITEM_STATUS;
	msg.subitem_id = AWLINK_ITEM_STATUS_USER1_INFO;
	msg.data = (uint8_t *)&data;
	msg.length = sizeof(data);

	awlink_encode(link,&msg,false,false);
}

void awlink_encode_status_user2_info(awlink_s * link)
{
	awlink_msg_s msg;
	awlink_status_user_info_s data;

	//float motor_output[4];
	//motor_output_get(motor_output); 
	//stm8s_read(motor_val_t,6);

	data.data1 = 0;
	data.data2 =0;
	data.data3 = 0;
	data.data4 = 0;
	data.data5 = 0;
	data.data6 = 0;
	data.data7 =0;	
	data.data8 = 50.0f;
	data.data9 = 60.0f;
	
	msg.item_id = AWLINK_ITEM_STATUS;
	msg.subitem_id = AWLINK_ITEM_STATUS_USER2_INFO;
	msg.data = (uint8_t *)&data;
	msg.length = sizeof(data);

	awlink_encode(link,&msg,false,false);
}

void awlink_encode_status_user3_info(awlink_s * link)
{
	awlink_msg_s msg;
	awlink_status_user_info_s data;
	
	data.data1 = 0.0f;
	data.data2 = 0.0f;
	data.data3 = 0.0f;
	data.data4 = 0.0f;
	data.data5 = 0.0f;
	data.data6 = 0.0f;
	data.data7 = 0.0f;	
	data.data8 = 0.0f;
	data.data9 = 0.0f;
	
	msg.item_id = AWLINK_ITEM_STATUS;
	msg.subitem_id = AWLINK_ITEM_STATUS_USER3_INFO;
	msg.data = (uint8_t *)&data;
	msg.length = sizeof(data);

	awlink_encode(link,&msg,false,false);
}

void awlink_encode_status_user4_info(awlink_s * link)
{
	awlink_msg_s msg;
	awlink_status_user_info_s data;
	
	data.data1 = 0.0f;
	data.data2 = 0.0f;
	data.data3 = 0.0f;
	data.data4 = 0.0f;
	data.data5 = 0.0f;
	data.data6 = 0.0f;
	data.data7 = 0.0f;	
	data.data8 = 0.0f;
	data.data9 = 0.0f;
	
	msg.item_id = AWLINK_ITEM_STATUS;
	msg.subitem_id = AWLINK_ITEM_STATUS_USER4_INFO;
	msg.data = (uint8_t *)&data;
	msg.length = sizeof(data);

	awlink_encode(link,&msg,false,false);
}

void awlink_encode_status_mission_info(awlink_s * link)
{
	awlink_msg_s msg;
	awlink_status_mission_info_s data;
	
	data.status = mission_get_run_status();
	data.time = mission_get_time();
	data.total = mission_get_total();
	data.count = mission_get_run_count();
	data.type = mission_get_run_type();
	
	msg.item_id = AWLINK_ITEM_STATUS;
	msg.subitem_id = AWLINK_ITEM_STATUS_MISSION_INFO;
	msg.data = (uint8_t *)&data;
	msg.length = sizeof(data);

	awlink_encode(link,&msg,false,false);
}

void awlink_encode_status_mp_info(awlink_s * link)
{
	awlink_msg_s msg;
	awlink_status_mp_info_s data;

	float acc[3];
	float gyro[3];
	float flow_vel[2];

	imu_get_acc_raw(acc);
	imu_get_gyro(gyro);
	flow_get_vel(flow_vel);
	
	data.acc[0] = acc[0];
	data.acc[1] = acc[1];
	data.acc[2] = acc[2];
	data.gyro[0] = gyro[0];
	data.gyro[1] = gyro[1];
	data.gyro[2] = gyro[2];
	data.imu_temp = imu_get_temp();
	
	data.flow_x = flow_vel[0];
	data.flow_y = flow_vel[1];
	data.flow_q = flow_get_quality();

	data.baro_alt = baro_get_alt();
	//data.baro_pressure = baro_get_pres();
    data.tof_alt= get_tof_data_yaw();
	data.baro_temp = baro_get_temp();
	
	msg.item_id = AWLINK_ITEM_STATUS;
	msg.subitem_id = AWLINK_ITEM_STATUS_MP_INFO;
	msg.data = (uint8_t *)&data;
	msg.length = sizeof(data);

	awlink_encode(link,&msg,false,false);
}

void awlink_encode_status_user1_name_info(awlink_s * link,awlink_msg_s * msg_rev)
{
	awlink_msg_s msg;
	awlink_status_user_name_info_s data;

	strncpy(data.label,"user1",AWLINK_STATUS_USER_NAME_INFO_LEN);

	strncpy(data.data1,"data1",AWLINK_STATUS_USER_NAME_INFO_LEN);
	strncpy(data.data2,"data2",AWLINK_STATUS_USER_NAME_INFO_LEN);
	strncpy(data.data3,"data3",AWLINK_STATUS_USER_NAME_INFO_LEN);
	strncpy(data.data4,"data4",AWLINK_STATUS_USER_NAME_INFO_LEN);
	strncpy(data.data5,"data5",AWLINK_STATUS_USER_NAME_INFO_LEN);
	strncpy(data.data6,"data6",AWLINK_STATUS_USER_NAME_INFO_LEN);
	strncpy(data.data7,"data7",AWLINK_STATUS_USER_NAME_INFO_LEN);
	strncpy(data.data8,"data8",AWLINK_STATUS_USER_NAME_INFO_LEN);
	strncpy(data.data9,"data9",AWLINK_STATUS_USER_NAME_INFO_LEN);

	msg.item_id = AWLINK_ITEM_STATUS;
	msg.subitem_id = AWLINK_ITEM_STATUS_USER1_NAME_INFO;
	msg.data = (uint8_t *)&data;
	msg.length = sizeof(data);

	awlink_encode(link,&msg,false,true);
}

void awlink_encode_status_user2_name_info(awlink_s * link,awlink_msg_s * msg_rev)
{
	awlink_msg_s msg;
	awlink_status_user_name_info_s data;

	strncpy(data.label,"user2",AWLINK_STATUS_USER_NAME_INFO_LEN);

	strncpy(data.data1,"data1",AWLINK_STATUS_USER_NAME_INFO_LEN);
	strncpy(data.data2,"data2",AWLINK_STATUS_USER_NAME_INFO_LEN);
	strncpy(data.data3,"data3",AWLINK_STATUS_USER_NAME_INFO_LEN);
	strncpy(data.data4,"data4",AWLINK_STATUS_USER_NAME_INFO_LEN);
	strncpy(data.data5,"data5",AWLINK_STATUS_USER_NAME_INFO_LEN);
	strncpy(data.data6,"data6",AWLINK_STATUS_USER_NAME_INFO_LEN);
	strncpy(data.data7,"data7",AWLINK_STATUS_USER_NAME_INFO_LEN);
	strncpy(data.data8,"data8",AWLINK_STATUS_USER_NAME_INFO_LEN);
	strncpy(data.data9,"data9",AWLINK_STATUS_USER_NAME_INFO_LEN);

	msg.item_id = AWLINK_ITEM_STATUS;
	msg.subitem_id = AWLINK_ITEM_STATUS_USER2_NAME_INFO;
	msg.data = (uint8_t *)&data;
	msg.length = sizeof(data);

	awlink_encode(link,&msg,false,true);
}

void awlink_encode_status_user3_name_info(awlink_s * link,awlink_msg_s * msg_rev)
{
	awlink_msg_s msg;
	awlink_status_user_name_info_s data;

	strncpy(data.label,"user3",AWLINK_STATUS_USER_NAME_INFO_LEN);

	strncpy(data.data1,"data1",AWLINK_STATUS_USER_NAME_INFO_LEN);
	strncpy(data.data2,"data2",AWLINK_STATUS_USER_NAME_INFO_LEN);
	strncpy(data.data3,"data3",AWLINK_STATUS_USER_NAME_INFO_LEN);
	strncpy(data.data4,"data4",AWLINK_STATUS_USER_NAME_INFO_LEN);
	strncpy(data.data5,"data5",AWLINK_STATUS_USER_NAME_INFO_LEN);
	strncpy(data.data6,"data6",AWLINK_STATUS_USER_NAME_INFO_LEN);
	strncpy(data.data7,"data7",AWLINK_STATUS_USER_NAME_INFO_LEN);
	strncpy(data.data8,"data8",AWLINK_STATUS_USER_NAME_INFO_LEN);
	strncpy(data.data9,"data9",AWLINK_STATUS_USER_NAME_INFO_LEN);

	msg.item_id = AWLINK_ITEM_STATUS;
	msg.subitem_id = AWLINK_ITEM_STATUS_USER3_NAME_INFO;
	msg.data = (uint8_t *)&data;
	msg.length = sizeof(data);

	awlink_encode(link,&msg,false,true);
}

void awlink_encode_status_user4_name_info(awlink_s * link,awlink_msg_s * msg_rev)
{
	awlink_msg_s msg;
	awlink_status_user_name_info_s data;

	strncpy(data.label,"user4",AWLINK_STATUS_USER_NAME_INFO_LEN);

	strncpy(data.data1,"data1",AWLINK_STATUS_USER_NAME_INFO_LEN);
	strncpy(data.data2,"data2",AWLINK_STATUS_USER_NAME_INFO_LEN);
	strncpy(data.data3,"data3",AWLINK_STATUS_USER_NAME_INFO_LEN);
	strncpy(data.data4,"data4",AWLINK_STATUS_USER_NAME_INFO_LEN);
	strncpy(data.data5,"data5",AWLINK_STATUS_USER_NAME_INFO_LEN);
	strncpy(data.data6,"data6",AWLINK_STATUS_USER_NAME_INFO_LEN);
	strncpy(data.data7,"data7",AWLINK_STATUS_USER_NAME_INFO_LEN);
	strncpy(data.data8,"data8",AWLINK_STATUS_USER_NAME_INFO_LEN);
	strncpy(data.data9,"data9",AWLINK_STATUS_USER_NAME_INFO_LEN);

	msg.item_id = AWLINK_ITEM_STATUS;
	msg.subitem_id = AWLINK_ITEM_STATUS_USER4_NAME_INFO;
	msg.data = (uint8_t *)&data;
	msg.length = sizeof(data);

	awlink_encode(link,&msg,false,true);
}

void awlink_handle_status(awlink_s * link,awlink_msg_s * msg)
{
	switch(msg->subitem_id){
		case AWLINK_ITEM_STATUS_USER1_NAME_INFO:
			awlink_encode_status_user1_name_info(link,msg);
			break;
		case AWLINK_ITEM_STATUS_USER2_NAME_INFO:
			awlink_encode_status_user2_name_info(link,msg);
			break;
		case AWLINK_ITEM_STATUS_USER3_NAME_INFO:
			awlink_encode_status_user3_name_info(link,msg);
			break;
		case AWLINK_ITEM_STATUS_USER4_NAME_INFO:
			awlink_encode_status_user4_name_info(link,msg);
			break;
	}
}


