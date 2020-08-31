#include "hal.h"
#include "lib_math.h"

#include "app_motor.h"
#include "app_imu.h"
#include "app_mag.h"
#include "app_rc.h"
#include "app_att_control.h"
#include "app_control.h"
#include "app_mission.h"
#include "app_debug.h"
#include "app_attitude.h"
#include "event.h"
#include "app_control_followme.h"
#include "hal_aruco_linux.h"
#include "hal_infra_red.h"

#include "awlink_protocol.h"
#include "awlink_stream.h"
#include "awlink_item_system.h"
#include "pilot_steam_control.h"
#include "app_control_takeoff.h"
#include "hal_arduino.h"
#include "hal_led.h"
#define DEBUG_ID DEBUG_ID_LINK

#define RANGE 1000.0;
int tof_flag=0;
int tof_althold=0;
bool file_flag=false;
typedef struct {
	int16_t roll;
	int16_t pitch;
	int16_t yaw;
	int16_t throttle;
}awlink_control_joystick_s;

typedef struct {
	uint8_t type;
	uint8_t rate;
}awlink_control_status_rate_s;

typedef struct {
	uint8_t type;
	uint8_t command;
}awlink_control_calibrate_s;

typedef struct PACKED{
	uint8_t mode;
	float param1;
	float param2;
}awlink_control_mode_s;

typedef struct PACKED{
	double lat;
	double lon;
	int16_t yaw;
	float  alt;
	float  accuracy;
	float  vel;
	uint8_t valid;
}awlink_control_followme_s;

typedef struct PACKED{
	float att0;
	float att1;
	float att2;
}awlink_control_att_offset_s;

typedef struct PACKED{
	float time;
	float val[4];
}awlink_control_motor_test_s;

typedef struct PACKED{
	uint8_t enable;
	uint16_t yaw;
}awlink_control_headfree_s;

typedef struct PACKED{
	uint8_t status;
}awlink_control_mission_s;
typedef struct PACKED{
	uint8_t alt;
}awlink_control_tof_althold_s;
typedef struct PACKED{
	uint8_t alt;
}awlink_control_takeoff_alt_s;
typedef struct PACKED{
	uint8_t shooting_id;
}awlink_control_shooting_id;

typedef struct PACKED{
    uint8_t action;
	uint8_t id;
	int16_t roll;
	int16_t pitch;
	int16_t yaw;
}awlink_control_aruco_data;

void awlink_decode_control_mission(awlink_s * link,awlink_msg_s * msg_rev)
{
	awlink_control_mission_s * data;
	data = (awlink_control_mission_s *)msg_rev->data;
	
	mission_set_run(data->status);
}

void awlink_decode_control_headfree(awlink_s * link,awlink_msg_s * msg_rev)
{
	awlink_control_headfree_s * data;
	data = (awlink_control_headfree_s *)msg_rev->data;
	
	rc_set_headfree(data->enable);
}
int get_tof_flag()
{
   return tof_flag;
}
void set_tof_flag(int vel)
{
    tof_flag=vel;
}
int get_tof_althold()
{
  return tof_althold;
}
void set_tof_althold(int atl_v)
{
  tof_althold=atl_v;
}
void awlink_decode_control_joystick(awlink_s * link,awlink_msg_s * msg_rev)
{
#if 0
	float roll,pitch,yaw,thr;
	awlink_control_joystick_s * data;
	
	data = (awlink_control_joystick_s *)msg_rev->data;
   // printf("roll=%d pitch=%d yaw=%d thr=%d\n",data->roll,data->pitch,data->yaw,data->throttle);
	if(link->control_enable == true){
		roll = (float)data->roll / RANGE;
		pitch = -(float)data->pitch / RANGE;
		yaw = (float)data->yaw / RANGE;
		thr = (float)data->throttle/ RANGE;
		
		if( roll != 0 || pitch != 0 || yaw !=0){
		 tof_flag=1;
		 rc_awlink_set_rc(roll,pitch,yaw,thr);
			}
        else{
         tof_flag=0;
		 rc_awlink_set_rc(roll,pitch,yaw,thr);

		}
	}
#endif	

#if 1
   float roll,pitch,yaw,thr;
	awlink_control_joystick_s * data;
	
	data = (awlink_control_joystick_s *)msg_rev->data;
	//if( data->roll != 0 || data->pitch != 0 ){
   // printf("roll=%d pitch=%d yaw=%d thr=%d\n",data->roll,data->pitch,data->yaw,data->throttle);
	//	}
	if(link->control_enable == true){
         if(data->roll == 0 && data->pitch ==0 && data->yaw == 0 && data->throttle == 0){
		    roll = get_aruco_x_p()/ RANGE;
		    pitch = -get_aruco_y_p()/ RANGE;
			yaw = get_aruco_yaw_p()/ RANGE;
			thr = get_aruco_thr()/RANGE;
         	}
		 else{
            roll = (float)data->roll / RANGE;
		    pitch = -(float)data->pitch / RANGE;
            yaw = (float)data->yaw / RANGE;
			thr = (float)data->throttle/ RANGE;
			}
		
		if( roll != 0 || pitch != 0 || yaw !=0){
		 tof_flag=1;
		 rc_awlink_set_rc(roll,pitch,yaw,thr);
		 
			}
        else{
         tof_flag=0;
		 rc_awlink_set_rc(roll,pitch,yaw,thr);

		}
	}


#endif
}

void awlink_decode_control_status_rate(awlink_s * link,awlink_msg_s * msg_rev)
{
	awlink_control_status_rate_s * data;
	data = (awlink_control_status_rate_s *)msg_rev->data;

	//printf("type = %d rate = %d \n",data->type,data->rate);
	awlink_stream_set_rate(link,data->type,data->rate);
	//awlink_stream_set_rate(link,6,25); //²âÊÔ
}

void awlink_decode_control_calibrate(awlink_s * link,awlink_msg_s * msg_rev)
{
	awlink_control_calibrate_s * data;
	data = (awlink_control_calibrate_s *)msg_rev->data;
	
	if(data->type == 0){
		if(data->command == 0)
			imu_set_acc_calib(true);
		else
			imu_set_acc_calib(false);
	}else if(data->type == 1){
		if(data->command == 0){
			mag_set_calib(true);
		}else{
			mag_set_calib(false);
		}
	}
}
bool get_file_flag()
{
return file_flag;
}
void set_file_flag()
{
  file_flag=false;
}
void awlink_decode_control_mode(awlink_s * link,awlink_msg_s * msg_rev)
{
	awlink_control_mode_s * data;
	data = (awlink_control_mode_s *)msg_rev->data;	

 //  if(data->mode == 8)   //²âÊÔ ·­×ª»»³ÉÅ×·É
   
    control_set_mode(data->mode,data->param1,data->param2);
	if(data->mode == 8)
	  file_flag=true;
}

void awlink_decode_control_followme(awlink_s * link,awlink_msg_s * msg_rev)
{
	awlink_control_followme_s * data;
	data = (awlink_control_followme_s *)msg_rev->data;

	control_followme_set_target(data->lat,data->lon,data->alt,data->accuracy,data->yaw,data->vel);
}

void awlink_decode_control_motor_test(awlink_s * link,awlink_msg_s * msg_rev)
{
	awlink_control_motor_test_s * data;
	data = (awlink_control_motor_test_s *)msg_rev->data;
	
	motor_set_test(true,data->time,data->val);
}

void awlink_decode_control_att_offset(awlink_s * link,awlink_msg_s * msg_rev)
{
	//awlink_control_att_offset_s * data;
	//data = (awlink_control_att_offset_s *)msg_rev->data;
}
void awlink_decode_control_tof_althold(awlink_s * link,awlink_msg_s* msg_rev)
{

    awlink_control_tof_althold_s* data;
	data = (awlink_control_tof_althold_s*)msg_rev->data;
    tof_althold=data->alt;
}
void awlink_decode_control_takeoff_alt(awlink_s * link,awlink_msg_s * msg_rev)
{
  awlink_control_takeoff_alt_s* data;
  data=(awlink_control_takeoff_alt_s*)msg_rev->data;
  set_takeoff_alt(data->alt);
}
void awlink_decode_control_aruco_dist(awlink_s *link,awlink_msg_s *msg_rev)
{
   // struct event_data event;
    awlink_control_aruco_data* data;
	data=(awlink_control_aruco_data*)msg_rev->data;

 // printf("action = %d id = %d roll = %f pitch = %f yaw = %f \n",data->action,data->id,(float)data->roll,(float)data->pitch,(float)data->yaw);
  set_power((float)data->roll,(float)data->pitch,(float)data->yaw,data->id,data->action);

}
void awlink_decode_control_shooting(awlink_s *link,awlink_msg_s *msg_rev)
{
  awlink_control_shooting_id *data;
  data=(awlink_control_shooting_id*)msg_rev->data;
  printf("id = %d\n",data->shooting_id);
  uart_shooting(data->shooting_id);  

}
void awlink_decode_control_con_shooting_id(awlink_s *link,awlink_msg_s *msg_rev)
{ 
  awlink_control_shooting_id *data;
  data=(awlink_control_shooting_id*)msg_rev->data;
  set_marking(data->shooting_id);
  printf("set_id = %d\n",data->shooting_id);
}
void awlink_decode_control_led_blink(awlink_s *link,awlink_msg_s *msg_rev)
{

  awlink_control_shooting_id *data;
  data=(awlink_control_shooting_id*)msg_rev->data;
  if(data->shooting_id == 0){
   led_blink(0xff);
  }else{
   led_blink(1);
  }
}

void awlink_handle_control(awlink_s * link,awlink_msg_s * msg)
{
	switch(msg->subitem_id){
		case AWLINK_ITEM_CONTROL_JOYSTICK:
			awlink_decode_control_joystick(link,msg);
			break;
		case AWLINK_ITEM_CONTROL_STATUS_RATE:
			awlink_encode_system_ack(link,AWLINK_ACK_OK,AWLINK_ITEM_CONTROL,msg->subitem_id);
			awlink_decode_control_status_rate(link,msg);
			break;
		case AWLINK_ITEM_CONTROL_CALIBRATE:
			awlink_encode_system_ack(link,AWLINK_ACK_OK,AWLINK_ITEM_CONTROL,msg->subitem_id);
			awlink_decode_control_calibrate(link,msg);
			break;
		case AWLINK_ITEM_CONTROL_MODE:
			awlink_encode_system_ack(link,AWLINK_ACK_OK,AWLINK_ITEM_CONTROL,msg->subitem_id);
			awlink_decode_control_mode(link,msg);							
			break;
		case AWLINK_ITEM_CONTROL_FOLLOWME:
			awlink_decode_control_followme(link,msg);
			break;
		case AWLINK_ITEM_CONTROL_ATT_OFFSET:
			awlink_decode_control_att_offset(link,msg);
			break;	
		case AWLINK_ITEM_CONTROL_MOTOR_TEST:
			awlink_decode_control_motor_test(link,msg);
			break;		
		case AWLINK_ITEM_CONTROL_HEADFREE:
			awlink_decode_control_headfree(link,msg);
			break;
		case AWLINK_ITEM_CONTROL_MISSION:
			awlink_decode_control_mission(link,msg);
			break;
		case AWLINK_ITEM_CONTROL_TOF_ALTHOLD:
			awlink_encode_system_ack(link,AWLINK_ACK_OK,AWLINK_ITEM_CONTROL,msg->subitem_id);
            awlink_decode_control_tof_althold(link,msg);
			break;
		case AWLINK_ITEM_CONTROL_TAKEOFF_ALT:
		    awlink_decode_control_takeoff_alt(link,msg);
		    break;
		case AWLINK_ITEM_CONTROL_ARUCO_DIST:
			awlink_encode_system_ack(link,AWLINK_ACK_OK,AWLINK_ITEM_CONTROL,msg->subitem_id);
			awlink_decode_control_aruco_dist(link,msg);
			break;	
		case AWLINK_ITEM_CONTROL_SHOOTING:
			awlink_encode_system_ack(link,AWLINK_ACK_OK,AWLINK_ITEM_CONTROL,msg->subitem_id);
			awlink_decode_control_shooting(link,msg);
			break;
		case AWLINK_ITEM_CONTROL_CON_SH_id:
			awlink_encode_system_ack(link,AWLINK_ACK_OK,AWLINK_ITEM_CONTROL,msg->subitem_id);
			awlink_decode_control_con_shooting_id(link,msg);
			break;
		case AWLINK_ITEM_CONTROL_LED_SWITCH:
			awlink_encode_system_ack(link,AWLINK_ACK_OK,AWLINK_ITEM_CONTROL,msg->subitem_id);
			awlink_decode_control_led_blink(link,msg);
			break;
	}
}


