#include "app_system.h"
#include "app_pos_control.h"
#include "app_att_control.h"
#include "app_attitude.h"
#include "app_motor.h"
#include "app_rc.h"
#include "app_debug.h"
#include "app_nav.h"
#include "app_rc.h"
#include "app_air_resistance_break.h"
#include "lib_inav_flow.h"
#include "lib_inav_baro.h"

#include "app_control_common.h"
#include "app_control_land.h"
#include "app_control.h"
#include "hal.h"
#define DEBUG_ID DEBUG_ID_CONTROL

#define ALTHOLD_STATUS_MOVE  2
#define ALTHOLD_STATUS_BREAK 1
#define ALTHOLD_STATUS_STABLE 0

#define ALTHOLD_BREAK_ENABLE true

static uint8_t althold_status_curr = ALTHOLD_STATUS_STABLE;
static uint8_t althold_status_next = ALTHOLD_STATUS_STABLE;

static float air_res_break_angle = 10.0f;
static float air_res_coff = 1.2f;

static float break_time[2] = {0};
static float break_angle[2] = {0};
static float att_offset[2] = {0};

static float att_rp_limit = 0.0f;
static float att_rp_gain = 0.0f;
static float rate_rp_limit = 0.0f;

static float althold_move_break_time_check = 0.0f;

static bool alt_hold_flag = true;

bool control_althold_check()
{
	uint8_t check = CONTROL_CHECK_ATT | CONTROL_CHECK_ALT | CONTROL_CHECK_ARM;

	return control_check(check);
}

void control_althold_param_init()
{
	param_set_var(CON__ALTHOLD__ATT_RP_LIMIT_NAME,&att_rp_limit);
	param_set_var(CON__ALTHOLD__ATT_RP_GAIN_NAME,&att_rp_gain);
	param_set_var(CON__ALTHOLD__RATE_RP_LIMIT_NAME,&rate_rp_limit);
}

void control_althold_init(float param1,float param2)
{
	pos_control_set_pos_ned_z_target(nav_get_pos_ned_z());
	inav_flow_set_flip1_mode(); //because flip2 mode only chance to normal mode,not chance to action mode
	inav_flow_set_action_mode();
	inav_baro_set_action_mode();
	air_res_break_clear();	
	air_res_break_set_param(air_res_break_angle,air_res_coff);
	nav_get_att_offset(att_offset);
	alt_hold_flag = true;
	INFO(DEBUG_ID,"althold init");
}

void control_althold_exit()
{
	inav_flow_set_normal_mode();
	inav_baro_set_normal_mode();
	air_res_break_clear();				
	INFO(DEBUG_ID,"althold exit");
}

void control_althold_get_break_time(float time[2])
{
	time[0] = break_time[0];
	time[1] = break_time[1];
}

void control_althold_get_break_angle(float angle[2])
{
	angle[0] = break_angle[0];
	angle[1] = break_angle[1];
}

uint8_t control_althold_get_status_curr()
{
	return althold_status_curr;
}

void control_althold_move(float dt,rc_s * rc)
{
	float att_sp[2]={0};	

	att_sp[0] = att_offset[0] +  rc->roll  * att_rp_gain;
	att_sp[1] = att_offset[1] +  rc->pitch * att_rp_gain;
	control_att_common_limit(dt,att_sp[0],att_sp[1],att_rp_limit,rate_rp_limit);
}
void control_althold_update(float dt,rc_s * rc)
{
	nav_s pos;
	att_s att;
	
	nav_get_pos(&pos);
	attitude_get(&att);
	debug_t("join althold mode\n");
	control_yaw_common_rc(dt,rc);
	
	if(alt_hold_flag == true){
		control_alt_pos_common(dt,pos_control_get_pos_ned_z_target(),0.5f);
		if(fabs(rc->thr) > 0 || fabs(pos.pos_ned[2] - pos_control_get_pos_ned_z_target()) < 0.1f){
			alt_hold_flag = false;
		}
		if(althold_status_curr == ALTHOLD_STATUS_STABLE){
			if(alt_hold_flag == false){
				inav_baro_set_normal_mode();
			}else{
				inav_baro_set_takeoff_mode();
			}
		}		
	}else{
		control_alt_common_rc(dt,rc);
	}

	if(althold_status_next != althold_status_curr){
		switch(althold_status_next){
			case ALTHOLD_STATUS_MOVE:
				althold_move_break_time_check = 0.0f;
				inav_baro_set_action_mode();
				break;
			case ALTHOLD_STATUS_BREAK:
				inav_baro_set_action_mode();
				break;
			case ALTHOLD_STATUS_STABLE:
				air_res_break_clear();				
				inav_baro_set_normal_mode();
				break;
			default:
				break;		
		}
				
		althold_status_curr = althold_status_next;
	}
	
	switch(althold_status_curr){
		case ALTHOLD_STATUS_MOVE:
			air_res_break_angle_integrate(dt,att,att_offset);
			air_res_break_get_time(break_time);
			air_res_break_get_angle(break_angle);
			control_althold_move(dt,rc);
			if(fabs(rc->pitch) == 0 && fabs(rc->roll) == 0){
				althold_move_break_time_check += dt;
				if(althold_move_break_time_check >= 0.1f){
					althold_status_next = ALTHOLD_BREAK_ENABLE ? ALTHOLD_STATUS_BREAK : ALTHOLD_STATUS_MOVE;
				}
			}else{
				althold_move_break_time_check = 0.0f;
			}
			break;
		case ALTHOLD_STATUS_BREAK:
			
			if(break_time[0] < 0.2f){
				break_angle[0] = att_offset[0];
			}else{
				break_time[0] -= dt;
			}
			if(break_time[1] < 0.2f){
				break_angle[1] = att_offset[1];
			}else{
				break_time[1] -= dt;
			}
			
			air_res_break_angle_integrate(dt,att,att_offset);

			control_att_common(dt,break_angle[0],break_angle[1],rate_rp_limit);
			
			if(fabs(rc->pitch) > 0 || fabs(rc->roll) > 0){
				althold_status_next = ALTHOLD_STATUS_MOVE;
			}
			if(break_time[0] < 0.2f && break_time[1] < 0.2f){
				althold_status_next  = ALTHOLD_STATUS_STABLE;
			}
			break;
		case ALTHOLD_STATUS_STABLE:
			control_att_common(dt,att_offset[0],att_offset[1],0.5f);

			if(fabs(rc->pitch) > 0 || fabs(rc->roll) > 0){
				althold_status_next = ALTHOLD_STATUS_MOVE;
			}
			break;
		default:
			break;

	}
	
	if(rc->thr_raw < 0){
		bool check = false;
		if(control_land_update_land_check_thr(dt) == true){
			check = true;
		}

		//if(control_land_update_land_check_acc(dt) == true){
		//	check = true;
		//}

		if(check == true){
			control_set_mode(CONTROL_MODE_STOP,0,0);
		}
	}else{
		control_land_update_land_check_thr_reset();
	}
}

