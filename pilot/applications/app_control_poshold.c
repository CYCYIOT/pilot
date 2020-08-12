#include "app_rc.h"
#include "app_debug.h"
#include "app_motor.h"
#include "app_nav.h"
#include "app_param.h"
#include "app_attitude.h"
#include "app_system.h"
#include "app_control.h"
#include "app_pos_control.h"
#include "app_att_control.h"
#include "app_control.h"
#include "app_control_common.h"
#include "app_control_land.h"
#include "app_flow.h"
#include "app_baro.h"
#include "app_air_resistance_break.h"
#include "lib_inav_flow.h"
#include "lib_inav_baro.h"
#include "lib_math.h"
#include "hal.h"
#include "hal_tof_vl53l1x.h"
#include "awlink_item_control.h"
#include "app_control_takeoff.h"

#define DEBUG_ID DEBUG_ID_CONTROL

#define STATUS_IMPACT   0
#define STATUS_STABLE   1
#define STATUS_BREAK2   2
#define STATUS_BREAK1   3
#define STATUS_MOVE     4
#define STATUS_ROTATE   5

#define LOW_SPEED_MODE  0
#define HIGH_SPEED_MODE 1

#define MAX_TIME        1.0f

static float air_res_break_angle;
static float air_res_coff;
static float ta_vel_limit;

static float att_rp_gain;
static float rate_rp_limit = 0.0f;

float break1_time[2] ={0.0f};
float break1_angle[2] = {0.0f};

float break2_time = 0.0f;
bool break2_vel_flag[2] = {false} ;
float break2_i_pop_time[2] = {0};
bool break2_mode = HIGH_SPEED_MODE;
	
uint8_t status_curr = STATUS_STABLE;
uint8_t status_next = STATUS_STABLE;

float control_pos_sp[3] = {0.0f,0.0f,0.0f};
float contorl_poshold_att_offset[2]= {0.0f,0.0f};

int contorl_status = STATUS_STABLE;

static bool alt_hold_flag = true;
static float poshold_move_break_time_check = 0.0f;
	
void control_poshold_set_break2_mode(int i)
{
	break2_mode = i==0 ? LOW_SPEED_MODE : HIGH_SPEED_MODE;
	INFO(DEBUG_ID,"break2_mode:%d",break2_mode);
}

bool control_poshold_get_break2_mode()
{
	return break2_mode;
}

uint8_t control_poshold_get_mode()
{
	return status_curr;
}

void control_poshold_get_break1_time(float time[2])
{
	time[0] = break1_time[0];
	time[1] = break1_time[1];
}

bool control_poshold_check()
{
	uint8_t check = CONTROL_CHECK_ATT | CONTROL_CHECK_ALT | CONTROL_CHECK_VEL | CONTROL_CHECK_ARM;

	return control_check(check);
}

void control_poshold_param_init()
{
	param_set_var(CON__POSHOLD__AIR_RES_COFF_NAME,&air_res_coff);
	param_set_var(CON__POSHOLD__AIR_RES_BREAK_ANGLE_NAME,&air_res_break_angle);
	param_set_var(CON__TAKEOFF__VEL_LIMIT_NAME	,&ta_vel_limit);

	param_get(CON__COMM__ATT_RP_GAIN_NAME,&att_rp_gain);
	param_get(CON__COMM__RATE_RP_LIMIT_NAME,&rate_rp_limit);

}

void control_poshold_break_reset()
{
	break1_time[0] = 0;
	break1_time[1] = 0;
	break2_time = 0;
	break2_vel_flag[0] = false;
	break2_vel_flag[1] = false;
	break2_i_pop_time[0] = 0;
	break2_i_pop_time[1] = 0;
}

void control_poshold_exit()
{
	pos_control_set_vel_pid_normal_mode();
	att_control_set_att_pid_normal_mode();
	control_poshold_break_reset();
	inav_flow_set_normal_mode();
	inav_baro_set_normal_mode();
	inav_baro_set_noise_enable(true);
	baro_set_baro_vel_enable(false);
	inav_flow_set_quality_fuse_enable(false);
	status_curr = STATUS_STABLE;
	status_next = STATUS_STABLE;
	air_res_break_clear();

	INFO(DEBUG_ID,"poshold exit");
}

void control_poshold_init(float param1,float param2)
{
	nav_s pos;

	nav_get_pos(&pos);
	control_poshold_break_reset();
	v3f_set(control_pos_sp,pos.pos_ned);
	pos_control_set_vel_pid_normal_mode();
	att_control_set_att_pid_normal_mode();
	baro_set_baro_vel_enable(false);
	inav_flow_set_normal_mode();
	inav_baro_set_normal_mode();
	nav_get_att_offset(contorl_poshold_att_offset);
	air_res_break_clear();	
	air_res_break_set_param(air_res_break_angle,air_res_coff);
	alt_hold_flag = true;
	status_curr = STATUS_STABLE;
	status_next = STATUS_STABLE;
	INFO(DEBUG_ID,"poshold init");
}

void control_poshold_stable_status(float dt,rc_s * rc,nav_s pos,att_s att)
{
	float vel_sp[3] = {0};

	if(pos.pos_valid == true){
		control_pos_ned_xy_common(dt,control_pos_sp,0,0.7f);
	}else if(pos.vel_valid == true){
		control_vel_ned_xy_common(dt,vel_sp,0,0.7f);
	}else{
		control_att_common_rc(dt,rc);
	}
	nav_att_offset_update();
	nav_get_att_offset(contorl_poshold_att_offset);
}

void control_poshold_rotate_status(float dt,rc_s * rc,nav_s pos,att_s att)
{
	float vel_sp[3] = {0};
	
	control_vel_ned_xy_common(dt,vel_sp,0,0.7f);
}

void control_poshold_impact_status(float dt,rc_s * rc,nav_s pos,att_s att)
{
	control_att_common_rc(dt,rc);
}

void control_poshold_move_status(float dt,rc_s * rc,nav_s pos,att_s att)
{
	float att_sp[3]={0};	
	
	att_sp[0] = contorl_poshold_att_offset[0] +  rc->roll  * att_rp_gain;
	att_sp[1] = contorl_poshold_att_offset[1] +  rc->pitch * att_rp_gain;
	control_att_common(dt,att_sp[0],att_sp[1],rate_rp_limit);
}

void control_poshold_break1_status(float dt,rc_s * rc,nav_s pos,att_s att)
{	
	if(break1_time[0] < 0.1f){
		break1_angle[0] = contorl_poshold_att_offset[0];
	}else{
		break1_time[0] -= dt;
	}
	if(break1_time[1] < 0.1f){
		break1_angle[1] = contorl_poshold_att_offset[1];
	}else{
		break1_time[1] -= dt;
	}

	control_att_common(dt,break1_angle[0],break1_angle[1],0.7f);
}

void control_poshold_break2_status(float dt,rc_s * rc,nav_s pos,att_s att)
{
	float vel_sp[3]={0};
	float rate_limit;
	float vel_length;
/*	
//	don't go to stable mode when wind in door.
	
	if(fabs(pos.vel_grd[0]) > 0.2f){
		break2_vel_flag[0] = false;
		break2_i_pop_time[0] = 0; 
		break2_time = 0;
	}else if(fabs(pos.vel_grd[0]) < 0.1f){
		break2_vel_flag[0] = true;
	}
	
	if(fabs(pos.vel_grd[1]) > 0.2f){
		break2_vel_flag[1] = false;
		break2_i_pop_time[1] = 0; 
		break2_time = 0;
	}else if(fabs(pos.vel_grd[1]) < 0.1f){
		break2_vel_flag[1] = true;
	}
*/
	if(fabs(pos.vel_grd[0]) < 0.1f && fabs(att_control_get_target_att_pitch() - att.att[1]) < 2.0f){
		   break2_vel_flag[0] = true;
	}
	
	if(fabs(pos.vel_grd[1]) < 0.1f && fabs(att_control_get_target_att_roll() - att.att[0]) < 2.0f){
		   break2_vel_flag[1] = true;
	}

	if(break2_vel_flag[0] == true && break2_i_pop_time[0] <= 0.3f){
		vel_control_xy_i_pop(0,1);
		break2_i_pop_time[0] += dt;
	}
	
	if(break2_vel_flag[1] == true && break2_i_pop_time[1] <= 0.3f){
		vel_control_xy_i_pop(1,1);
		break2_i_pop_time[1] += dt;
	}
	
	vel_length = pythagorous2(pos.vel_ned[0],pos.vel_ned[1]);
	if(vel_length < 0.03f && break2_vel_flag[0] == true && break2_vel_flag[1] == true){
		break2_time += dt;
	}
	
	if(break2_mode == LOW_SPEED_MODE){
		rate_limit = 0.5f;
		pos_control_set_vel_pid_break_low_speed_mode();
	}else{
		rate_limit = 1.0f;
		pos_control_set_vel_pid_break_high_speed_mode();
	}
	
	control_vel_ned_xy_common(dt,vel_sp,0,rate_limit);
}

void control_poshold_status_exit()
{
	if(status_next != status_curr){
		inav_flow_set_normal_mode();
		inav_baro_set_normal_mode();
		baro_set_baro_vel_enable(false);
		inav_baro_set_noise_enable(true);
		pos_control_set_vel_pid_normal_mode();
		att_control_set_att_pid_normal_mode();
		inav_flow_set_quality_fuse_enable(false);
	}
}

void control_poshold_status_init(nav_s pos)
{
	if(status_next != status_curr){
		status_curr = status_next;
		switch(status_curr){
			case STATUS_STABLE:
				control_poshold_break_reset();
				v3f_set(control_pos_sp,pos.pos_ned);
				break;
			case STATUS_IMPACT:
				inav_flow_set_action_mode();
				pos_control_xy_reinit();
				break;
			case STATUS_MOVE:
			//	pos_control_set_pos_ned_z_target(pos.pos_ned[2]);
				inav_flow_set_action_mode();
				inav_flow_set_quality_fuse_enable(true);
				baro_set_baro_vel_enable(true);
				poshold_move_break_time_check = 0.0f;
				break;
			case STATUS_BREAK1: 
				inav_flow_set_action_mode();
				inav_baro_set_break_mode();
				inav_flow_set_quality_fuse_enable(true);
				baro_set_baro_vel_enable(true);
			//	inav_baro_set_noise_enable(false);
				break;
			case STATUS_BREAK2:
				nav_recover_acc_bias();
				control_poshold_break_reset();
				inav_flow_set_action_mode();
				inav_flow_set_quality_fuse_enable(true);
				inav_baro_set_break_mode();
				vel_control_xy_i_pop(0,0);
				vel_control_xy_i_pop(1,0);
				baro_set_baro_vel_enable(true);
			//	inav_baro_set_noise_enable(false);
				break;
			case STATUS_ROTATE:
				nav_backup_acc_bias();
				inav_flow_set_rotate_mode();
			//   pos_control_set_vel_pid_rotate_mode();
				break;
			default:
				break;
		}
	}
}

static int count_tf = 0;
static bool take_flag_p =true;
static bool pt_flag = false;

void set_pt_flag()
{
  pt_flag=false;
}
bool get_pt_flag()
{
 return pt_flag;
}
void control_poshold_update(float dt,rc_s * rc)
{
	nav_s pos;
	att_s att;
	
	nav_get_pos(&pos);
	attitude_get(&att);
	
	control_yaw_common_rc(dt,rc);

	if(alt_hold_flag == true){
		control_alt_pos_common(dt,pos_control_get_pos_ned_z_target(),ta_vel_limit);
		
       // debug_t("poshhold pos.pos_ned[2] = %f target_alt = %f tof = %f\n",pos.pos_ned[2],pos_control_get_pos_ned_z_target(),get_tof_data_yaw());
		if(fabs(rc->thr) > 0 || fabs(pos.pos_ned[2] - pos_control_get_pos_ned_z_target()) < 0.1f || (control_get_mode_last() == CONTROL_MODE_THROWN)){
			alt_hold_flag = false;	
			debug_t("join poshold\n");
		}
		if(status_curr == STATUS_STABLE){
			if(alt_hold_flag == false){
				inav_flow_set_normal_mode();
				inav_baro_set_normal_mode();
				take_flag_p=false;
			}else{
			   if(control_get_mode_last() == CONTROL_MODE_THROWN)
			   	{
			   	  inav_flow_set_thrown_mode();
				 
			   	}else{
                  inav_flow_set_takeoff_mode();
				}
				
				inav_baro_set_takeoff_mode();
				
			}
		}	
	}else{
		control_alt_common_rc(dt,rc);
	}

    if(control_get_mode_last() == CONTROL_MODE_TAKEOFF && get_takeoff_flag_poshold()== true){      //起飞完成后定高一次
     if(count_tf++ > 1000){
	 set_tof_althold(100); 
	 pt_flag=true;
	 count_tf=0;
     }
	}
	control_poshold_status_exit();
	control_poshold_status_init(pos);
	
	switch(status_curr){
		case STATUS_STABLE: 
			control_poshold_stable_status(dt,rc,pos,att);
			if(fabs(att.rate[0]) > 2.5f || fabs(att.rate[1]) > 2.5f || fabs(att.rate[2]) > 1.5f ||
		   		fabs(att.att[0]) > 10.0f || fabs(att.att[1]) > 10.0f){
		   			status_next = STATUS_IMPACT;
			}
			break;
		case STATUS_IMPACT: 
			control_poshold_impact_status(dt,rc,pos,att);
			if(fabs(att.rate[0]) < 0.3f && fabs(att.rate[1]) < 0.3f && fabs(att.rate[2]) < 1.0f &&
			   fabs(att.att[0]) < 5.0f && fabs(att.att[1]) < 5.0f ){
			  		status_next = STATUS_BREAK2;
			}
			break;
		case STATUS_MOVE: 
			control_poshold_move_status(dt,rc,pos,att);
			air_res_break_angle_integrate(dt,att,contorl_poshold_att_offset);
			air_res_break_get_angle(break1_angle);
			air_res_break_get_time(break1_time);
			if(fabs(rc->roll) == 0 && fabs(rc->pitch) == 0){
				poshold_move_break_time_check += dt;
				if(poshold_move_break_time_check >= 0.1f){
					if(flow_get_quality_f() > 0.3f){
						status_next = STATUS_BREAK2;
					}else{
						status_next = STATUS_BREAK1;
					}
				}
			}else{
				poshold_move_break_time_check = 0.0f;
			}
			break;
		case STATUS_BREAK1: 
			air_res_break_angle_integrate(dt,att,contorl_poshold_att_offset);
			control_poshold_break1_status(dt,rc,pos,att);
			if((break1_time[0] < 0.1f && break1_time[1] < 0.1f) || flow_get_quality_f() > 0.5f){
				status_next  = STATUS_BREAK2;
			}

			break;
		case STATUS_BREAK2: 	
			control_poshold_break2_status(dt,rc,pos,att);
			air_res_break_angle_integrate(dt,att,contorl_poshold_att_offset);
			if(fabs(pos.vel_grd[0]) < 0.3f && fabs(pos.vel_grd[1]) < 0.3f){
				air_res_break_clear();	
			}
			
			if(break2_time > 0.2f){
				status_next = STATUS_STABLE;
			}

			break;
		case STATUS_ROTATE:
			control_poshold_rotate_status(dt,rc,pos,att);
			if(fabs(rc->yaw) == 0.0f){
				status_next = STATUS_BREAK2;
			}
			break;
		default: 
			break;
	}

	if(fabs(rc->yaw) > 0.05f){
		status_next = STATUS_ROTATE;
	}

	if(fabs(rc->roll) > 0 || fabs(rc->pitch) > 0){
		status_next = STATUS_MOVE;
	}
	
	if(rc->thr < 0){
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



