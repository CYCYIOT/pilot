#include "app_rc.h"
#include "app_debug.h"
#include "app_imu.h"
#include "app_nav.h"
#include "app_param.h"
#include "app_attitude.h"
#include "app_system.h"
#include "app_control.h"
#include "app_att_control.h"
#include "app_pos_control.h"
#include "app_control_common.h"
#include "hal_aruco_linux.h"
#include "hal.h"
#include "lib_math.h"


#define RTH_RC_ALT_DEADZONE 0.15f
#define DIS_CENTER   0.1
typedef enum{
	ARUCO_STATUS_IDLE = 0,
	ARUCO_STATUS_GO_HOME,
	ARUCO_STATUS_LANDING,
	ARUCO_STATUS_LANDED,
}aruco_status_e;

#define DEBUG_ID DEBUG_ID_CONTROL
#define RADIUS_ARUCO 	0.1f
static float dt_sum_timeout = 0;
static float dt_sum = 0;

static aruco_status_e aruco_status;
static nav_s pos_curr;

static float pos_sp[3] = {0.0f};

#define RTH_LAND_ACC_CHECK_THRESHOLD   (-2.0f * 9.8f)   // m/s^2
#define RTH_LAND_VEL_CHECK_THRESHOLD   (0.3f)
#define RTH_LAND_VEL_CHECK_TIMEOUT     (3.0f)
static float control_rth_land_check_timer = 0.0f;

bool control_aruco_check()
{
	uint8_t check = CONTROL_CHECK_ATT | CONTROL_CHECK_ALT | CONTROL_CHECK_ARM;

	if(control_check(check) == true){
		nav_get_pos(&pos_curr);

		if(pos_curr.pos_valid == true ){
			return true;
		}else{
			INFO(DEBUG_ID,"aruco rth check pos failed");
			return false;
		}
	}

	return false;
}

void control_aruco_param_init()
{
}

void control_aruco_exit()
{
	INFO(DEBUG_ID,"aruco exit");
}

void control_aruco_xy_yaw(float x,float y)
{
   float yaw;
   yaw = attitude_get_att_yaw();

 //  x = x * cosf(yaw) - y * sinf(yaw);
   
 //  y = y * cosf(yaw) + x * sinf(yaw);

  if(yaw < -45 && yaw > -135){    //left
   pos_sp[0] = pos_sp[0] - x ;
   pos_sp[1] = pos_sp[1] - y;

  }else if(yaw > -45 && yaw < 45){   //up
   pos_sp[0] = pos_sp[0] + y;
   pos_sp[1] = pos_sp[1] - x;

  }else if(yaw < 135 && yaw > 45){   //right
   pos_sp[0] = pos_sp[0] + x;
   pos_sp[1] = pos_sp[1] + y;

  }else {                           //down
   pos_sp[0] = pos_sp[0] - y;
   pos_sp[1] = pos_sp[1] + x;

  }
  
}

void control_aruco_init(float param1,float param2)
{	
	v3f_set(pos_sp,pos_curr.pos_ned);
    debug_t("aruco init (%3.3f,%3.3f,%3.3f)\n",pos_sp[0],pos_sp[1],pos_sp[2]);
    dt_sum_timeout = 0;
//    pos_sp[0] = pos_sp[0] + param2;
//    pos_sp[1] = pos_sp[1] - param1;
	control_aruco_xy_yaw( param1, param2);
	aruco_status = ARUCO_STATUS_GO_HOME;
    debug_t("aruco cur (%3.3f,%3.3f,%3.3f x = %3.3f y = %3.3f)\n",pos_sp[0],pos_sp[1],pos_sp[2],param2,param1);
	INFO(DEBUG_ID,"aruco init (%3.3f,%3.3f,%3.3f)",pos_sp[0],pos_sp[1],pos_sp[2]);
}

bool control_aruco_go_home(float dt,rc_s * rc)
{
	float length_to_sp = pythagorous2((pos_sp[0] - pos_curr.pos_ned[0]),(pos_sp[1] - pos_curr.pos_ned[1]));
	float length_vel = pythagorous2(pos_curr.vel_ned[0],pos_curr.vel_ned[1]);
    float x,y;

	get_pos_aruco(&x,&y);
	dt_sum+=dt;
    if(dt_sum > 0.5f){
	dt_sum = 0;
	v2f_set(pos_sp,pos_curr.pos_ned);
	if(y < 0){
	 y-=0.15;
	}	
	control_aruco_xy_yaw(x, y);    
    }

    dt_sum_timeout+=dt;
	if(dt_sum_timeout > 15){
     dt_sum_timeout = 0;
	 debug_t("timeout aruco center  \n");
	 return true;
	}
	
	if(length_to_sp < RADIUS_ARUCO ){
		if(length_vel < 0.1f){
            debug_t("x = %f y = %f landing:%5.1f,%5.1f,%5.1f\n",x,y,pos_curr.pos_ned[0],pos_curr.pos_ned[1],pos_curr.pos_ned[2]);
			INFO(DEBUG_ID,"landing:%5.1f,%5.1f,%5.1f",pos_curr.pos_ned[0],pos_curr.pos_ned[1],pos_curr.pos_ned[2]);
		//	return true;
		}
	}

    if(fabs(x) < DIS_CENTER && fabs(y) < DIS_CENTER){
       debug_t("x = %f y = %f\n",x,y);
	   return true;
	}
	
	if(fabs(rc->thr_raw) > RTH_RC_ALT_DEADZONE && fabs(pos_curr.pos_ned[2] - pos_sp[2]) < 0.5f){
		control_alt_common_rc(dt,rc);
		pos_sp[2] = pos_control_get_pos_ned_z_target();
	}else{
		control_alt_pos_common(dt,pos_sp[2],2.0f);
	}

	control_pos_ned_xy_common(dt,pos_sp,0,0.5f);
	control_yaw_common_rc(dt,rc);
	return false;
}

bool control_aruco_land_check(float dt)
{
	float acc[3];
	float vel_z;
	imu_get_acc(acc);
	if(acc[2] < RTH_LAND_ACC_CHECK_THRESHOLD){
		return true;
	}

	vel_z = nav_get_vel_ned_z();
	if(fabs(vel_z - 0.0f) < RTH_LAND_VEL_CHECK_THRESHOLD){
		control_rth_land_check_timer += dt;
	}else{
		control_rth_land_check_timer = 0.0f;
	}
	
	if(control_rth_land_check_timer > RTH_LAND_VEL_CHECK_TIMEOUT){
		return true;
	}

	return false;
}

bool control_aruco_landing(float dt,rc_s * rc)
{
	control_pos_ned_xy_common(dt,pos_sp,0,0.5f);
	control_yaw_common_rc(dt,rc);
	control_alt_vel_common(dt,-0.8f,0.8f);
	
	if(control_aruco_land_check(dt) == true){
		return true;
	}
	
	return false;
}

bool control_aruco_landed(float dt,rc_s * rc)
{
	control_set_mode(CONTROL_MODE_STOP,0,0);
	return true;
}

void control_aruco_update(float dt,rc_s * t_rc)
{
	nav_get_pos(&pos_curr);
	if(pos_curr.pos_valid == false){
		debug_t("[ARUCO]position lost\n");
		control_set_mode(CONTROL_MODE_ALTHOLD,0,0);
	}

	switch(aruco_status){
		case ARUCO_STATUS_GO_HOME:
			if(control_aruco_go_home(dt,t_rc) == true){
				//aruco_status = ARUCO_STATUS_LANDING;
				control_set_normal_mode();
			}
			break;
		case ARUCO_STATUS_LANDING:
			if(control_aruco_landing(dt,t_rc) == true){
				aruco_status = ARUCO_STATUS_LANDED;
			}
			break;
		case ARUCO_STATUS_LANDED:
			if(control_aruco_landed(dt,t_rc) == true){
				aruco_status = ARUCO_STATUS_IDLE;
			}
			break;
		case ARUCO_STATUS_IDLE:
			break;
	}
}

