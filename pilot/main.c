#include <linux/input.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>
#include <unistd.h>
#include <termios.h>

#include "hal.h"
#include "event.h"
#include "app_attitude.h"
#include "app_att_control.h"
#include "app_pos_control.h"
#include "app_control.h"
#include "app_system.h" 
#include "app_log.h" 
#include "app_param.h" 
#include "app_param_calib.h" 
#include "app_motor.h"
#include "app_rc.h"
#include "app_imu.h"
#include "app_baro.h"
#include "app_flow.h"
#include "app_gps.h"
#include "app_rangefinder.h"
#include "app_mag.h"
#include "app_debug.h"
#include "app_nav.h"
#include "app_batt.h"
#include "app_awlink.h"
#include "app_failsafe.h"
#include "app_notify.h"
#include "hal_tof_vl53l1x.h"
#include "pilot_steam_control.h"
#include "app_aruco.h"
#include "hal_aruco_linux.h"
#include "hal_arduino.h"
#define MAIN_LOOP_TIME	(1.0f/MAIN_LOOP_HZ)

#define DEBUG_ID DEBUG_ID_MAIN

void module_param_init()
{
	param_init();
	param_calib_init();
	
	log_param_init();
	nav_param_init();
	imu_param_init();
	attitude_param_init();
	mag_param_init();
	control_param_init();
	att_control_param_init();
	pos_control_param_init();
	motor_param_init();
	rc_param_init();
	failsafe_param_init();
	system_param_init();
	flow_param_init();
	rangefinder_param_init();
	
	param_load();
	param_calib_load();
}

void module_init()
{
	awlink_init();
	log_init();
	hal_init();
	imu_init();
	mag_init();
	baro_init();
	rangefinder_init();
	gps_init();
	motor_init();
	att_control_init();
	pos_control_init();
	rc_init();
	flow_init();
	batt_init();
	system_init();
	control_init();
	attitude_init();
	nav_init();
	failsafe_init();
	notify_init();
	pilot_steam_control_init();
//	event_init();
}

void module_exit()
{
	awlink_exit();
}


int pilot_thread_main(void* paramter)
{
	struct timespec prev;
	float dt,run_time_dt;

	get_diff_time(&prev,true);
	usleep(2000);
	
	while(system_get_run() == true){
		dt = get_diff_time(&prev,true);
		if(dt < 1.0f){
			//sensor update
			imu_update(dt);
			mag_update(dt);
			baro_update(dt);
			rangefinder_update(dt);
			gps_update(dt);
			flow_update(dt);
            //aruco_update(dt);
			//position update
			attitude_update(dt);
			nav_update(dt);
            pilot_recv_from_udp_update(dt);
#ifdef X_1
	        arduino_update(dt);
#endif
			//rc and awlink update
			rc_update(dt);
			awlink_update(dt);
 
			//control mode and motor update
			control_update(dt);
			motor_update(dt);

			//system update			
			batt_update(dt);
			system_update(dt);
			failsafe_update(dt);    //安全检查
			log_update(dt);
			notify_update(dt);     //通知
		}
#if 1
		run_time_dt = get_diff_time(&prev,false);
               // ERR(DEBUG_ID,"run_time_dt:%3.3fms",run_time_dt*1000);
		if(run_time_dt > MAIN_LOOP_TIME || dt > (0.001 + MAIN_LOOP_TIME)){
			ERR(DEBUG_ID,"run:%3.3fms,dt:%3.3fms",run_time_dt*1000,dt*1000);
			run_time_dt = MAIN_LOOP_TIME - 0.0005f;
			get_diff_time(&prev,true);
		}
		

		usleep((MAIN_LOOP_TIME - run_time_dt) * 1000 * 1000);
#endif
	}

	module_exit();

	return 0;
}

int main(int argc, char *argv[])
{
	hal_linux_mlock();

	debug_init();

	INFO(DEBUG_ID,"=====AWpilot(%dHz)=====",MAIN_LOOP_HZ);
	INFO(DEBUG_ID,"=====Version(%d)=====",VER);
    INFO(DEBUG_ID,"=====Model(%c)====",MODEL_UAV);

	if(argc >= 2 && strncmp(argv[1],"debug",5) == 0){
		if(argc == 2){
			debug_module_show();
			return 0;
		}else if(argc == 3){
			debug_module_set(argv[2]);
		}
	}

	module_param_init();
	module_init();

    hal_linux_create_thread("pilot-aruco",30,200 * 1024,aruco_thread,NULL,SCHED_DEFAULT);
	hal_linux_create_thread("pilot-tof",20,200 * 1024,pilot_recv_from_img,NULL,SCHED_DEFAULT);	
	hal_linux_create_thread("pilot-main",99,500 * 1024,pilot_thread_main,NULL,SCHED_DEFAULT);	
	hal_linux_create_thread("pilot-log",20,200 * 1024,log_thread_main,NULL,SCHED_DEFAULT);

	while(system_get_run() == true){
		sleep(1);
	}

	return 0;
}
