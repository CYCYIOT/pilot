#include <string.h>
#include "hal.h"
#include "param.h"

#include "app_batt.h"
#include "app_debug.h"
#include "app_system.h"

#define NOTIFY_HZ   5
#define LOW_CAP     20
uint8_t led_status = 4;
uint8_t tmp_led_status = 0;
uint8_t start_cnt = 0;
float charging_init_timeout = 1.0f;
static float charging_init_time;

void notify_init(void)
{
}

void notify_set_led_status(uint8_t status)
{
	tmp_led_status = status;
}

void notify_led_status_select(float dt)
{
	if(batt_get_charge() == BATT_CHARGING_STATUS_ON){
		charging_init_time += dt;
		if(charging_init_time > charging_init_timeout){
			hal_set_led_status(LED_STATUS_CHARGING);
		}else{
			hal_set_led_status(LED_STATUS_STANDBY);
		}		
	}else{
		charging_init_time = 0;
		if(batt_get_cap() < LOW_CAP){
			hal_set_led_status(LED_STATUS_LOWBATT);
		}else if(system_get_awlink_online() == true){
			hal_set_led_status(LED_STATUS_FLYING);
		}else if(start_cnt > 30){
			hal_set_led_status(LED_STATUS_STANDBY);
			start_cnt = 30;
		}else{
			hal_set_led_status(LED_STATUS_STARTING);
		}
	}
}

void notify_update(float dt)
{
	static float notify_dt = 0.0f;
	notify_dt += dt;	
	if(notify_dt > (1.0f/NOTIFY_HZ)){
		start_cnt++;
		notify_led_status_select(notify_dt);		
		notify_dt = 0.0f;
	}
}

