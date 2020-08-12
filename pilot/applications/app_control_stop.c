#include "app_rc.h"
#include "app_debug.h"
#include "app_system.h"
#include "hal_dps280.h"
#include "app_batt.h"
#include "hal.h"

#define DEBUG_ID DEBUG_ID_CONTROL

bool control_stop_check()
{
	return true;
}

void control_stop_param_init()
{
}

void control_stop_exit()
{
	INFO(DEBUG_ID,"stop exit");
}

void control_stop_init(float param1,float param2)
{
	system_set_armed(false);
//	debug_t("vel2 = %d \n",batt_get_origin_vol());
	INFO(DEBUG_ID,"stop init");
}

void control_stop_update(float dt,rc_s * rc)
{
	

}

