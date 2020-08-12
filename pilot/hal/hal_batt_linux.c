#include <fcntl.h>
#include <unistd.h>

#include "app_debug.h"

#define DEBUG_ID DEBUG_ID_HAL

#define BATT_LINUX_UPDATE_RATE	0.20f//second

#define BAT_PAHT	"/dev/lradc_battery"
#define	BAT_HIGHT_RESISITOR     150
#define	BAT_LOW_RESISITOR       100

int batt_fd = 0;

enum battery_charge_state{
	NO_INSERT,
	CHARGING,
	CHARGED,
};

struct battery_info {
	unsigned int voltage;
	enum battery_charge_state state;
};


/*
static unsigned int calc_voltage(unsigned int adc_val)
{
    return (adc_val)*2200/63*(BAT_HIGHT_RESISITOR+BAT_LOW_RESISITOR)/BAT_LOW_RESISITOR;
}
*/
bool batt_linux_open()
{
	batt_fd = open(BAT_PAHT,O_RDONLY);
	if(batt_fd < 0){
		DEBUG(DEBUG_ID,"open batt linux failed (%s)",BAT_PAHT);
		return false;
	}else{
		INFO(DEBUG_ID,"use batt linux");
	}

	return true;
}

bool batt_linux_read(float dt,int *batt_status)
{
	static float batt_linux_timer = BATT_LINUX_UPDATE_RATE;
	struct battery_info buf;

	batt_linux_timer += dt;
	if(batt_linux_timer > BATT_LINUX_UPDATE_RATE){
		batt_linux_timer = 0.0f;
		read(batt_fd,&buf,sizeof(buf));
		*batt_status = buf.state;
		DEBUG(DEBUG_ID,"bat status :%d ",*batt_status);		
		return true;
	}else{
		return false;
	}
}

