#ifndef _APP_BATT_H_
#define _APP_BATT_H_

#define BATT_CHARGING_STATUS_OFF	0
#define BATT_CHARGING_STATUS_ON		1


void batt_init(void);
void batt_update(float dt);

int batt_get_origin_vol();
uint8_t batt_get_cap();
uint8_t batt_get_vol();
uint8_t batt_get_charge();
int batt_get_raw_adc_val();
int batt_get_raw_bat_val();
float batt_get_cap_tmp();
#endif
