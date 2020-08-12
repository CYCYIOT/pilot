#ifndef _APP_FAILSAFE_H_
#define _APP_FAILSAFE_H_

#include <stdbool.h>

//order by priority , big value mean high priority
#define FS_MODE_RC_TIMEOUT		0
#define FS_MODE_BAT_LOW			1
#define FS_MODE_HIGH_TEMP		2
#define FS_MODE_MOTOR_LOCKED	3
#define FS_MODE_COLLISION		4
#define FS_MODE_ATT_LIMIT		5
#define FS_MODE_MAX				6

void failsafe_init(void);
void failsafe_param_init(void);
void failsafe_update(float dt);

uint32_t failsafe_get_mode();
uint8_t *failsafe_get_motor_val();
uint8_t *failsafe_get_motor_locked_cnt();
uint8_t failsafe_get_collision_val();

void failsafe_set_att_limit_enable(bool enable);
void failsafe_set_collision_enable(bool enable);
void failsafe_set_motor_locked_enable(bool enable);
void failsafe_set_collision_file_enable(bool enable);

#endif
