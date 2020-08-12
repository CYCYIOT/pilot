#ifndef _APP_CONTROL_H_
#define _APP_CONTROL_H_

#include <stdbool.h>
#include <stdint.h>

#define CONTROL_CHECK_ATT		(1 << 0)
#define CONTROL_CHECK_ALT		(1 << 1)
#define CONTROL_CHECK_VEL		(1 << 2)
#define CONTROL_CHECK_REL_POS	(1 << 3)
#define CONTROL_CHECK_ABS_POS	(1 << 4)
#define CONTROL_CHECK_ARM		(1 << 5)
#define CONTROL_CHECK_DISARM	(1 << 6)

#define CONTROL_MODE_STABILIZE	0
#define CONTROL_MODE_ALTHOLD	1
#define CONTROL_MODE_POSHOLD	2
#define CONTROL_MODE_TAKEOFF	3
#define CONTROL_MODE_LAND		4
#define CONTROL_MODE_FOLLOWME	5
#define CONTROL_MODE_MISSION	6
#define CONTROL_MODE_CIRCLE		7
#define CONTROL_MODE_FLIP		8
#define CONTROL_MODE_RTH		9
#define CONTROL_MODE_STOP		10
#define CONTROL_MODE_LOCAL360	11
#define CONTROL_MODE_THROWN     12
#define CONTROL_MODE_MAX		13

void control_param_init();
void control_init();
void control_update(float dt);

bool control_check(uint8_t check);

uint8_t control_get_mode();
uint8_t control_get_mode_last();

bool control_set_mode(uint8_t new_mode,float param1,float param2);
bool control_set_mode_last();
void control_set_normal_mode();

#endif

