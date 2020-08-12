#ifndef _PILOT_STEAM_CONTROL_H
#define _PILOT_STEAM_CONTROL_H

 
enum cmd{
	CMD_SAFE_MODE=83,		 //83
	CMD_WING_PROTECTION,
	CMD_CONTROL_YAW,
	CMD_CONTROL_ALT,
	CMD_IMG_LOAD,
	CMD_TOF_ALTHOLD,       //88
	CMD_TAKEOFF_ALT,
	CMD_INFRARED_SHOOTING,
};


bool get_collision_flag();
bool get_wing_protecttion_flag();

void pilot_steam_control_init();
int pilot_recv_from_img(void *arg);
int pilot_recv_from_udp_update(float arg);
int pilot_send_to_img(int vel);

void steam_control_tof_althold(int alt);

void steam_control_tof_althold_udp(int alt);



#endif
















