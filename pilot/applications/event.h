#ifndef __EVENT_H
#define __EVENT_H

typedef enum{
	ACTION_DEF,
	ACTION_YAW,
	ACTION_CENTER,
}EV;

struct event_data{
	EV action;
	int id;
	float roll;
    float pitch;
    float yaw;
	void *param;
};

void event_init(void);
void event_put(struct event_data *event);
void event_get(struct event_data *event);

#endif
