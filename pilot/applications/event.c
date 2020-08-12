#include <pthread.h>
#include "event.h"

#define EVENT_NUM 5

struct event_buffer{
	struct event_data state_buf[EVENT_NUM];
	int w_idx;
	int r_idx;
	pthread_mutex_t mutex;
	pthread_cond_t update;
};

struct event_buffer g_event;

void event_put(struct event_data *event)
{
	pthread_mutex_lock(&g_event.mutex);
	g_event.state_buf[g_event.w_idx] = *event;
	g_event.w_idx = (g_event.w_idx+1) % EVENT_NUM;
	pthread_cond_signal(&g_event.update);
	pthread_mutex_unlock(&g_event.mutex);
}

void event_get(struct event_data *event)
{
	if(g_event.r_idx == g_event.w_idx)
	{
		pthread_mutex_lock(&g_event.mutex);
		pthread_cond_wait(&g_event.update, &g_event.mutex);
		pthread_mutex_unlock(&g_event.mutex);
	}
	*event = g_event.state_buf[g_event.r_idx];
	g_event.r_idx = (g_event.r_idx+1) % EVENT_NUM;
}

void event_init(void)
{
	g_event.w_idx = 0;
	g_event.r_idx = 0;
	pthread_mutex_init(&g_event.mutex,NULL);
	pthread_cond_init(&g_event.update,NULL);
}
