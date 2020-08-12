#ifndef _HAL_LINUX_H_
#define _HAL_LINUX_H_

#include <stdbool.h>
#include <pthread.h>

void hal_linux_send_signal(int signal);

void hal_linux_init(void);
void hal_linux_mlock(void);

int hal_linux_create_thread(const char *name, int priority, int stack_size, void* entry,void* parameter,int sched_method);
void hal_linux_uart_cfg(int fd, int speed);
void hal_linux_power_up_gps(void);
void hal_linux_power_down_gps(void);

bool hal_linux_cpu_open();
bool hal_linux_cpu_read(float dt,float * cpu_free);

bool hal_linux_mem_open();
bool hal_linux_mem_read(float dt,float * mem_free);

#endif
