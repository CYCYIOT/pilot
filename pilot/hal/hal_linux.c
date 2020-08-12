#include <pthread.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include <termios.h>
#include <signal.h>
#include <sys/mman.h>
#include <sys/types.h>
#include <sys/sysinfo.h>

#include "app_debug.h"
#include "param.h"

#define DEBUG_ID DEBUG_ID_HAL

#define MAX_SAFE_STACK 128 * 1024

#define CPU_LINUX_HZ 1
#define MEM_LINUX_HZ 1

typedef struct cpu_info
{
	unsigned long long usr, nic, sys, idle;
	unsigned long long iowait, irq, softirq, steal;
	unsigned long long total;
}cpu_t;

FILE * stat_fd;
cpu_t cpu_last;

int hal_linux_read_pid(const char *filename)
{
    FILE *fp = fopen(filename, "r");
    if(!fp) {
        printf("%s(): fopen('%s') failed: %s\n", __func__, filename, strerror(errno));
        return -1;
    }
    
    int pid = -1;
    int len = fscanf(fp, "%d", &pid);
    if(len < 0) {
        printf("%s(): fscanf('%s') failed: %s\n", __func__, filename, strerror(errno));
        fclose(fp);
        return -1;
    }
    
    fclose(fp);
    return pid;
}

void hal_linux_send_signal(int signal)
{
	int sig = 0;
	int pid = hal_linux_read_pid("/tmp/flow.pid");

	if(signal == 1){
		sig = SIGUSR1;
		printf("%s(): Send SIGUSR1 to flow\n", __func__);
	}else{
		sig = SIGUSR2;
		printf("%s(): Send SIGUSR2 to flow\n", __func__);
	}
	
	kill(pid, sig);
}

void hal_linux_mlock(void)
{
	unsigned char dummy[MAX_SAFE_STACK];
	
	memset(dummy, 0, MAX_SAFE_STACK);
	
	if(mlockall(MCL_CURRENT|MCL_FUTURE) == -1){
		perror("mlockall failed");
		exit(-2);
	}
}

float cal_cpu(cpu_t *last, cpu_t *now)
{
    float cpu_use;
	unsigned total_diff;
	
	total_diff = (unsigned)(now->total - last->total);
	cpu_use = 100 * (float)(now->idle - last->idle) / (float)total_diff;

    return cpu_use;
}

bool get_cpu(cpu_t * cpu)
{
	int ret;
    char buff[256];

    stat_fd = fopen("/proc/stat" , "r");
    if(stat_fd == NULL){
		perror("fopen:");
		return false;
    }

    fgets(buff, sizeof(buff), stat_fd);
    ret = sscanf(buff,"cpu %llu %llu %llu %llu %llu %llu %llu %llu",&cpu->usr,&cpu->nic,&cpu->sys,&cpu->idle,&cpu->iowait,&cpu->irq,&cpu->softirq,&cpu->steal);
	if(ret >= 4){
		cpu->total = cpu->usr + cpu->nic + cpu->sys + cpu->idle + cpu->iowait + cpu->irq + cpu->softirq + cpu->steal;
	}

	fclose(stat_fd);
	return true;
}

bool hal_linux_cpu_open()
{
	return get_cpu(&cpu_last);
}

bool hal_linux_cpu_read(float dt,float * cpu_free)
{
	static float dt_sum = 0;
    cpu_t cpu_now;
    float val = 0;

	dt_sum += dt;
	if(dt_sum >= (1.0f / CPU_LINUX_HZ)){
		dt_sum = 0.0f;

		if(get_cpu(&cpu_now) == false){
			return false;
		}
		
		val = cal_cpu(&cpu_last,&cpu_now);
		cpu_last = cpu_now;
		
		*cpu_free = val;
		
		return true;
	}else{
		return false;
	}
}

bool hal_linux_mem_open()
{
	return true;
}

bool hal_linux_mem_read(float dt,float * mem_free)
{
	static float dt_sum = 0;
	struct sysinfo si;

	dt_sum += dt;
	if(dt_sum >= (1.0f / MEM_LINUX_HZ)){
		dt_sum = 0.0f;

		if(sysinfo(&si) < 0){
			return false;
		}
		
		*mem_free = si.freeram / 1024;
		
		return true;
	}else{
		return false;
	}
}

void hal_linux_power_up_gps(void)
{
	int gps_power_fd = -1;
	char cmd = '1';
	gps_power_fd = open("/sys/gps_power/gps_power",O_RDWR);
	if(gps_power_fd > 0){
		write(gps_power_fd,&cmd,1);
	}else{
		INFO(DEBUG_ID,"can not power up gps");
	}
	close(gps_power_fd);
	sleep(1);
	return;
}

void hal_linux_power_down_gps(void)
{
	int gps_power_fd = -1;
	char cmd = '0';
	gps_power_fd = open("/sys/gps_power/gps_power",O_RDWR);
	if(gps_power_fd > 0){
		write(gps_power_fd,&cmd,1);
	}else{
		INFO(DEBUG_ID,"can not power up gps");
	}
	close(gps_power_fd);
	return;
}


void hal_linux_uart_cfg(int fd, int speed)
{  
    struct termios options;  

    tcgetattr(fd, &options);  

	
    options.c_cflag |= (CLOCAL | CREAD);
    options.c_cflag &= ~CSIZE;
	options.c_cflag &= ~PARENB;             //add
    options.c_cflag &= ~CRTSCTS;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag |= CS8;

    options.c_lflag = 0;  
    options.c_lflag &= ~ISIG;
    options.c_lflag &= ~ECHO;

    options.c_oflag = 0;
	options.c_oflag &= ~OPOST;

    options.c_iflag |= IGNPAR;
    options.c_iflag &= ~IXON;
    options.c_iflag &= ~ICRNL;
	
    cfsetspeed(&options, speed);             //add

    tcflush(fd, TCIFLUSH);
    tcsetattr(fd, TCSANOW, &options);
	tcflush(fd,TCIOFLUSH);
   
}

int hal_linux_create_thread(const char *name, int priority, int stack_size, void* entry,void* parameter,int sched_method)
{
    int rv;
    pthread_t task;
	pthread_attr_t attr;
	struct sched_param param;
	
	rv = pthread_attr_init(&attr);
    if (rv != 0) {
		INFO(DEBUG_ID,"platform_create_thread: failed to init thread attrs");
		return rv;
	}
	
	rv = pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);
	if (rv != 0) {
		INFO(DEBUG_ID,"platform_create_thread: failed to set inherit sched");
		return rv;
	}

	rv = pthread_attr_setschedpolicy(&attr, sched_method);
	if (rv != 0) {
		INFO(DEBUG_ID,"platform_create_thread: failed to set sched policy");
		return rv;
	}
	
	rv = pthread_attr_setstacksize(&attr, stack_size);
	if (rv != 0) {
		printf("platform_create_thread: failed to set stack size");
		return rv;
	}

	if(sched_method != SCHED_OTHER){
		param.sched_priority = priority;
		rv = pthread_attr_setschedparam(&attr, &param);
		if (rv != 0) {
			INFO(DEBUG_ID,"platform_create_thread: failed to set sched param");
			return rv;
		}
	}
	
	rv = pthread_create(&task, &attr, entry,parameter);
	if (rv != 0) {
		if (rv == EPERM) {
			INFO(DEBUG_ID,"warning: unable to start");
			rv = pthread_create(&task, NULL, entry, parameter);

			if (rv != 0) {
				INFO(DEBUG_ID,"platform_create_thread: failed to create thread");
				return (rv < 0) ? rv : -rv;
			}
		}else{
			return rv;
		}
	}
	
	return 0;
}

void terminate1(int sig_no)
{
	int fd;
	char msg;

	fd = open("/mnt/err.txt",O_CREAT | O_RDWR);

	if(fd > -1){
		msg = 0x30 + sig_no;
		write(fd,&msg,1);
		close(fd);
	}
	
	ERR(DEBUG_ID,"get signal %d,stop",sig_no);

	exit(0);
}

void hal_linux_init(void)
{
	struct sigaction sa;
	sa.sa_handler = SIG_IGN;
	sigemptyset(&sa.sa_mask);
	sa.sa_flags = 0;
	if (sigaction(SIGPIPE, &sa, NULL) < 0)
	{
		perror("cannot ignore SIGPIPE");
	}

	signal(SIGINT,terminate1);
	signal(SIGABRT,terminate1);
	signal(SIGSEGV,terminate1);
	signal(SIGSTOP,terminate1);
	signal(SIGXCPU,terminate1);

#if 0
	signal(SIGHUP,terminate1);
	signal(SIGQUIT,terminate1);
	signal(SIGILL,terminate1);
	signal(SIGIOT,terminate1);
	signal(SIGBUS,terminate1);
	signal(SIGBUS,terminate1);
	signal(SIGFPE,terminate1);
	signal(SIGKILL,terminate1);
	signal(SIGTERM,terminate1);
	signal(SIGXFSZ,terminate1);
	signal(SIGXFSZ,terminate1);
#endif

}

