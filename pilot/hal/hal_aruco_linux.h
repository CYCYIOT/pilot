#ifndef _HAL_ARUCO_LINUX_H_
#define _HAL_ARUCO_LINUX_H_

typedef struct {
   long    msg_type;  
   uint16_t  id_size; 
   int  id_data[4][5];
}aruco_msg_t;

bool aruco_linux_open();
bool aruco_linux_read(float dt,float vel[2],float *quality,uint16_t * ver);
int aruco_thread(void *arg);

float get_aruco_thr();
float get_aruco_yaw_p();
float get_aruco_x_p();
float get_aruco_y_p();

void set_power(float roll,float pitch,float yaw,int id_aruco,int action);
void set_power_zero();



#define APP_CAL 1


#endif
