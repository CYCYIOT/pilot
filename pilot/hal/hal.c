#include "app_debug.h"
#include "app_system.h"
#include "app_batt.h"
#include "app_param.h" 

#include "hal.h"
#include "hal_linux.h"
#include "hal_ist8310.h"
#include "hal_mpu6050.h"
#include "hal_vl53l1x.h"
#include "hal_ks103.h"
#include "hal_fbm320.h"
#include "hal_qmc5883.h"
#include "hal_mmc5883.h"
#include "hal_hmc5883.h"
#include "hal_lis3mdl.h"
#include "hal_bk2461.h"
#include "hal_ubx.h"
#include "hal_stm8s.h"
#include "hal_flow_lc306.h"
#include "hal_flow_pmw3901.h"
#include "hal_flow_linux.h"
#include "hal_dps280.h"
#include "hal_spl06.h"
#include "hal_nmea.h"
#include "hal_batt_linux.h"
#include "hal_tof_vl53l1x.h"
#include "hal_infra_red.h"
#include "hal_aruco_linux.h"
#include "hal_arduino.h"
#define DEBUG_ID DEBUG_ID_HAL

bool spl06_used			= false;
bool qmc5883_used		= false;
bool mmc5883_used		= false;
bool hmc5883_used		= false;
bool fbm320_used		= false;
bool dps280_used		= false;
bool ks103_used			= false;
bool vl53l1x_used		= false;
bool mpu6050_used		= false;
bool lis3mdl_used		= false;
bool bk2461_used		= false;
bool ubx_used			= false;
bool nmea_used		    = false;
bool stm8s_used     	= false;
bool flow_linux_used	= false;
bool flow_lc306_used	= false;
bool flow_pmw3901_used	= false;
bool batt_linux_used	= false;
bool ist8310_used		= false;
bool cpu_linux_used		= false;
bool mem_linux_used		= false;
bool tof_vl53l1x_used   = false;
bool infra_red_used     = false;
bool aruco_used         = false;
bool arduino_used       = false;
int hal_init(void)
{
    int ret = 0;

	hal_linux_init();

	//GPS
	ubx_used = ubx_open();
	//nmea_used = nmea_open();

	//IMU
	mpu6050_used = mpu6050_open();
      
    //tof
    tof_vl53l1x_used=tof_vl53l1x_open();
    //infra_red
   infra_red_used=infra_red_open();
	//MOTOR
	stm8s_used = stm8s_open();	

	//BARO
	fbm320_used = fbm320_open();
	dps280_used = dps280_open();
	spl06_used = spl06_open();

	//RF
	vl53l1x_used = vl53l1x_open();
	ks103_used = ks103_open();

	//MAG
	ist8310_used = ist8310_open();
	hmc5883_used = hmc5883_open();
	qmc5883_used = qmc5883_open();
	mmc5883_used = mmc5883_open();
	lis3mdl_used = lis3mdl_open();

	//FLOW
	flow_linux_used = flow_linux_open();
	//flow_lc306_used = flow_lc306_open();
	//flow_pmw3901_used = flow_pmw3901_open();
    
    // ARUCO
    aruco_used=aruco_linux_open();
	//BATT
	batt_linux_used = batt_linux_open();

#ifdef X_1
    //arduino
    arduino_used = arduino_open();
#endif	
	//CPU & MEMORY
	cpu_linux_used = hal_linux_cpu_open();
	mem_linux_used = hal_linux_mem_open();

	return ret;
}

bool hal_get_rangefinder(float dt,float * dist)
{
#if 0
        if(tof_vl53l1x_used == true){
            return  tx_tof_data(dt,dist);
         }
#endif
#if 1
	if(vl53l1x_used == true){
		return vl53l1x_read(dt,dist);
	}	

	if(ks103_used == true){
		return ks103_read(dt,dist);
	}	
#endif
	return false;
}

bool hal_get_baro(float dt,float * pres,float * alt,float * temp)
{
#if 0
        if(tof_vl53l1x_used == true){
            return  tx_tof_data(dt,alt);
         }
#endif
	if(dps280_used == true){
		return dps280_read(dt,pres,alt,temp);
	}
	
	if(fbm320_used == true){
		return fbm320_read(dt,pres,alt,temp);
	}
	
	if(spl06_used == true){
		return spl06_read(dt,pres,alt,temp);
	}

	return false;
}

bool hal_get_imu(float dt,float acc[3],float gyro[3],float * temp)
{
	if(mpu6050_used == true){
		return mpu6050_read(dt,acc,gyro,temp);
	}
	
	return false;
}

bool hal_get_tof()
{
     if(tof_vl53l1x_used == true)
             printf("tof ok\n");
    
 return true;
}
bool hal_get_mag(float dt,float mag[3])
{
	if(ist8310_used == true){
		return ist8310_read(dt,mag);
	}

	if(hmc5883_used == true){
		return hmc5883_read(dt,mag);
	}

	if(lis3mdl_used == true){
		return lis3mdl_read(dt,mag);
	}

	if(qmc5883_used == true){
		return qmc5883_read(dt,mag);
	}
	
	if(mmc5883_used == true){
		return mmc5883_read(dt,mag);
	}

	return false;
}

bool hal_get_cpu_free(float dt,float * cpu_free)
{
	if(cpu_linux_used == true){
		return hal_linux_cpu_read(dt,cpu_free);
	}

	return false;
}

bool hal_get_mem_free(float dt,float * mem_free)
{
	if(mem_linux_used == true){
		return hal_linux_mem_read(dt,mem_free);
	}

	return false;
}

bool hal_set_motor(float *motor,uint8_t num)
{
	if(stm8s_used == true){
		return stm8s_write(motor,num);
	}
	
	return false;
}

bool hal_get_motor(uint8_t *val ,int len)
{
	if(stm8s_used == true){
		return stm8s_read(val,len);
	}

	return false;
}

bool hal_get_rc(float dt,float rc[5])
{
	if(bk2461_used == true){
		return bk2461_read(rc);
	}
	
	return false;
}

bool hal_get_gps(float dt,gps_info_s *dat)
{
	if(ubx_used == true){
		return ubx_read(dt,dat);
	}

	if(nmea_used == true){
		return nmea_read(dt,dat);
	}

	return false;
}

bool hal_get_flow(float dt,float vel[2],float *quality,uint16_t * ver)
{
	if(flow_linux_used == true){
		return flow_linux_read(dt,vel,quality,ver);
	}

	return false;
}

bool hal_get_aruco(float dt,float vel[2],float *quality,uint16_t * ver)
{
	if(aruco_used == true){
		return aruco_linux_read(dt,vel,quality,ver);
	}

	return false;
}

bool hal_get_flow2(float dt,float vel[2],float *quality,uint16_t * ver)
{
	if(flow_lc306_used == true){
		return flow_lc306_read(dt,vel,quality,ver);
	}

	return false;
}

bool hal_get_flow3(float dt,float vel[2],float *quality,uint16_t * ver)
{
	if(flow_pmw3901_used == true){
		return flow_pmw3901_read(dt,vel,quality,ver);
	}

	return false;
}

bool hal_get_batt(float dt,int *batt_status)
{
	if(batt_linux_used == true){
		return batt_linux_read(dt,batt_status);
	}
	
	return false;
}

void hal_set_led_status(uint8_t status)
{
	if(stm8s_used == true){
		stm8s_set_led(status);
	}
}


