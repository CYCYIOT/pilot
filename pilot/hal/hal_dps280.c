#include <fcntl.h>
#include <unistd.h>
#include "app_debug.h"
#include "hal.h"
#include "lib_math.h"
#include "param.h"
#include "awlink_item_control.h"
#include "app_control.h"

#include "hal_tof_vl53l1x.h"
#define DEBUG_ID DEBUG_ID_HAL

#define DPS280_PATH "/dev/dps280"

#define POW_2_23_MINUS_1	0x7FFFFF   //implies 2^23-1
#define POW_2_24			0x1000000
#define POW_2_15_MINUS_1	0x7FFF
#define POW_2_16			0x10000
#define POW_2_11_MINUS_1	0x7FF
#define POW_2_12			0x1000
#define POW_2_20			0x100000
#define POW_2_19_MINUS_1	524287


/* Struct to hold calibration coefficients read from device*/
typedef struct 
{
  /* calibration registers */

  int16_t 	C0;	// 12bit
  int16_t 	C1;	// 12bit
  int32_t	C00;	// 20bit
  int32_t   C10;	// 20bit
  int16_t 	C01;	// 16bit
  int16_t	C11;	// 16bit
  int16_t	C20;	// 16bit
  int16_t	C21;	// 16bit
  int16_t	C30;	// 16bit

}dps280_cal_coeff_regs_s;


typedef struct {
	uint8_t  pressure[3];
	uint8_t  temperature[3];
	uint32_t tmp_osr_scale_coeff;
	uint32_t prs_osr_scale_coeff;
	dps280_cal_coeff_regs_s calib_coeffs;
}dps280_report_s;

#define DPS280_UPDATE_PRESS	0.040 //second
#define DPS280_UPDATE_TEMP	0.005 //second

static int dps280_fd = -1;
float tof_vel;
float  dps_vel;
int t_flag=0;
int takeoff_flag=0;
int t_count=0;
int c_flag=0;
float tof_old2=0;
float tof_old1=0;
float  tof2_vel;
float alt_old =0;
void dps280_convert(dps280_report_s *rp,float * pressure,float *temp)
{
   
   double	press_raw;
   double	temp_raw;

   double 	temp_scaled;
   double 	temp_final;
   double 	press_scaled;
   double 	press_final;
   

	press_raw = (rp->pressure[2]) + (rp->pressure[1]<<8) + (rp->pressure[0] <<16);
    temp_raw  = (rp->temperature[2]) + (rp->temperature[1]<<8) + (rp->temperature[0] <<16);

	if(temp_raw > POW_2_23_MINUS_1){
		temp_raw = temp_raw - POW_2_24;
	}

	if(press_raw > POW_2_23_MINUS_1){
		press_raw = press_raw - POW_2_24;
	}

	temp_scaled = (double)temp_raw / (double) (rp->tmp_osr_scale_coeff);

	temp_final =  (rp->calib_coeffs.C0 /2.0f) + rp->calib_coeffs.C1 * temp_scaled ;
    
	press_scaled = (double) press_raw / rp->prs_osr_scale_coeff;

	press_final = rp->calib_coeffs.C00 +
                      press_scaled *  (  rp->calib_coeffs.C10 + press_scaled *
                      ( rp->calib_coeffs.C20 + press_scaled * rp->calib_coeffs.C30 )  ) +
                      temp_scaled * rp->calib_coeffs.C01 +
                      temp_scaled * press_scaled * ( rp->calib_coeffs.C11 +
                                                      press_scaled * rp->calib_coeffs.C21 );


	press_final = press_final * 0.01f;	//to convert it into mBar

	*temp = temp_final;
    *pressure    = press_final;  //press_final;

}

float dps280_altitude_convert(float Press, float Ref_P)
{
	return 44330 * (1 - powf(((float)Press / (float)Ref_P),(1/5.255)));
}

bool dps280_open()
{
	dps280_fd = open(DPS280_PATH,O_RDONLY);
	if(dps280_fd < 0){
		DEBUG(DEBUG_ID,"open dps280 failed (%s)", DPS280_PATH);
		return false;
	}else{
		INFO(DEBUG_ID,"use dps280 baro");
	}

	return true;
}
float pressure_ref = 0.0f;
float dps280_timer = 0.0f;
float dps280_init_timer = 0.0f;

void set_takeoff_flag(int vel)
{
takeoff_flag=vel;

}

bool dps280_read(float dt,float * pres,float * alt,float * temp)
{
#if 1
	dps280_report_s report_dps280; 
	float pressure_tmp;
	float temp_tmp;
	float alt_tmp;
    float tof_alt=0;
	int ret = 0;
#endif
       
	dps280_timer += dt;
	dps280_init_timer += dt;

	if(dps280_timer > DPS280_UPDATE_PRESS){
		dps280_timer = 0.0f;
                
#if 1
		ret = read(dps280_fd,&report_dps280,sizeof(report_dps280));
		if(ret > 0){
				dps280_convert(&report_dps280,&pressure_tmp,&temp_tmp);
				if(dps280_init_timer > 1.0f && pressure_ref == 0.0f){
					//printf("baro init :%f\n",pressure_tmp);
					pressure_ref = pressure_tmp;
				}

				if(pressure_ref != 0){
					alt_tmp = dps280_altitude_convert(pressure_tmp,pressure_ref);
              if(hal_get_tof_fd() == true){
			 
			        tof_old1=tof_old2;
					tx_tof_data(&dt,&tof_alt);
					tof_old2=tof_alt;
                    //get_tof_flag()== 1||
   			     if((get_file_flag()== true )|| fabs(tof_old1 - tof_alt) > 0.1 ){  //ÇÐ»»ÆøÑ¹¼Æ
                   if(control_get_mode() != CONTROL_MODE_TAKEOFF && control_get_mode() != CONTROL_MODE_STOP){
					   takeoff_flag=1;
                     // debug_t("tof_old = %f tof_now = %f \n",tof_old1,tof_alt);
                       c_flag=0;
                       t_count=0; 
                       if(t_flag == 0){
                          tof_vel=alt_old - alt_tmp;
						  t_flag=1;
	                      }
                   	}else{
                       takeoff_flag = 0;
					}
                   }
				 else{
				    if(takeoff_flag == 1){
                       if(t_count < 510)
					   t_count++;
					   t_flag=0; 
				       	}
					 }
			     if(takeoff_flag == 0){
					 *alt = tof_alt;
					}
				 else{
  					if(t_count > 500){    // ÇÐ»»»ØTOF
                      
					  if(c_flag == 0){
					  	 
                         dps_vel = alt_old - tof_alt;
                         c_flag=1;
						 
					      }
						*alt = tof_alt + dps_vel;

                       // debug_t("alt = %f tof_alt = %f dps_vel = %f \n",);
						
						 }
					else{
					   *alt = alt_tmp + tof_vel;
					      }
					  	}
				 alt_old=*alt;
                    }
				else{
					  *alt = alt_tmp;
					}	
					*pres = pressure_tmp;
					*temp = temp_tmp;
                 
					return true;
				}
				return false;
		}else{
			return false;
		}
	}
	return false;
#endif 
#if 0
           tx_tof_data(&dt,alt);
            return true;
           }
     return false;
#endif
}

