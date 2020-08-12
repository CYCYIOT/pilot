#include <string.h>
#include "param.h"

#include "app_debug.h"
#include "app_system.h"

#include "hal.h"
#include "hal_stm8s.h"
#include "pilot_steam_control.h"
#define DEBUG_ID DEBUG_ID_BATT
#define BAT_VAL_NUM        5
#define BAT_FULL_S         4200
#define BAT_LOW_S          3000

#define BAT_FULL_D         4000

#ifdef EVAB2
#define BAT_LOW_D           2850
#define BAT_ADC_SCALE		2.5f
#else 
#define BAT_LOW_D          3000// 3150   // 最低电压
#define BAT_ADC_SCALE		2.65f   // 2.0
#endif	
//#define BAT_HIGH_D          3000    //默认有护翼，最低电压值

#ifdef M                                  //减速组无人机(睿抗)
#define ARMED_RATE1     0.19              // 55s %10
#define ARMED_RATE2     0.19 
#define ARMED_RATE3     0.19  
#define ARMED_RATE4     0.19  
#define BAT_HIGH_D      3000
#define VOL_VIR         3500
#elif X                                   // c语言无人机
#define ARMED_RATE1     0.17 
#define ARMED_RATE2     0.17 
#define ARMED_RATE3     0.17 
#define ARMED_RATE4     0.17
#define BAT_HIGH_D      3000
#define VOL_VIR         3500
#else                                    //思创无人机
#define ARMED_RATE1     0.3             //(30s %10)
#define ARMED_RATE2     0.3   
#define ARMED_RATE3     0.3
#define ARMED_RATE4     0.3
#define BAT_HIGH_D      3000          //默认有护翼，最低电压阈值
#define VOL_VIR         3200

#endif

#define DISARMED_RATE      0.01
#define HIGH_CAP           80	   
#define MEDIUM_CAP         50         
#define LOW_CAP_1          20
#define LOW_CAP            10
#define FULL_CAP           100        


#define ZERO_CAP           1
#define LOW_CAP_COUNT      5
#define CHARGE_RATE        0.02   
#define VOL_COUNT          10

#define BATT_VOL_OFFSET    150     // 100
#define VOL_SCALE          1.02 

//static float charge_cap = 0;
static float tmp_cap_d = 100.0f;
static float tmp_cap_s = 0.0f;

static int raw_adc_val;
static int raw_bat_val;
static uint8_t batt_cap = 0;
static uint8_t batt_vol = 0;
static uint8_t batt_charging = 0;
static uint8_t batt_full = 0;
static int batt_origin_vol = 0;
static uint8_t low_cap_flag = 0;
static uint8_t low_cap_cnt = 0;
static int tmp_vol = 0;
static uint8_t set_tmp_cap_d_flag = 0;
static uint8_t set_init_cap_flag = 0;
static uint8_t batt_fly_status = 0;
static float  batt_wait_time = 5000.0f;
static float batt_wait_dt = 0.0f;
static int batt_flag=0;
static int cap_rate=1;
static int cap_rate_count=0;
static int cap_rate_flag=0;
int batt_get_origin_vol()
{
	return batt_origin_vol;
}

uint8_t batt_get_cap()
{
	return batt_cap;
}

uint8_t batt_get_vol()
{
	return batt_vol;
}

uint8_t batt_get_charge()
{
	return batt_charging;
}
int batt_get_raw_adc_val()
{
	return raw_adc_val;
}
int batt_get_raw_bat_val()
{
	return raw_bat_val;
}
float batt_get_cap_tmp()
{
       return tmp_cap_d;
}

void batt_calc(float dt,int vol)
{	
	batt_origin_vol = vol;
	batt_vol = vol/100.0f;
    int cap_t=0;
	int cap_max=1;
	int cap_min=0;
	int cap_vel=0;
	
	if(system_get_armed() == true){

		batt_fly_status = 1;
		batt_flag=1;
		if(set_tmp_cap_d_flag == 0){            //第一次赋值
			set_tmp_cap_d_flag = 1;
			tmp_cap_d = batt_cap;
		}	
		
        if(tmp_cap_d > 50 && vol < VOL_VIR  && cap_rate_flag == 0){        // 开机第一次检测虚电，加快掉电速度
           if(cap_rate_count++ > 3){
			cap_rate=3*cap_rate;
			cap_rate_flag=1;
			debug_t("tmp_cap_d=%f vol=%d\n",tmp_cap_d,vol);
           	}
		}

		if(tmp_cap_d > HIGH_CAP){
			tmp_cap_d -= dt * ARMED_RATE1 * cap_rate;
		}else if(tmp_cap_d > MEDIUM_CAP){
			tmp_cap_d -= dt * ARMED_RATE2 * cap_rate;
		}else if(tmp_cap_d > LOW_CAP_1){
			tmp_cap_d -= dt * ARMED_RATE3 * cap_rate;
		}else if(tmp_cap_d > LOW_CAP){
			tmp_cap_d -= dt * ARMED_RATE4 * cap_rate;
		}else{
			tmp_cap_d -= dt * ARMED_RATE2 * 0.5f;
		}
	
	   if(get_wing_protecttion_flag()== false)          //判断电池数据下限
	   	{
	   	 
		if(vol < BAT_LOW_D && low_cap_flag == 0){
			low_cap_cnt++;
			if(low_cap_cnt > LOW_CAP_COUNT){
				low_cap_cnt = 0;
				low_cap_flag = 1;
				tmp_cap_d = LOW_CAP - 1;				
			}
		 }
	   	}
	   else
	   {
        if(vol < BAT_HIGH_D && low_cap_flag == 0){
			low_cap_cnt++;
			if(low_cap_cnt > LOW_CAP_COUNT){
				low_cap_cnt = 0;
				low_cap_flag = 1;
				tmp_cap_d = LOW_CAP - 1;				
			}
		}
        
	   }
	}
	else{
		set_tmp_cap_d_flag = 0;
		if(set_init_cap_flag == 0){    //每次开机计算一次
			set_init_cap_flag = 1;
			tmp_vol = vol;
			set_tmp_cap_d_flag = 0;
			tmp_cap_s =  100.0 * (float)(vol + BATT_VOL_OFFSET - BAT_LOW_S )/(float)(BAT_FULL_S - BAT_LOW_S);
		
		}
		if(batt_fly_status == 0){    //原先降落5秒开始计算电量，现在不计算
			if(vol < tmp_vol){
				tmp_vol = vol;			
				tmp_cap_s =  100.0 * (float)(vol + BATT_VOL_OFFSET - BAT_LOW_S )/(float)(BAT_FULL_S - BAT_LOW_S);
            
			}
			
		}

        if(batt_flag == 0)             //每次开机计算,起飞之后不再计算
      	{
      		
		if(vol > 4300){
			//tmp_cap_s=100;
			cap_t=100;
			cap_min=0;
			cap_max=1;
			cap_vel=0;
		}else if(4300>= vol && vol >4110){
			//tmp_cap_s=90;
			cap_t=88;
			cap_min=4110;
			cap_max=4300;
			cap_vel=12;
		}else if(4110 >= vol && vol > 3930){
			//tmp_cap_s=80;
			cap_t=66;
			cap_min=3930;
			cap_max=4110;
			cap_vel=22;
		}else if(3930>= vol && vol > 3872){
			//tmp_cap_s=30;
			cap_t=38;
			cap_min=3872;
			cap_max=3930;
			cap_vel=18;
		}else if(3872 >= vol && vol > 3588){
			//tmp_cap_s=20;
			cap_t=10;
			cap_min=3588;
			cap_max=3872;
			cap_vel=28;
		}else if(3588 >= vol){
			//tmp_cap_s=15;
			cap_t=9;
			cap_vel=0;
			cap_max=1;
			cap_min=0;
		}
      //  batt_flag=1;
       if(cap_max != cap_min){
        tmp_cap_s = cap_t + cap_vel*(vol - cap_min)/(cap_max - cap_min);  //除数不能为0
       	}
	   
      	}
     
		if(tmp_cap_s > FULL_CAP ){
			tmp_cap_s = FULL_CAP;
		}	
		

	}
	

	if(system_get_armed() == true){
		batt_cap = tmp_cap_d;
		
	}else{
		if(batt_fly_status == 1){			
			batt_wait_dt += dt;
			if(batt_wait_dt > batt_wait_time){   //降落时间检测，刷新电量
				batt_fly_status = 0;
				batt_wait_dt = 0.0f;
			}else{
				batt_cap = tmp_cap_d;
			}			
		}else{
			if(tmp_cap_s < tmp_cap_d){
				batt_cap = tmp_cap_s; 
				
			}else{
				batt_cap = tmp_cap_d;
			}
		}
	}
#if 0
	if(batt_charging == 1){
		if(batt_full == 1){
			batt_cap = 100;
		}else{	
			charge_cap = 100.0 * (float)(vol +  BAT_LOW_S )/(float)(BAT_FULL_S - BAT_LOW_S);
			batt_cap = charge_cap;
		}	
		low_cap_flag = 0;
	}	
#endif
	if(batt_cap < ZERO_CAP){
		batt_cap = ZERO_CAP; 
	}	

}

void batt_init(void)
{
	INFO(DEBUG_ID,"init");
}
void batt_update(float dt)
{
	static float batt_sample_dt;
	static float batt_dt;
	static uint16_t bat_val_buf[5];
	int sum = 0;
	static uint8_t bat_val_cnt; 
	uint8_t val[5];
	uint16_t bat_val;
	uint8_t i,j;
	int vol;
	float batt_val;
	int temp;
	int batt_status;
	batt_dt += dt;
	batt_sample_dt += dt;
	if(batt_sample_dt > 0.2f){
		batt_sample_dt = 0;
		stm8s_read(val,5);
		bat_val = val[0];
		bat_val <<= 8;
		bat_val += val[1];
		bat_val_buf[bat_val_cnt] = bat_val;
		bat_val_cnt++;
		raw_adc_val = bat_val;
	}
	if(bat_val_cnt == BAT_VAL_NUM){		
		bat_val_cnt = 0;
		for(j = 0;j < BAT_VAL_NUM-1;j++){
			for(i = 0;i < BAT_VAL_NUM-1-j;i++){
				if(bat_val_buf[i] > bat_val_buf[i+1]){
					temp = bat_val_buf[i];
					bat_val_buf[i] = bat_val_buf[i+1];
					bat_val_buf[i+1] = temp;
				}	
			}	
		}
		for(i = 1;i < BAT_VAL_NUM-1;i++){
			sum += bat_val_buf[i];
		}
		bat_val = sum/(BAT_VAL_NUM-2);
		batt_val = bat_val * BAT_ADC_SCALE * 3.3f /1023.0f ;
		vol = (int)(batt_val * 1000.0f);

		raw_bat_val = vol;
		batt_calc(batt_dt,vol);
		batt_dt = 0.0f;
	}
	if(hal_get_batt(dt,&batt_status) == true){
		if(batt_status == 0){
			batt_charging = 0;
			batt_full = 0;
		}else if(batt_status == 1){
			batt_charging = 0;
			batt_full = 0;
		}else{
			batt_full = 1;
		}		
	}
	DEBUG_HZ(DEBUG_ID,2,dt,"cap:%d vol:%d charging:%d",batt_cap,batt_vol,batt_charging);
}


