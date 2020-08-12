#include <fcntl.h>

#include "app_debug.h"

#include "lib_math.h"
#include "param.h"

#define DEBUG_ID DEBUG_ID_HAL

#define FBM320_PATH "/dev/fbm320"

typedef struct{
	uint16_t C0, C1, C2, C3, C6, C8, C9, C10, C11, C12; 
	uint32_t C4, C5, C7;
	uint8_t ver;
} fbm320_calib_data;

typedef struct{
	uint8_t mode;
	int32_t pressure;
	int32_t temp;
	fbm320_calib_data calib;
}fbm320_report_s;

#define FBM320_UPDATE_PRESS	0.013 //second
#define FBM320_UPDATE_TEMP	0.006 //second

static int fbm320_fd = -1;

float fbm320_altitude_convert(int32_t Press, int32_t Ref_P)										//Calculate relative altitude
{
	return 44330 * (1 - powf(((float)Press / (float)Ref_P), (1/5.255)));
}

float fbm320_altitude_convert_new(int32_t Press)																				//Calculate absolute altitude, unit: mm
{
	int8_t P0;			
	int16_t hs1, dP0;			
	int32_t h0, hs0, HP1, HP2;			
	int32_t alt;
	
	if(Press >= 103000)
	{	
		P0	=	103;
		h0	=	-138507;
		hs0	=	-21007;
		hs1	=	311;
	}	
	else if(Press >= 98000)
	{	
		P0	=	98;
		h0	=	280531;
		hs0	=	-21869;
		hs1	=	338;
	}	
	else if(Press >= 93000)
	{	
		P0	=	93;
		h0	=	717253;
		hs0	=	-22813;
		hs1	=	370;
	}	
				
	else if(Press >= 88000)
	{	
		P0	=	88;
		h0	=	1173421;
		hs0	=	-23854;
		hs1	=	407;
	}	
	else if(Press >= 83000)
	{	
		P0	=	83;
		h0	=	1651084;
		hs0	=	-25007;
		hs1	=	450;
	}	
	else if(Press >= 78000)
	{	
		P0	=	78;
		h0	=	2152645;
		hs0	=	-26292;
		hs1	=	501;
	}	
	else if(Press >= 73000)
	{	
		P0	=	73;
		h0	=	2680954;
		hs0	=	-27735;
		hs1	=	560;
	}	
	else if(Press >= 68000)
	{	
		P0	=	68;
		h0	=	3239426;
		hs0	=	-29366;
		hs1	=	632;
	}	
	else if(Press >= 63000)
	{	
		P0	=	63;
		h0	=	3832204;
		hs0	=	-31229;
		hs1	=	719;
	}	
	else if(Press >= 58000)
	{	
		P0	=	58;
		h0	=	4464387;
		hs0	=	-33377;
		hs1	=	826;
	}	
	else if(Press >= 53000)
	{	
		P0	=	53;
		h0	=	5142359;
		hs0	=	-35885;
		hs1	=	960;
	}		
	else if(Press >= 48000)
	{	
		P0	=	48;
		h0	=	5874268;
		hs0	=	-38855;
		hs1	=	1131;
	}	
	else if(Press >= 43000)
	{	
		P0	=	43;
		h0	=	6670762;
		hs0	=	-42434;
		hs1	=	1354;
	}	
	else if(Press >= 38000)
	{	
		P0	=	38;
		h0	=	7546157;
		hs0	=	-46841;
		hs1	=	1654;
	}	
	else if(Press >= 33000)
	{	
		P0	=	33;
		h0	=	8520395;
		hs0	=	-52412;
		hs1	=	2072;
	}	
	else
	{	
		P0	=	28;
		h0	=	9622536;
		hs0	=	-59704;
		hs1	=	2682;
	}
					
	dP0	=	Press - P0 * 1000;
	HP1	=	(hs0 * dP0) >> 2;
	HP2	=	(((hs1 * dP0) >> 10)* dP0) >> 4;			

	alt = ((h0 << 6) + HP1 + HP2) >> 6;

	return	(float)(alt / 1000.0f);										//Return absolute altitude
}


void fbm320_convert_new(int32_t temp,int32_t pressure,fbm320_calib_data calib,int32_t * RT,int32_t * RP)
{
	int32_t DT, DT2, X01, X02, X03, X11, X12, X13, X21, X22, X23, X24, X25, X26, X31, X32, CF, PP1, PP2, PP3, PP4;
	int32_t dC05, dRT, DPC2, DP;
	int32_t BT1, BT2, DPTS1, DPTI1, DPTS2, DPTI2, DPC2S0, DPC2S1;

	if(calib.ver == 3)																									//Version: FBM320-03, RPC-01+02 coefficient
	{
		DPC2S0 = 559;
		DPC2S1 = 549;
		BT1 = 2000;
		BT2 = 700;
		DPTS1 = -1513;
		DPTI1 = 46;
		DPTS2 = 819;
		DPTI2 = -9;
	}

	DT	=	((temp - 8388608) >> 4) + (calib.C0 << 4);
	X01	=	(calib.C1 + 4459) * DT >> 1;
	X02	=	((((calib.C2 - 256) * DT) >> 14) * DT) >> 4;
	X03	=	(((((calib.C3 * DT) >> 18) * DT) >> 18) * DT);
	*RT	=	((2500 << 15) - X01 - X02 - X03) >> 15;
				
	DT2	=	(X01 + X02 + X03) >> 12;
				
	X11	=	((calib.C5 - 4443) * DT2);
	X12	=	(((calib.C6 * DT2) >> 16) * DT2) >> 2;
	X13	=	((X11 + X12) >> 10) + ((calib.C4 + 120586) << 4);
				
	X21	=	((calib.C8 + 7180) * DT2) >> 10;
	X22	=	(((calib.C9 * DT2) >> 17) * DT2) >> 12;
	X23 =	abs(X22 - X21);

	X24	=	(X23 >> 11) * (calib.C7 + 166426);
	X25	=	((X23 & 0x7FF) * (calib.C7 + 166426)) >> 11;
	X26 = (X21 >= X22) ? (((0 - X24 - X25) >> 11) + calib.C7 + 166426) : (((X24 + X25) >> 11) + calib.C7 + 166426);

	PP1	=	((pressure - 8388608) - X13) >> 3;
	PP2	=	(X26 >> 11) * PP1;
	PP3	=	((X26 & 0x7FF) * PP1) >> 11;
	PP4	=	(PP2 + PP3) >> 10;
				
	CF	=	(2097152 + calib.C12 * DT2) >> 3;
	X31	=	(((CF * calib.C10) >> 17) * PP4) >> 2;
	X32	=	(((((CF * calib.C11) >> 15) * PP4) >> 18) * PP4);
	*RP	=	((X31 + X32) >> 15) + PP4 + 100000;

	if(calib.ver == 3){
		dC05 = calib.C5 - 16384;
		dRT = *RT - 2500;
		DPC2 = (((DPC2S1 * dC05) >> 9) + DPC2S0) >> 4;
		DP = (((DPC2 * dRT) >> 15) * dRT) >> 11;
		*RP -= DP;

		if(*RT < BT1) {
			DP = ((*RT * DPTS1) >> 16) + DPTI1;
			if(*RT < BT2){
				DP = ((*RT * DPTS2) >> 16) + DPTI2 + DP;
			}
			*RP -= DP;
		}
	}
}


void fbm320_convert(int32_t temp,int32_t pressure,fbm320_calib_data calib,int32_t * real_temp,int32_t * real_pressure)
{
	int32_t DT, DT2, X01, X02, X03, X11, X12, X13, X21, X22, X23, X24, X25, X26, X31, X32, CF, PP1, PP2, PP3, PP4;
	
	DT = ((temp - 8388608) >> 4) + (calib.C0 << 4);
	X01 = (calib.C1 + 4459) * DT >> 1;
	X02 = ((((calib.C2 - 256) * DT) >> 14) * DT) >> 4;
	X03 = (((((calib.C3 * DT) >> 18) * DT) >> 18) * DT);
	*real_temp = ((2500 << 15) - X01 - X02 - X03) >> 15;
				
	DT2 = (X01 + X02 + X03) >> 12;
				
	X11 = ((calib.C5 - 4443) * DT2);
	X12 = (((calib.C6 * DT2) >> 16) * DT2) >> 2;
	X13 = ((X11 + X12) >> 10) + ((calib.C4 + 120586) << 4);
				
	X21 = ((calib.C8 + 7180) * DT2) >> 10;
	X22 = (((calib.C9 * DT2) >> 17) * DT2) >> 12;
	X23 = (X22 >= X21) ? (X22 - X21) : (X21 - X22);

	X24 = (X23 >> 11) * (calib.C7 + 166426);
	X25 = ((X23 & 0x7FF) * (calib.C7 + 166426)) >> 11;
	X26 = (X21 >= X22) ? (((0 - X24 - X25) >> 11) + calib.C7 + 166426) : (((X24 + X25) >> 11) + calib.C7 + 166426);

	PP1 = ((pressure - 8388608) - X13) >> 3;
	PP2 = (X26 >> 11) * PP1;
	PP3 = ((X26 & 0x7FF) * PP1) >> 11;
	PP4 = (PP2 + PP3) >> 10;
				
	CF = (2097152 + calib.C12 * DT2) >> 3;
	X31 = (((CF * calib.C10) >> 17) * PP4) >> 2;
	X32 = (((((CF * calib.C11) >> 15) * PP4) >> 18) * PP4);
	*real_pressure = ((X31 + X32) >> 15) + PP4 + 99880;
}

bool fbm320_open()
{
	fbm320_fd = open(FBM320_PATH, O_RDONLY);
	if(fbm320_fd < 0){
		DEBUG(DEBUG_ID,"open fbm320 failed (%s)", FBM320_PATH);
		return false;
	}else{
		INFO(DEBUG_ID,"use fbm320 baro");
	}

	return true;
}

bool fbm320_read(float dt,float * pres,float * alt,float * temp)
{
	static float fbm320_timer = 0;
	static float fbm320_init_time = 0;
	static fbm320_report_s report_fbm320 = {.mode = 0};
	static int32_t fbm320_temp = 0;
	static int32_t fbm320_pressure = 0;
	static float fbm320_alt_gound = 0.0f;
	float fbm320_alt;

	fbm320_timer += dt;
	fbm320_init_time += dt;
	if(report_fbm320.mode == 1){
		if(fbm320_timer > FBM320_UPDATE_PRESS){
			fbm320_timer = 0.0f;
		}else{
			return false;
		}
	}else{
		if(fbm320_timer > FBM320_UPDATE_TEMP){
			fbm320_timer = 0.0f;
		}else{
			return false;
		}
	}

	if(read(fbm320_fd, &report_fbm320, sizeof(report_fbm320)) > 0){
		//printf("%d %d %d\r\n",report_fbm320.temp,report_fbm320.pressure,report_fbm320.mode);
		if(report_fbm320.mode){
			if(report_fbm320.temp != report_fbm320.pressure){
				fbm320_convert_new(report_fbm320.temp,report_fbm320.pressure,report_fbm320.calib,&fbm320_temp,&fbm320_pressure);
				fbm320_alt = fbm320_altitude_convert_new(fbm320_pressure);
				//printf("fbm320:%+3.3f %d %d\r\n",fbm320_alt,fbm320_temp,fbm320_pressure);
				if(!isnan(fbm320_alt) && fbm320_init_time >= 1.0f){
					*alt = fbm320_alt - fbm320_alt_gound;
					*temp = (float)fbm320_temp / 100.0f;
					*pres = fbm320_pressure;

					report_fbm320.mode = 0;
					report_fbm320.temp = 0;
					report_fbm320.pressure = 0;
					return true;
				}else{
					fbm320_alt_gound = fbm320_alt;
				}
			}else{
				INFO(DEBUG_ID,"fbm320 error %d %d %d\r\n",report_fbm320.temp,report_fbm320.pressure,report_fbm320.mode);
			}
			report_fbm320.mode = 0;
			report_fbm320.temp = 0;
			report_fbm320.pressure = 0;
			return false;
		}else{
			report_fbm320.mode = 1;
			return false;
		}
	}else{
		return false;
	}
}

