#ifndef _MATHLIB_H_
#define _MATHLIB_H_

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>
#include <unistd.h>
#include <float.h>
#include <time.h> 
#include <string.h>

#include <sys/time.h>

#define CONSTANTS_ONE_G 9.80665f //m/s^2

#ifndef M_PI
#define M_PI		(3.14159265358979323846f)
#endif
#ifndef M_PI_F
#define M_PI_F		(float)(M_PI)
#endif

#define DEG_TO_RAD	(M_PI / 180.0f)
#define RAD_TO_DEG	(180.0f / M_PI)

static inline float complementary_filter(float input1,float input2,float filter)
{
	return ((input1 * filter) + (input2 * (1.0f - filter)));
}

static inline float radians(float deg)
{
	return deg * DEG_TO_RAD;   //角度转弧度
}

static inline float degrees(float rad)
{
	return rad * RAD_TO_DEG;  //弧度转角度
}

static inline float invSqrt(float x)
{
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}

static inline int min(int val1, int val2)
{
	return (val1 < val2) ? val1 : val2;
}

static inline int max(int val1, int val2)
{
	return (val1 > val2) ? val1 : val2;
}

static inline float sq(float v) 
{
	return v*v;
}

static inline float pythagorous2(float a, float b)
{
	return sqrtf(sq(a)+sq(b));
}

static inline float pythagorous3(float a, float b, float c) 
{
	return sqrtf(sq(a)+sq(b)+sq(c));
}

static inline float pythagorous_v3f(float data[3]) 
{
	return sqrtf(sq(data[0])+sq(data[1])+sq(data[2]));
}

static inline float constrain_float(float amt, float low, float high)
{
	if (isnan(amt)) {
		return (low+high)*0.5f;
	}
	return ((amt)<(low)?(low):((amt)>(high)?(high):(amt)));
}

static inline int8_t constrain_int8(int8_t amt, int8_t low, int8_t high) 
{
	return ((amt)<(low)?(low):((amt)>(high)?(high):(amt)));
}

static inline int16_t constrain_int16(int16_t amt, int16_t low, int16_t high) 
{
	return ((amt)<(low)?(low):((amt)>(high)?(high):(amt)));
}

static inline uint16_t constrain_uint16(uint16_t amt, uint16_t low, uint16_t high) 
{
	return ((amt)<(low)?(low):((amt)>(high)?(high):(amt)));
}

static inline int32_t constrain_int32(int32_t amt, int32_t low, int32_t high)
{
	return ((amt)<(low)?(low):((amt)>(high)?(high):(amt)));
}

static inline float get_diff_time(struct timespec * start , bool update)
{
	float dt;
	struct timespec now;
	
	clock_gettime(CLOCK_MONOTONIC,&now);
	dt = (float)(now.tv_sec  - start->tv_sec);
	dt += (float)(now.tv_nsec - start->tv_nsec) * 1e-9;

	if(update == true){
		start->tv_sec = now.tv_sec;
		start->tv_nsec = now.tv_nsec;
	}

	return dt;
}

static inline float wrap_180_cd_float(float angle)
{
    if (angle > 540.0f || angle < -540.0f) {
        // for large numbers use modulus
        angle = fmod(angle,360.0f);
    }
    if (angle > 180.0f) { angle -= 360.0f; }
    if (angle < -180.0f) { angle += 360.0f; }
    return angle;
}

static inline float wrap_360_cd_float(float angle)
{
    if (angle >= 720.0f || angle < -360.0f) {
        // for larger number use fmodulus
        angle = fmod(angle, 360.0f);
    }
    if (angle >= 360.0f) angle -= 360.0f;
    if (angle < 0.0f) angle += 360.0f;
    return angle;
}

static inline void bf_to_ef(float r[3][3],float bf[3],float ef[3])
{
	int i,j;

	for (i = 0; i < 3; i++) {
		ef[i] = 0.0f;
		for (j = 0; j < 3; j++){
			ef[i] += r[i][j] * bf[j];
		}
	}
}

static inline void ef_to_bf(float r[3][3],float bf[3],float ef[3])
{
	int i,j;
	for (i = 0; i < 3; i++) {
		bf[i] = 0.0f;
	
		for (j = 0; j < 3; j++) {
			bf[i] += r[j][i] * ef[j];
		}
	}
}

//true mean input float is zero
static inline bool float_is_zero(float d)
{
	if (d >= -FLT_EPSILON && d <= FLT_EPSILON){
		return true;
	}else{
		return false;
	}
}

static inline void v3f_set(float out[3],float in1[3])
{
	out[0] = in1[0];
	out[1] = in1[1];
	out[2] = in1[2];
}

static inline void v3f_set_val(float out[3],float val)
{
	out[0] = val;
	out[1] = val;
	out[2] = val;
}

static inline void v2f_set(float out[3],float in1[3])
{
	out[0] = in1[0];
	out[1] = in1[1];
}

static inline void v2f_set_val(float out[2],float val)
{
	out[0] = val;
	out[1] = val;
}

static inline void v20_set_val(int out[4][5],int in[4][5])
{
 int i=0;
 int j=0;
 for(i=0;i<4;i++){
  for(j=0;j<5;j++){
      out[i][j]=in[i][j];  
  }
 }

}
static inline uint16_t crc16_init()
{
	return 0xffff;
}

static inline uint16_t crc16_update(uint8_t data,uint16_t crc)
{
	uint8_t tmp;
	
	tmp = data ^ (uint8_t)(crc & 0xff);
	tmp ^= (tmp<<4);
	crc = (crc>>8) ^ (tmp<<8) ^ (tmp <<3) ^ (tmp>>4);

	return crc;
}

#endif

