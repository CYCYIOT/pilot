#include "app_attitude.h"
#include "lib_math.h"

static float air_res_break_angle;
static float air_res_coff;

static float break_time[2];
static float break_angle[2];
static float integral_angle_ned[2];
static float integral_angle_body[2];

void air_res_break_set_param(float angle,float coff)
{
	air_res_break_angle = angle;	//angle + att_offset can't larger than att limit
	air_res_coff = coff; 			// break time is larger when AIR_RES_COFF is smaller
}
void air_res_break_get_time(float time[2])
{
	time[0] = break_time[0];
	time[1] = break_time[1];
}

void air_res_break_get_angle(float angle[2])
{
	angle[0] = break_angle[0];
	angle[1] = break_angle[1];
}

float air_res_break_get_air_res_coff()
{
	return air_res_coff;
}
void air_res_break_get_angle_integrate(float integrate[2])
{
	integrate[0] = integral_angle_body[0];
	integrate[1] = integral_angle_body[1];
}

void air_res_break_clear()
{
	integral_angle_ned[0] = 0;
	integral_angle_ned[1] = 0;
	integral_angle_body[0] = 0;
	integral_angle_body[1] = 0;		
}

void air_res_break_angle_integrate(float dt,att_s att,float att_offset[2])
{
	float angle_body[2];
	float angle_ned[2];
	float yaw_cos = 0.0f;
 	float yaw_sin = 0.0f;
	
	angle_body[0] = att.att[0] - att_offset[0];
	angle_body[1] = att.att[1] - att_offset[1];
	
	yaw_cos = cosf(radians(att.att[2]));
	yaw_sin = sinf(radians(att.att[2]));
	angle_ned[0] = angle_body[0] * yaw_cos - angle_body[1] * yaw_sin;
	angle_ned[1] = angle_body[0] * yaw_sin + angle_body[1] * yaw_cos;
	
//	air_res_coff = (1-fabs(att.r[2][2])) * AIR_RES_COFF+ 0.1f; // AIR_RES_COFF = 8.0f

	integral_angle_ned[0] += (angle_ned[0] - air_res_coff * integral_angle_ned[0]) * dt;
	integral_angle_ned[1] += (angle_ned[1] - air_res_coff * integral_angle_ned[1]) * dt;
	integral_angle_body[0] =  integral_angle_ned[0] * yaw_cos + integral_angle_ned[1] * yaw_sin; 
	integral_angle_body[1] = -integral_angle_ned[0] * yaw_sin + integral_angle_ned[1] * yaw_cos;
	break_time[0] = fabs(integral_angle_body[0] / air_res_break_angle);
	break_time[1] = fabs(integral_angle_body[1] / air_res_break_angle);
	break_angle[0] = (integral_angle_body[0] > 0 ?  -air_res_break_angle : air_res_break_angle) + att_offset[0];
	break_angle[1] = (integral_angle_body[1] > 0 ?  -air_res_break_angle : air_res_break_angle) + att_offset[1];

}



