#include "app_debug.h"
#include "app_param.h"
#include "app_nav.h"

#include "ahrs.h"
#include "lib_math.h"

#define DEBUG_ID DEBUG_ID_ATTITUDE

uint8_t ahrs_mode = AHRS_MODE_DISARM;

float ahrs_rp_p;
float ahrs_rp_i;
float ahrs_yaw_p;
float ahrs_yaw_i;

float ahrs_rp_action_check_timeout;
float ahrs_rp_action_check_time;
float ahrs_rp_action_check;
float ahrs_rp_p_action;
float ahrs_rp_i_action;

float ahrs_rp_p_arm;
float ahrs_rp_i_arm;
float ahrs_rp_p_disarm;
float ahrs_rp_i_disarm;

float ahrs_yaw_p_disarm;
float ahrs_yaw_i_disarm;
float ahrs_yaw_p_arm;
float ahrs_yaw_i_arm;

bool ahrs_flip_mode = false;

void ahrs_param_init()
{
	param_set_var(AHRS__RP_ACTION_CHECK_TIMEOUT_NAME	,&ahrs_rp_action_check_timeout);
	param_set_var(AHRS__RP_ACTION_CHECK_NAME			,&ahrs_rp_action_check);
	param_set_var(AHRS__RP_P_ACTION_NAME				,&ahrs_rp_p_action);
	param_set_var(AHRS__RP_I_ACTION_NAME				,&ahrs_rp_i_action);

	param_set_var(AHRS__RP_P_ARM_NAME			,&ahrs_rp_p_arm);
	param_set_var(AHRS__RP_I_ARM_NAME			,&ahrs_rp_i_arm);
	param_set_var(AHRS__YAW_P_ARM_NAME			,&ahrs_yaw_p_arm);
	param_set_var(AHRS__YAW_I_ARM_NAME			,&ahrs_yaw_i_arm);

	param_set_var(AHRS__RP_P_DISARM_NAME		,&ahrs_rp_p_disarm);
	param_set_var(AHRS__RP_I_DISARM_NAME		,&ahrs_rp_i_disarm);
	param_set_var(AHRS__YAW_P_DISARM_NAME		,&ahrs_yaw_p_disarm);
	param_set_var(AHRS__YAW_I_DISARM_NAME		,&ahrs_yaw_i_disarm);
}

void ahrs_set_flip_mode(bool enable)
{
	ahrs_flip_mode = enable;
	INFO(DEBUG_ID,"flip mode:%d",ahrs_flip_mode);
}

void ahrs_init(ahrs_estimator *est,float *acc,float *mag)
{ 
	float xh;
	float yh;
	float att[3];
	float att_calc;

	est->inited = true;

	v3f_set_val(est->gyro_err_int,0);
	v3f_set_val(est->mag_err_int,0);

	INFO(DEBUG_ID,"acc: %3.3f %3.3f %3.3f",acc[0],acc[1],acc[2]);
	INFO(DEBUG_ID,"mag: %3.3f %3.3f %3.3f",mag[0],mag[1],mag[2]);
	if(fabs(acc[0]) > (CONSTANTS_ONE_G + 1.0f) || fabs(acc[1]) > (CONSTANTS_ONE_G + 1.0f) || fabs(acc[2]) > (CONSTANTS_ONE_G + 1.0f)){
		INFO(DEBUG_ID,"ahrs init false");
		est->inited = false;
	}
	
	att_calc = constrain_float(-acc[0]/(-CONSTANTS_ONE_G),-1.0f,1.0f);

	att[0] = -atan2f(acc[1],-acc[2]);
	att[1] = asinf(att_calc);
	xh = mag[0]*cosf(att[1]) + mag[1]*sinf(att[0])*sinf(att[1]) + mag[2]*cosf(att[0])*sinf(att[1]);
	yh = mag[2]*sinf(att[0]) - mag[1]*cosf(att[0]);

	if((xh == 0.0f) && (yh == 0.0f)){		
		att[2] = 0.0f;
	}else{
		att[2] = atan2f(yh,xh);
	}
	
	att[0] = degrees(att[0]);
	att[1] = degrees(att[1]);
	att[2] = degrees(att[2]);
	att[2] = wrap_180_cd_float(att[2]);

	ahrs_init_from_euler(est,att);

	ahrs_rp_action_check_time = ahrs_rp_action_check_timeout;

	INFO(DEBUG_ID,"init att:%3.3f,%3.3f,%3.3f",att[0],att[1],att[2]);
}

uint8_t ahrs_get_mode()
{
	return ahrs_mode;
}

float ahrs_get_check_time()
{
	return ahrs_rp_action_check_time;
}

float ahrs_get_rp_p()
{
	return ahrs_rp_p;
}

float ahrs_get_rp_i()
{
	return ahrs_rp_i;
}

void ahrs_apply_acc(ahrs_estimator *est,float acc[3])
{
	v3f_set(est->acc_raw,acc);
	est->acc_updated = true;
}

void ahrs_apply_gyro(ahrs_estimator *est,float gyro[3])
{
	v3f_set(est->gyro_raw,gyro);
	est->gyro_updated = true;
}

void ahrs_apply_mag(ahrs_estimator *est,float mag[3])
{
	v3f_set(est->mag_raw,mag);
	est->mag_updated = true;
}

void ahrs_update(float dt,ahrs_estimator *est,bool armed)
{
	float recipNorm;
	float halfvx, halfvy, halfvz;
	float halfXx, halfXy, halfXz;
	float halfex, halfey, halfez;
	float halfmx, halfmy, halfmz;
	float hx, hy, hz, bx, bz;
	float qa, qb, qc,qd;
	float acc[3],gyro[3],mag[3];
	
	float gx_g=0.0f;
	float gy_g=0.0f;
	float gz_g=0.0f;
	float gx_m=0.0f;
	float gy_m=0.0f;
	float gz_m=0.0f;

	if(armed == false){
		ahrs_mode = AHRS_MODE_DISARM;
		ahrs_rp_action_check_time = ahrs_rp_action_check_timeout;
	}else{
		if(ahrs_flip_mode == true){
			ahrs_mode = AHRS_MODE_FLIP;
		}else{
			if(fabs(nav_get_acc_body_sum() - CONSTANTS_ONE_G) > ahrs_rp_action_check){
				ahrs_mode = AHRS_MODE_ACTION;
				ahrs_rp_action_check_time = 0.0f;
			}else{
				ahrs_rp_action_check_time += dt;
			}

			if(ahrs_rp_action_check_time >= ahrs_rp_action_check_timeout){
				ahrs_rp_action_check_time = ahrs_rp_action_check_timeout;
				ahrs_mode = AHRS_MODE_NORMAL;
			}
		}
	}

	if(ahrs_mode == AHRS_MODE_DISARM){
		ahrs_rp_p = ahrs_rp_p_disarm;
		ahrs_rp_i = ahrs_rp_i_disarm;
		ahrs_yaw_p = ahrs_yaw_p_disarm;
		ahrs_yaw_i = ahrs_yaw_i_disarm;
	}else if(ahrs_mode == AHRS_MODE_NORMAL){
		ahrs_rp_p = ahrs_rp_p_arm;
		ahrs_rp_i = ahrs_rp_i_arm;
		ahrs_yaw_p = ahrs_yaw_p_arm;
		ahrs_yaw_i = ahrs_yaw_i_arm;
	}else if(ahrs_mode == AHRS_MODE_FLIP){
		ahrs_rp_p = 0.0f;
		ahrs_rp_i = 0.0f;
		ahrs_yaw_p = 0.0f;
		ahrs_yaw_i = 0.0f;
	}else if(ahrs_mode == AHRS_MODE_ACTION){
		ahrs_rp_p = ahrs_rp_p_action;
		ahrs_rp_i = ahrs_rp_i_action;
		ahrs_yaw_p = ahrs_yaw_p_arm;
		ahrs_yaw_i = ahrs_yaw_i_arm;
	}
	
	v3f_set(acc,est->acc_raw);
	v3f_set(gyro,est->gyro_raw);
	v3f_set(mag,est->mag_raw);

	halfmx = 0.0f;
	halfmy = 0.0f;
	halfmz = 0.0f;
	
	if(est->acc_updated == true && est->gyro_updated == true){
	    // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	    if(!((acc[0] == 0.0f) && (acc[1] == 0.0f) && (acc[2] == 0.0f)))
	    {
		    // Normalise accelerometer measurement
		    recipNorm = invSqrt(sq(acc[0]) + sq(acc[1]) + sq(acc[2]));
		    acc[0] *= recipNorm;
		    acc[1] *= recipNorm;
		    acc[2] *= recipNorm;

		    // Estimated direction of gravity and vector perpendicular to magnetic flux
		    halfvx = est->q[1] * est->q[3] - est->q[0] * est->q[2];
		    halfvy = est->q[0] * est->q[1] + est->q[2] * est->q[3];
		    halfvz = 0.5f - est->q[1]*est->q[1] - est->q[2]*est->q[2];

		    // Error is sum of cross product between estimated and measured direction of gravity
		    halfex = -(acc[1] * halfvz - acc[2] * halfvy);
		    halfey = -(acc[2] * halfvx - acc[0] * halfvz);
		    halfez = -(acc[0] * halfvy - acc[1] * halfvx);

		    // Compute and apply integral feedback if enabled
		    if(ahrs_rp_i >= 0.0f)
		    {
				est->gyro_err_int[0] += ahrs_rp_i * halfex * dt;  // integral error scaled by Ki
				est->gyro_err_int[1] += ahrs_rp_i * halfey * dt;
				est->gyro_err_int[2] += ahrs_rp_i * halfez * dt;
				gx_g = gyro[0];
				gy_g = gyro[1];
				gz_g = gyro[2];
				gx_g += est->gyro_err_int[0];  // apply integral feedback
				gy_g += est->gyro_err_int[1];
				gz_g += est->gyro_err_int[2];
		    }
		    else
		    {
				est->gyro_err_int[0] = 0.0f; // prevent integral windup
				est->gyro_err_int[1] = 0.0f;
				est->gyro_err_int[2] = 0.0f;
		    }

		    // Apply proportional feedback
		    gx_g += ahrs_rp_p * halfex;
		    gy_g += ahrs_rp_p * halfey;
		    gz_g += ahrs_rp_p * halfez;
	    }
		est->acc_updated  = false;
		est->gyro_updated = false;
	}else{
		return;
	}

	if(est->mag_updated == true){
		if(!((mag[0] == 0.0f) && (mag[1] == 0.0f) && (mag[2] == 0.0f)))
	    {
		    // Normalise accelerometer measurement
		    recipNorm = invSqrt(mag[0] * mag[0] + mag[1] * mag[1] + mag[2] * mag[2]);
		    mag[0] *= recipNorm;
		    mag[1] *= recipNorm;
		    mag[2] *= recipNorm;

			// compute reference direction of flux
	        hx = 2.0f * (mag[0] * (0.5f - est->q[2]*est->q[2] - est->q[3]*est->q[3]) + mag[1] * (est->q[1]*est->q[2] - est->q[0]*est->q[3]) + mag[2] * (est->q[1]*est->q[3] + est->q[0]*est->q[2]));

	        hy = 2.0f * (mag[0] * (est->q[1]*est->q[2] + est->q[0]*est->q[3]) + mag[1] * (0.5f - est->q[1]*est->q[1] - est->q[3]*est->q[3]) + mag[2] * (est->q[2]*est->q[3] - est->q[0]*est->q[1]));

	        hz = 2.0f * (mag[0] * (est->q[1]*est->q[3] - est->q[0]*est->q[2]) + mag[1] * (est->q[2]*est->q[3] + est->q[0]*est->q[1]) + mag[2] * (0.5f - est->q[1]*est->q[1] - est->q[2]*est->q[2]));

	        bx = sqrt((hx * hx) + (hy * hy));

	        bz = hz;

	        // estimated direction of flux (w)
	        halfXx = 2.0f * (bx * (0.5f - est->q[2]*est->q[2] - est->q[3]*est->q[3]) + bz * (est->q[1]*est->q[3] - est->q[0]*est->q[2]));

	        halfXy = 2.0f * (bx * (est->q[1]*est->q[2] - est->q[0]*est->q[3]) + bz * (est->q[0]*est->q[1] + est->q[2]*est->q[3]));

	        halfXz = 2.0f * (bx * (est->q[0]*est->q[2] + est->q[1]*est->q[3]) + bz * (0.5f - est->q[1]*est->q[1] - est->q[2]*est->q[2]));

		    // error
			halfmx = (mag[1] * halfXz - mag[2] * halfXy);
			halfmy = (mag[2] * halfXx - mag[0] * halfXz);
			halfmz = (mag[0] * halfXy - mag[1] * halfXx);
		  

		    // Compute and apply integral feedback if enabled
		    if(ahrs_yaw_i >= 0.0f)
		    {
				est->mag_err_int[0] += ahrs_yaw_i * halfmx * dt;  // integral error scaled by Ki
				est->mag_err_int[1] += ahrs_yaw_i * halfmy * dt;
				est->mag_err_int[2] += ahrs_yaw_i * halfmz * dt;
				gx_m += est->mag_err_int[0];  // apply integral feedback
				gy_m += est->mag_err_int[1];
				gz_m += est->mag_err_int[2];
			}
		    else
		    {
				est->mag_err_int[0] = 0.0f; // prevent integral windup
				est->mag_err_int[1] = 0.0f;
				est->mag_err_int[2] = 0.0f;
		    }
			
		    // Apply proportional feedback	
		    gx_m += ahrs_yaw_p * halfmx;
		    gy_m += ahrs_yaw_p * halfmy;
		    gz_m += ahrs_yaw_p * halfmz;
	    }
		est->mag_updated = false;
	}else{
		gx_m = 0.0f; 
 		gy_m = 0.0f;
		gz_m = 0.0f;	
	}

	est->gyro[0] = gx_g;
	est->gyro[1] = gy_g;
	est->gyro[2] = gz_g + gz_m;

	gyro[0] = gx_g;
	gyro[1] = gy_g;
	gyro[2] = gz_g + gz_m;
	
    // Integrate rate of change of quaternion
    gyro[0] *= (0.5f * dt);   // pre-multiply common factors
    gyro[1] *= (0.5f * dt);
    gyro[2] *= (0.5f * dt);
    qa = est->q[0];
    qb = est->q[1];
    qc = est->q[2];
	qd = est->q[3];
    est->q[0] += (-qb * gyro[0] - qc * gyro[1] - qd * gyro[2]);
    est->q[1] += ( qa * gyro[0] + qc * gyro[2] - qd * gyro[1]);
    est->q[2] += ( qa * gyro[1] - qb * gyro[2] + qd * gyro[0]);
    est->q[3] += ( qa * gyro[2] + qb * gyro[1] - qc * gyro[0]);
  
    // Normalise quaternion
    recipNorm = invSqrt(est->q[0] * est->q[0] + est->q[1] * est->q[1] + est->q[2] * est->q[2] + est->q[3] * est->q[3]);
    est->q[0] *= recipNorm;
    est->q[1] *= recipNorm;
    est->q[2] *= recipNorm;
    est->q[3] *= recipNorm;
}

void ahrs_qua2euler(float *q,float att[3])
{
	att[0] = atan2f(2.0f * (q[0] * q[1] + q[2] * q[3]), 1.0f - 2.0f * (q[1] * q[1] + q[2] * q[2]));
	att[1] = asinf(2.0f * (q[0] * q[2] - q[3] * q[1]));
	att[2] = atan2f(2.0f * (q[0] * q[3] + q[1] * q[2]), 1.0f - 2.0f * (q[2] * q[2] + q[3] * q[3]));

	att[0] = degrees(att[0]);
	att[1] = degrees(att[1]);
	att[2] = degrees(att[2]);
}

void ahrs_qua2dcm(float *q,float d[3][3])
{
	float aSq = q[0] * q[0];
	float bSq = q[1] * q[1];
	float cSq = q[2] * q[2];
	float dSq = q[3] * q[3];
	
	d[0][0] = aSq + bSq - cSq - dSq;
	d[0][1] = 2.0f * (q[1] * q[2] - q[0] * q[3]);
	d[0][2] = 2.0f * (q[0] * q[2] + q[1] * q[3]);
	d[1][0] = 2.0f * (q[1] * q[2] + q[0] * q[3]);
	d[1][1] = aSq - bSq + cSq - dSq;
	d[1][2] = 2.0f * (q[2] * q[3] - q[0] * q[1]);
	d[2][0] = 2.0f * (q[1] * q[3] - q[0] * q[2]);
	d[2][1] = 2.0f * (q[0] * q[1] + q[2] * q[3]);
	d[2][2] = aSq - bSq - cSq + dSq;
}

void ahrs_euler2qua(float att[3],float *q)
{
	double att_r[3];
	double cosPhi_2;
	double sinPhi_2;
	double cosTheta_2;
	double sinTheta_2;
	double cosPsi_2;
	double sinPsi_2;

	att_r[0] = radians(att[0]);
	att_r[1] = radians(att[1]);
	att_r[2] = radians(att[2]);

	cosPhi_2 = cos((double)(att_r[0]) / 2.0);
	sinPhi_2 = sin((double)(att_r[0]) / 2.0);
	cosTheta_2 = cos((double)(att_r[1]) / 2.0);
	sinTheta_2 = sin((double)(att_r[1]) / 2.0);
	cosPsi_2 = cos((double)(att_r[2]) / 2.0);
	sinPsi_2 = sin((double)(att_r[2]) / 2.0);

	q[0] = (float)(cosPhi_2 * cosTheta_2 * cosPsi_2 + sinPhi_2 * sinTheta_2 * sinPsi_2);
	q[1] = (float)(sinPhi_2 * cosTheta_2 * cosPsi_2 - cosPhi_2 * sinTheta_2 * sinPsi_2);
	q[2] = (float)(cosPhi_2 * sinTheta_2 * cosPsi_2 + sinPhi_2 * cosTheta_2 * sinPsi_2);
	q[3] = (float)(cosPhi_2 * cosTheta_2 * sinPsi_2 - sinPhi_2 * sinTheta_2 * cosPsi_2);
}

void ahrs_euler2dcm(float att[3],float d[3][3])
{
	float att_r[3];
	float sr;
	float cr;
	float cp;
	float sp;
	float sy;
	float cy;

	att_r[0] = radians(att[0]);
	att_r[1] = radians(att[1]);
	att_r[2] = radians(att[2]);

	sr = sinf(att_r[0]);
	cr = cosf(att_r[0]);
	cp = cosf(att_r[1]);
	sp = sinf(att_r[1]);
	sy = sinf(att_r[2]);
	cy = cosf(att_r[2]);

	d[0][0] = cp * cy;
	d[0][1] = (sr * sp * cy) - (cr * sy);
	d[0][2] = (cr * sp * cy) + (sr * sy);
	d[1][0] = cp * sy;
	d[1][1] = (sr * sp * sy) + (cr * cy);
	d[1][2] = (cr * sp * sy) - (sr * cy);
	d[2][0] = -sp;
	d[2][1] = sr * cp;
	d[2][2] = cr * cp;
}

void ahrs_init_from_euler(ahrs_estimator *est,float att[3])
{
	ahrs_euler2qua(att,est->q);
}

