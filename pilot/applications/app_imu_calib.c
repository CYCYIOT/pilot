#include "app_param_calib.h"
#include "app_debug.h"
#include "app_imu.h"
#include "app_imu_calib.h"

#include "lib_math.h"
#include "geo.h"

#define DEBUG_ID DEBUG_ID_IMU

#define ACC_SAMPLES_NUM 800

#define ACC_CALIB_MODE_LEVEL	0
#define ACC_CALIB_MODE_SIX_AXIS	1

float acc_calib_mode;
uint8_t acc_calib_step;
uint8_t acc_calib_status;
uint16_t acc_samples_num = 0;
uint8_t progress_cnt = 0;
float acc_samples[6][3];

uint8_t imu_calib_acc_get_progress()
{
	uint8_t progress;

	progress = (uint8_t)((float)acc_samples_num/ACC_SAMPLES_NUM  * 100.0f);

	if(acc_samples_num >= ACC_SAMPLES_NUM){
		progress_cnt ++;
		if(progress_cnt == 5){
			progress_cnt = 0;
			acc_samples_num = 0;
		}
	}
	
	return progress;
}

void calibrate_reset_matrices(float dS[6], float JS[6][6])
{
    int16_t j,k;
    for( j=0; j<6; j++ ) {
        dS[j] = 0.0f;
        for( k=0; k<6; k++ ) {
            JS[j][k] = 0.0f;
        }
    }
}

void calibrate_update_matrices(float dS[6], float JS[6][6],float beta[6], float data[3])
{
    int16_t j, k;
    float dx, b;
    float residual = 1.0;
    float jacobian[6];
    
    for( j=0; j<3; j++ ) {
        b = beta[3+j];
        dx = (float)data[j] - beta[j];
        residual -= b*b*dx*dx;
        jacobian[j] = 2.0f*b*b*dx;
        jacobian[3+j] = -2.0f*b*dx*dx;
    }
    
    for( j=0; j<6; j++ ) {
        dS[j] += jacobian[j]*residual;
        for( k=0; k<6; k++ ) {
            JS[j][k] += jacobian[j]*jacobian[k];
        }
    }
}

void calibrate_find_delta(float dS[6], float JS[6][6], float delta[6])
{
    //Solve 6-d matrix equation JS*x = dS
    //first put in upper triangular form
    int16_t i,j,k;
    float mu;

    //make upper triangular
    for( i=0; i<6; i++ ) {
        //eliminate all nonzero entries below JS[i][i]
        for( j=i+1; j<6; j++ ) {
            mu = JS[i][j]/JS[i][i];
            if( mu != 0.0f ) {
                dS[j] -= mu*dS[i];
                for( k=j; k<6; k++ ) {
                    JS[k][j] -= mu*JS[k][i];
                }
            }
        }
    }

    //back-substitute
    for( i=5; i>=0; i-- ) {
        dS[i] /= JS[i][i];
        JS[i][i] = 1.0f;
        
        for( j=0; j<i; j++ ) {
            mu = JS[i][j];
            dS[j] -= mu*dS[i];
            JS[i][j] = 0.0f;
        }
    }

    for( i=0; i<6; i++ ) {
        delta[i] = dS[i];
    }
}


bool calibrate_accel(float accel_sample[6][3],float accel_offsets[3],float accel_scale[3])
{
    int16_t i;
    int16_t num_iterations = 0;
    float eps = 0.000000001;
    float change = 100.0;
    float data[3];
    float beta[6];
    float delta[6];
    float ds[6];
    float JS[6][6];
    bool success = true;

    // reset
    beta[0] = beta[1] = beta[2] = 0;
    beta[3] = beta[4] = beta[5] = 1.0f/CONSTANTS_ONE_G;
    
    while( num_iterations < 20 && change > eps ) {
        num_iterations++;

        calibrate_reset_matrices(ds, JS);

        for( i=0; i<6; i++ ) {
            data[0] = accel_sample[i][0];
            data[1] = accel_sample[i][1];
            data[2] = accel_sample[i][2];
            calibrate_update_matrices(ds, JS, beta, data);
        }

        calibrate_find_delta(ds, JS, delta);

        change =    delta[0]*delta[0] +
                    delta[0]*delta[0] +
                    delta[1]*delta[1] +
                    delta[2]*delta[2] +
                    delta[3]*delta[3] / (beta[3]*beta[3]) +
                    delta[4]*delta[4] / (beta[4]*beta[4]) +
                    delta[5]*delta[5] / (beta[5]*beta[5]);

        for( i=0; i<6; i++ ) {
            beta[i] -= delta[i];
        }
    }

    // copy results out
    accel_scale[0] = beta[3] * CONSTANTS_ONE_G;
    accel_scale[1] = beta[4] * CONSTANTS_ONE_G;
    accel_scale[2] = beta[5] * CONSTANTS_ONE_G;
    accel_offsets[0] = beta[0] * accel_scale[0];
    accel_offsets[1] = beta[1] * accel_scale[1];
    accel_offsets[2] = beta[2] * accel_scale[2];

    // sanity check scale
    if(isnan(accel_scale[0]) || isnan(accel_scale[1]) || isnan(accel_scale[2])) {
        success = false;
    }
    if(fabsf(accel_scale[0]-1.0f) > 0.1f || fabsf(accel_scale[1]-1.0f) > 0.1f || fabsf(accel_scale[2]-1.0f) > 0.1f ) {
        success = false;
    }
	
    // sanity check offsets (3.5 is roughly 3/10th of a G, 5.0 is roughly half a G)
    if(isnan(accel_offsets[0]) || isnan(accel_offsets[1]) || isnan(accel_offsets[2])) {
        success = false;
    }
    if(fabsf(accel_offsets[0]) > 3.5f || fabsf(accel_offsets[1]) > 3.5f || fabsf(accel_offsets[2]) > 3.5f ) {
        success = false;
    }

    // return success or failure
    return success;
}

void imu_calib_acc_start(void)
{
	acc_calib_status = IMU_CALIB_ACC_STATUS_START;
	acc_calib_step = IMU_CALIB_ACC_STEP_LEVEL;
	
	acc_samples[IMU_CALIB_ACC_STEP_LEVEL][0] = 0.0f;
	acc_samples[IMU_CALIB_ACC_STEP_LEVEL][1] = 0.0f;
	acc_samples[IMU_CALIB_ACC_STEP_LEVEL][2] = 0.0f;
	acc_samples_num = 0;
	
	INFO(DEBUG_ID,"imu acc calibration started");
}

void imu_calib_acc_confirm_step(void)
{
	if(acc_calib_mode == ACC_CALIB_MODE_SIX_AXIS){
		acc_samples_num = 1;
	}
}

uint8_t imu_calib_acc_get_step(void)
{
	return acc_calib_step;
}

void imu_calib_acc_set_status_null(void)
{
	acc_calib_status = IMU_CALIB_ACC_STATUS_NULL;
}

uint8_t imu_calib_acc_get_status(void)
{
	return acc_calib_status;
}

void imu_calib_acc_mode_level_update(void)
{
	if(acc_samples_num >= ACC_SAMPLES_NUM){
		float accel_offset[3];
		float accel_scale[3];
		
		acc_samples[IMU_CALIB_ACC_STEP_LEVEL][0] /= acc_samples_num;
		acc_samples[IMU_CALIB_ACC_STEP_LEVEL][1] /= acc_samples_num;
		acc_samples[IMU_CALIB_ACC_STEP_LEVEL][2] /= acc_samples_num;

		accel_offset[0] = acc_samples[IMU_CALIB_ACC_STEP_LEVEL][0];
		accel_offset[1] = acc_samples[IMU_CALIB_ACC_STEP_LEVEL][1];
		accel_offset[2] = acc_samples[IMU_CALIB_ACC_STEP_LEVEL][2] + CONSTANTS_ONE_G;
		
		v3f_set_val(accel_scale,1.0f);

		INFO(DEBUG_ID,"acc calib:%3.3f,%3.3f,%3.3f",acc_samples[IMU_CALIB_ACC_STEP_LEVEL][0],acc_samples[IMU_CALIB_ACC_STEP_LEVEL][1],acc_samples[IMU_CALIB_ACC_STEP_LEVEL][2]);
		INFO(DEBUG_ID,"acc calib done:%3.3f,%3.3f,%3.3f",accel_offset[0],accel_offset[1],accel_offset[2]);
		if(fabs(accel_offset[0]) > 0.5f || fabs(accel_offset[1]) > 0.5f || fabs(accel_offset[2]) > 0.5f){
			ERR(DEBUG_ID,"acc calib value too large");
		}
		
		imu_set_acc_cal(accel_offset,accel_scale);
		acc_calib_status = IMU_CALIB_ACC_STATUS_EXIST;
	}else{
		float acc[3];
		float gyro[3];

		imu_get_acc_raw(acc);
		imu_get_gyro(gyro);
		
		if(pythagorous_v3f(gyro) <= 0.1f){
			acc_samples[IMU_CALIB_ACC_STEP_LEVEL][0] += acc[0];
			acc_samples[IMU_CALIB_ACC_STEP_LEVEL][1] += acc[1];
			acc_samples[IMU_CALIB_ACC_STEP_LEVEL][2] += acc[2];
			acc_samples_num++;
		}
	}

}

void imu_calib_acc_mode_six_axis_update(void)
{
	float acc[3];

	imu_get_acc_raw(acc);
	acc_samples[acc_calib_step][0] += acc[0];
	acc_samples[acc_calib_step][1] += acc[1];
	acc_samples[acc_calib_step][2] += acc[2];
	//printf("update %d %d (%3.3f %3.3f %3.3f)\r\n",acc_samples_num,acc_calib_status,acc[0],acc[1],acc[2]);

	if(acc_samples_num >= ACC_SAMPLES_NUM){
		acc_samples[acc_calib_step][0] /= acc_samples_num;
		acc_samples[acc_calib_step][1] /= acc_samples_num;
		acc_samples[acc_calib_step][2] /= acc_samples_num;
		INFO(DEBUG_ID,"%d %3.3f,%3.3f,%3.3f",acc_calib_step,acc_samples[acc_calib_step][0],acc_samples[acc_calib_step][1],acc_samples[acc_calib_step][2]);
		acc_calib_step++;
		acc_samples_num = 0;
	}else{
		acc_samples_num++;
	}

	if(acc_calib_step == IMU_CALIB_ACC_STEP_DONE){
		float new_offsets[3];
		float new_scaling[3];
		float accel_offset[3];
		float accel_scale[3];
		bool success = calibrate_accel(acc_samples,new_offsets,new_scaling);
		
		INFO(DEBUG_ID,"Offsets:%.2f %.2f %.2f",new_offsets[0],new_offsets[1],new_offsets[2]);
		INFO(DEBUG_ID,"Scaling:%.2f %.2f %.2f",new_scaling[0],new_scaling[1],new_scaling[2]);
		if(success){
			INFO(DEBUG_ID,"ACC Calibration successful");
			
			v3f_set(accel_offset,new_offsets);
			v3f_set(accel_scale,new_scaling);
		
			imu_set_acc_cal(accel_offset,accel_scale);
			
			acc_calib_status = IMU_CALIB_ACC_STATUS_EXIST;
		}else{
			acc_calib_status = IMU_CALIB_ACC_STATUS_FAILED;
		}
	}
}

void imu_calib_acc_update(void)
{
	if(acc_calib_status != IMU_CALIB_ACC_STATUS_START){
		return;
	}

	if(acc_calib_mode == ACC_CALIB_MODE_LEVEL){
		imu_calib_acc_mode_level_update();
	}else{
		imu_calib_acc_mode_six_axis_update();
	}	
}

void imu_calib_acc_param_init()
{
	param_calib_define(ACC_CALIB_MODE_NAME,ACC_CALIB_MODE_DEF,&acc_calib_mode);
}

void imu_calib_acc_init(void)
{
	acc_calib_status = IMU_CALIB_ACC_STATUS_NULL;
	acc_calib_step = IMU_CALIB_ACC_STEP_LEVEL;
	acc_samples_num = 0;
}

