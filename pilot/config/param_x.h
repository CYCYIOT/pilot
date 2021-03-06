#ifndef _PARAM_X_H_
#define _PARAM_X_H_

#define MODEL_UAV  'X'
#define X_1 1


#define IMU__ROTATION_DEF	6.0f
#define MAG__ROTATION_DEF	0.0f
#define FLOW__ROTATION_DEF	1.0f

#define AHRS__RP_ACTION_CHECK_TIMEOUT_DEF	0.2f
#define AHRS__RP_ACTION_CHECK_DEF			0.4f
#define AHRS__RP_P_ACTION_DEF				0.1f
#define AHRS__RP_I_ACTION_DEF				0.001f
#define AHRS__RP_P_ARM_DEF					0.8f
#define AHRS__RP_I_ARM_DEF					0.01f
#define AHRS__YAW_P_ARM_DEF					0.8f
#define AHRS__YAW_I_ARM_DEF					0.002f
#define AHRS__RP_P_DISARM_DEF				3.0f
#define AHRS__RP_I_DISARM_DEF				0.001f
#define AHRS__YAW_P_DISARM_DEF				0.8f
#define AHRS__YAW_I_DISARM_DEF				0.002f

#define RANGEFINDER__MAX_VAL_DEF			3.5f
#define RANGEFINDER__MIN_VAL_DEF			0.1f
#define RANGEFINDER__ROTATE_CHECK_DEF		0.85f

#define IMU__ACC_XY_LOW_FILTER_DEF			12.0f
#define IMU__ACC_Z_LOW_FILTER_DEF			20.0f
#define IMU__GYRO_SCALE_X_DEF				1.00f
#define IMU__GYRO_SCALE_Y_DEF				1.00f
#define IMU__GYRO_SCALE_Z_DEF				1.00f

#define FLOW__DELAY_DEF						0.03f		//0328-0.02    7740-0.03     3a03-0.03
#define FLOW__SCALE_DEF						0.0015f	    //0328-0.0016  7740-0.0015   3a03-0.0018

#define INAV__FLOW__FUSE_POS_FLIP1_DEF			3.00f
#define INAV__FLOW__FUSE_VEL_FLIP1_DEF			10.00f
#define INAV__FLOW__FUSE_BIAS_FLIP1_DEF			0.00f
#define INAV__FLOW__FUSE_POS_FLIP2_DEF			1.50f
#define INAV__FLOW__FUSE_VEL_FLIP2_DEF			6.00f
#define INAV__FLOW__FUSE_BIAS_FLIP2_DEF			13.00f
#define INAV__FLOW__FUSE_POS_FLIP_STEP_DEF		0.50f
#define INAV__FLOW__FUSE_VEL_FLIP_STEP_DEF		1.00f
#define INAV__FLOW__FUSE_BIAS_FLIP_STEP_DEF		1.00f
#define INAV__FLOW__FUSE_POS_TAKEOFF_DEF		1.00f
#define INAV__FLOW__FUSE_VEL_TAKEOFF_DEF		3.00f
#define INAV__FLOW__FUSE_BIAS_TAKEOFF_DEF		3.00f
#define INAV__FLOW__FUSE_POS_NORMAL_DEF			1.00f
#define INAV__FLOW__FUSE_VEL_NORMAL_DEF			2.50f
#define INAV__FLOW__FUSE_BIAS_NORMAL_DEF		2.00f  //2.00f
#define INAV__FLOW__FUSE_POS_ROTATE_DEF			1.00f
#define INAV__FLOW__FUSE_VEL_ROTATE_DEF			100.00f // 3
#define INAV__FLOW__FUSE_BIAS_ROTATE_DEF		5.00f
#define INAV__FLOW__FUSE_POS_ACTION_DEF			1.00f 
#define INAV__FLOW__FUSE_VEL_ACTION_DEF			4.00f
#define INAV__FLOW__FUSE_BIAS_ACTION_DEF		0.001f
#define INAV__FLOW__QUALITY_WEIGHT_MIN_DEF		0.30f
#define INAV__FLOW__QUALITY_WEIGHT_SCALE_DEF 	1.25f

#define INAV__RF__FUSE_VEL_NORMAL_DEF			0.6f
#define INAV__RF__FUSE_BIAS_NORMAL_DEF			0.12f
#define INAV__RF__FUSE_VEL_FLIP_DEF				4.0f
#define INAV__RF__FUSE_BIAS_FLIP_DEF			0.01f
#define INAV__RF__FUSE_VEL_NOISE_DEF			0.005f
#define INAV__RF__FUSE_BIAS_NOISE_DEF			0.005f
#define INAV__RF__VEL_NOISE_DEF					1.0f

#define INAV__GPS__FUSE_POS_NORMAL_DEF			0.5f
#define INAV__GPS__FUSE_VEL_NORMAL_DEF			0.5f
#define INAV__GPS__FUSE_BIAS_NORMAL_DEF			0.3f

#define INAV__BARO__FUSE_POS_ACTION_DEF			0.50f
#define INAV__BARO__FUSE_VEL_ACTION_DEF			0.30f
#define INAV__BARO__FUSE_BIAS_ACTION_DEF		0.05f  //new_bias = pos^2 * old_bias / 0.05
#define INAV__BARO__FUSE_POS_BREAK_DEF			1.50f
#define INAV__BARO__FUSE_VEL_BREAK_DEF			1.30f
#define INAV__BARO__FUSE_BIAS_BREAK_DEF			0.20f
#define INAV__BARO__FUSE_POS_FLIP_DEF			8.00f
#define INAV__BARO__FUSE_VEL_FLIP_DEF			1.50f
#define INAV__BARO__FUSE_BIAS_FLIP_DEF			0.00f
#define INAV__BARO__FUSE_POS_TAKEOFF_DEF		1.20f
#define INAV__BARO__FUSE_VEL_TAKEOFF_DEF		1.00f
#define INAV__BARO__FUSE_BIAS_TAKEOFF_DEF		5.50f
#define INAV__BARO__FUSE_POS_NORMAL_DEF			0.55f
#define INAV__BARO__FUSE_VEL_NORMAL_DEF			0.55f
#define INAV__BARO__FUSE_BIAS_NORMAL_DEF		0.50f
#define INAV__BARO__FUSE_POS_NOISE_DEF			0.30f
#define INAV__BARO__FUSE_VEL_NOISE_DEF			0.30f
#define INAV__BARO__FUSE_BIAS_NOISE_DEF			0.0001f
#define INAV__BARO__FUSE_POS_STEP_DEF			0.06f
#define INAV__BARO__FUSE_VEL_STEP_DEF			0.06f
#define INAV__BARO__FUSE_BIAS_STEP_DEF			1.80f
#define INAV__BARO__VEL_NOISE_DEF				0.60f
#define INAV__BARO__VEL_CHECK_ACC_FILTER_DEF	2.00f
#define INAV__BARO__VEL_CHECK_VAL_DEF			0.05f
#define INAV__BARO__VEL_CHECK_TIMEOUT_DEF		0.5f

#define CON__COMM__ATT_RP_LIMIT_DEF				5.00f
#define CON__COMM__RATE_RP_LIMIT_DEF			1.50f
#define CON__COMM__RATE_RP_STAB_LIMIT_DEF		0.20f
#define CON__COMM__RATE_YAW_LIMIT_DEF			2.00f
#define CON__COMM__POS_Z_LIMIT_DEF     			20.0f
#define CON__COMM__VEL_Z_LIMIT_DEF				0.5f
#define CON__COMM__VEL_Z_STAB_LIMIT_DEF			0.1f
#define CON__COMM__VEL_XY_LIMIT_DEF				1.0f
#define CON__COMM__ATT_RP_GAIN_DEF				15.0f
#define CON__COMM__RATE_YAW_GAIN_DEF			1.0f
#define CON__COMM__VEL_Z_GAIN_DEF				0.5f
#define CON__COMM__RF_GAIN_DEF					1.0f
#define CON__COMM__RF_VEL_CHECK_DEF				1.2f

#define CON__TAKEOFF__SPIN_TIMEOUT              3.0f
#define CON__TAKEOFF__SPIN_DEF					0.15f
#define CON__TAKEOFF__ALT_DEF					1.0f
#define CON__TAKEOFF__VEL_LIMIT_DEF				0.5f
#define CON__TAKEOFF__RC_CHECK_DEF				0.3f

#define CON__LAND__VEL_DEF						-0.60f
#define CON__LAND__CHECK_ACC_DEF				-20.0f
#define CON__LAND__CHECK_THR_DEF				0.3f
#define CON__LAND__CHECK_THR_TIME_DEF			1.5f

#define CON__ALTHOLD__ATT_RP_LIMIT_DEF          30.0f
#define CON__ALTHOLD__ATT_RP_GAIN_DEF           25.0f
#define CON__ALTHOLD__RATE_RP_LIMIT_DEF         3.0f

#define CON__FLIP__RISE_SPEED_DEF				1.0f
#define CON__FLIP__BURST_ATT_LIMIT_DEF			15.0f
#define CON__FLIP__BURST_RATE_LIMIT_DEF			10.0f
#define CON__FLIP__BURST_ATT_CHECK_DEF			180.0f
#define CON__FLIP__BURST_RATE_CHECK_DEF			18.0f
#define CON__FLIP__BREAK_ATT_DEF				360.0f
#define CON__FLIP__HOLD_ATT_DEF					0.0f
#define CON__FLIP__BREAK_GAIN_DEF				0.16f
#define CON__FLIP__STAB_GAIN_DEF				0.40f
#define CON__FLIP__YAW_GAIN_DEF					0.20f
#define CON__FLIP__HOLD_TIMEOUT_DEF				0.5f
#define CON__FLIP__STABLE_TIMEOUT_DEF			0.2f

#define CON__POSHOLD__AIR_RES_BREAK_ANGLE_DEF 	10.0f
#define CON__POSHOLD__AIR_RES_COFF_DEF	 		0.3f

#define ATT__RATE_RP_LOW_FILTER_DEF				50.0f
#define ATT__RATE_YAW_LOW_FILTER_DEF			30.0f

#define PID__ATT__ROLL_P_DEF 					0.10f  //0.15f
#define PID__ATT__PITCH_P_DEF					0.10f  //0.15f
#define PID__ATT__YAW_P_DEF 					0.04f

#define PID__RATE__ROLL_P_DEF					0.10f
#define PID__RATE__ROLL_I_DEF					0.30f
#define PID__RATE__ROLL_D_DEF					0.003f
#define PID__RATE__ROLL_D_W_DEF 				0.7f
#define PID__RATE__ROLL_I_MAX_DEF				0.2f
#define PID__RATE__ROLL_I_INIT_DEF				0.0f

#define PID__RATE__PITCH_P_DEF					0.10f
#define PID__RATE__PITCH_I_DEF					0.30f
#define PID__RATE__PITCH_D_DEF					0.003f
#define PID__RATE__PITCH_D_W_DEF 				0.7f
#define PID__RATE__PITCH_I_MAX_DEF				0.2f
#define PID__RATE__PITCH_I_INIT_DEF				0.0f

#define PID__RATE__YAW_P_DEF					0.40f
#define PID__RATE__YAW_I_DEF					0.15f
#define PID__RATE__YAW_D_DEF					0.00f
#define PID__RATE__YAW_I_MAX_DEF				1.00f
#define PID__RATE__YAW_I_INIT_DEF				0.01f

#define PID__RATE__FLIP_ROLL_P_DEF			0.15f
#define PID__RATE__FLIP_ROLL_I_DEF			0.00f
#define PID__RATE__FLIP_ROLL_D_DEF			0.004f

#define PID__RATE__FLIP_PITCH_P_DEF			0.15f
#define PID__RATE__FLIP_PITCH_I_DEF			0.00f
#define PID__RATE__FLIP_PITCH_D_DEF			0.004f

#define PID__RATE__FLIP_YAW_P_DEF			0.40f
#define PID__RATE__FLIP_YAW_I_DEF			0.00f
#define PID__RATE__FLIP_YAW_D_DEF			0.005f

#define PID__POS__XY_P_DEF					0.1f
#define PID__POS__Z_P_DEF         			0.8f

#define PID__VEL__XY_P_DEF					0.18f
#define PID__VEL__XY_I_DEF					0.10f
#define PID__VEL__XY_D_DEF					0.0f
#define PID__VEL__XY_D_W_DEF      			0.3f
#define PID__VEL__XY_I_MAX_DEF				0.3f
#define PID__VEL__X_I_INIT_DEF				0.0f
#define PID__VEL__Y_I_INIT_DEF				0.0f

#define PID__VEL__BREAK_XY_P_DEF			0.40f
#define PID__VEL__BREAK_XY_I_DEF			0.08f

#define PID__VEL__Z_P_DEF         			0.3f
#define PID__VEL__Z_I_DEF         			0.2f
#define PID__VEL__Z_D_DEF         			0.01f
#define PID__VEL__Z_D_W_DEF       			0.4f
#define PID__VEL__Z_I_MAX_DEF				0.85f
#define PID__VEL__Z_I_MIN_DEF				0.2f
#define PID__VEL__Z_I_INIT_DEF				0.6f

#define MOTOR__MAP_0_DEF		3.0f
#define MOTOR__MAP_1_DEF		0.0f
#define MOTOR__MAP_2_DEF		1.0f
#define MOTOR__MAP_3_DEF		2.0f
#define MOTOR__MAP_4_DEF		4.0f
#define MOTOR__MAP_5_DEF		5.0f
#define MOTOR__MAP_6_DEF		6.0f
#define MOTOR__MAP_7_DEF		7.0f

#define MOTOR__RP_MAX_DEF		1.50f
#define MOTOR__YAW_MAX_DEF		0.50f
#define MOTOR__THR_MAX_DEF		0.95f
#define MOTOR__OUT_MAX_DEF		1.00f
#define MOTOR__OUT_MIN_DEF		0.05f

#define RC__DEADZONE_ROLL_DEF		0.05f
#define RC__DEADZONE_PITCH_DEF		0.05f
#define RC__DEADZONE_YAW_DEF		0.05f
#define RC__DEADZONE_THR_DEF		0.05f
#define RC__MODE_HIGH_DEF			1.0f
#define RC__MODE_MEDIUM_DEF			0.6f
#define RC__MODE_LOW_DEF			0.3f

#define FS__LOW_BATT_DEF			10.0f  //20.0f
#define FS__RC_TIMEOUT_DEF			0.5f
#define FS__ATT_LIMIT_DEF			60.0f
#define FS__IMU_TEMP_LIMIT_DEF		80.0f

#define LOG__RUN_DEF				1.0f
#define LOG__RATE_FLOW_DEF			25.0f
#define LOG__RATE_SENS_DEF			20.0f
#define LOG__RATE_ALT_DEF			20.0f
#define LOG__RATE_ATT_DEF			20.0f
#define LOG__RATE_AHRS_DEF			20.0f
#define LOG__RATE_PIDRR_DEF			50.0f
#define LOG__RATE_PIDRP_DEF			50.0f
#define LOG__RATE_PIDRY_DEF			50.0f
#define LOG__RATE_PIDVX_DEF			50.0f
#define LOG__RATE_PIDVY_DEF			50.0f
#define LOG__RATE_PIDVZ_DEF			50.0f
#define LOG__RATE_IMU_DEF			20.0f
#define LOG__RATE_BAT_DEF			20.0f
#define LOG__RATE_NAV_DEF			20.0f
#define LOG__RATE_NAV2_DEF			20.0f
#define LOG__RATE_NAV3_DEF			20.0f
#define LOG__RATE_NAV4_DEF			20.0f
#define LOG__RATE_RCI_DEF			20.0f
#define LOG__RATE_RCO_DEF			20.0f
#define LOG__RATE_BARO_DEF			10.0f
#define LOG__RATE_GPS_DEF			0.0f
#define LOG__RATE_SYSTEM_DEF		5.0f
#define LOG__RATE_POSHOLD_DEF		20.0f
#define LOG__RATE_RF_DEF    		20.0f

#endif
