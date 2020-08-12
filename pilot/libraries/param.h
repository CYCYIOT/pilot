#ifndef _PARAM_H_
#define _PARAM_H_

#ifdef K
#include "param_k.h"
#elif  EVAB2
#include "param_evab2.h"
#elif M
#include "param_m.h"
#elif X
#include "param_x.h"
#else
#error "no arch"
#endif

#define MAIN_LOOP_HZ    500
#define SCHED_DEFAULT	SCHED_FIFO

#define LOG_TF_CHECK	1
#define LOG_PATH		LOG_PATH_TF
#define LOG_PATH_TF		"/mnt/log.bin"
#define LOG_PATH_TMP	"/tmp/log.bin"

//------------------------------------------------------------------------

#define MAG_OFFSET_X_DEF    0.0f
#define MAG_OFFSET_Y_DEF    0.0f
#define MAG_OFFSET_Z_DEF    0.0f
#define MAG_RADIUS_DEF      468.0f
#define MAG_CALIB_STATUS	0.0f

#define ACC_CALIB_STATUS	0.0f
#define ACC_CALIB_MODE_DEF	0.0f
#define ACC_OFFSET_X_DEF	0.0f 
#define ACC_OFFSET_Y_DEF    0.0f
#define ACC_OFFSET_Z_DEF    0.0f
#define ACC_SCALE_X_DEF		1.0f
#define ACC_SCALE_Y_DEF		1.0f
#define ACC_SCALE_Z_DEF		1.0f

#define MAG_OFFSET_X_NAME       			"MAG_OFFSET_X"
#define MAG_OFFSET_Y_NAME					"MAG_OFFSET_Y"
#define MAG_OFFSET_Z_NAME					"MAG_OFFSET_Z"
#define MAG_RADIUS_NAME						"MAG_RADIUS"
#define MAG_CALIB_STATUS_NAME				"MAG_CALIB_STATUS" 


#define ACC_CALIB_MODE_NAME					"ACC_CALIB_MODE"
#define ACC_OFFSET_X_NAME					"ACC_OFFSET_X"
#define ACC_OFFSET_Y_NAME					"ACC_OFFSET_Y"
#define ACC_OFFSET_Z_NAME					"ACC_OFFSET_Z"
#define ACC_SCALE_X_NAME					"ACC_SCALE_X"
#define ACC_SCALE_Y_NAME					"ACC_SCALE_Y"
#define ACC_SCALE_Z_NAME					"ACC_SCALE_Z"
#define ACC_CALIB_STATUS_NAME				"ACC_CALIB_STATUS" 

//----------------------------------------------------------------------

#define MAG__ROTATION_NAME						"MAG__ROTATION"

#define AHRS__RP_ACTION_CHECK_TIMEOUT_NAME		"AHRS__RP_ACTION_CHECK_TIMEOUT"
#define AHRS__RP_ACTION_CHECK_NAME				"AHRS__RP_ACTION_CHECK"
#define AHRS__RP_P_ACTION_NAME					"AHRS__RP_P_ACTION"
#define AHRS__RP_I_ACTION_NAME					"AHRS__RP_I_ACTION"
#define AHRS__RP_P_ARM_NAME						"AHRS__RP_P_ARM"
#define AHRS__RP_I_ARM_NAME						"AHRS__RP_I_ARM"
#define AHRS__YAW_P_ARM_NAME					"AHRS__YAW_P_ARM"
#define AHRS__YAW_I_ARM_NAME					"AHRS__YAW_I_ARM"
#define AHRS__RP_P_DISARM_NAME					"AHRS__RP_P_DISARM"
#define AHRS__RP_I_DISARM_NAME					"AHRS__RP_I_DISARM"
#define AHRS__YAW_P_DISARM_NAME					"AHRS__YAW_P_DISARM"
#define AHRS__YAW_I_DISARM_NAME					"AHRS__YAW_I_DISARM"

#define RANGEFINDER__MAX_VAL_NAME				"RANGEFINDER__MAX_VAL"
#define RANGEFINDER__MIN_VAL_NAME				"RANGEFINDER__MIN_VAL"
#define RANGEFINDER__ROTATE_CHECK_NAME			"RANGEFINDER__ROTATE_CHECK"

#define IMU__ROTATION_NAME						"IMU__ROTATION"
#define IMU__ACC_XY_LOW_FILTER_NAME				"IMU__ACC_XY_LOW_FILTER"
#define IMU__ACC_Z_LOW_FILTER_NAME				"IMU__ACC_Z_LOW_FILTER"
#define IMU__GYRO_SCALE_X_NAME					"IMU__GYRO_SCALE_X"
#define IMU__GYRO_SCALE_Y_NAME					"IMU__GYRO_SCALE_Y"
#define IMU__GYRO_SCALE_Z_NAME					"IMU__GYRO_SCALE_Z"

#define FLOW__ROTATION_NAME						"FLOW__ROTATION"
#define FLOW__DELAY_NAME						"FLOW__DELAY"
#define FLOW__SCALE_NAME						"FLOW__SCALE"

#define INAV__FLOW__FUSE_POS_FLIP1_NAME			"INAV__FLOW__FUSE_POS_FLIP1"
#define INAV__FLOW__FUSE_VEL_FLIP1_NAME			"INAV__FLOW__FUSE_VEL_FLIP1"
#define INAV__FLOW__FUSE_BIAS_FLIP1_NAME		"INAV__FLOW__FUSE_BIAS_FLIP1"
#define INAV__FLOW__FUSE_POS_FLIP2_NAME			"INAV__FLOW__FUSE_POS_FLIP2"
#define INAV__FLOW__FUSE_VEL_FLIP2_NAME			"INAV__FLOW__FUSE_VEL_FLIP2"
#define INAV__FLOW__FUSE_BIAS_FLIP2_NAME		"INAV__FLOW__FUSE_BIAS_FLIP2"
#define INAV__FLOW__FUSE_POS_FLIP_STEP_NAME		"INAV__FLOW__FUSE_POS_FLIP_STEP"
#define INAV__FLOW__FUSE_VEL_FLIP_STEP_NAME		"INAV__FLOW__FUSE_VEL_FLIP_STEP"
#define INAV__FLOW__FUSE_BIAS_FLIP_STEP_NAME	"INAV__FLOW__FUSE_BIAS_FLIP_STEP"
#define INAV__FLOW__FUSE_POS_TAKEOFF_NAME		"INAV__FLOW__FUSE_POS_TAKEOFF"
#define INAV__FLOW__FUSE_VEL_TAKEOFF_NAME		"INAV__FLOW__FUSE_VEL_TAKEOFF"
#define INAV__FLOW__FUSE_BIAS_TAKEOFF_NAME		"INAV__FLOW__FUSE_BIAS_TAKEOFF"
#define INAV__FLOW__FUSE_POS_NORMAL_NAME		"INAV__FLOW__FUSE_POS_NORMAL"
#define INAV__FLOW__FUSE_VEL_NORMAL_NAME		"INAV__FLOW__FUSE_VEL_NORMAL"
#define INAV__FLOW__FUSE_BIAS_NORMAL_NAME		"INAV__FLOW__FUSE_BIAS_NORMAL"
#define INAV__FLOW__FUSE_POS_ROTATE_NAME		"INAV__FLOW__FUSE_POS_ROTATE"
#define INAV__FLOW__FUSE_VEL_ROTATE_NAME		"INAV__FLOW__FUSE_VEL_ROTATE"
#define INAV__FLOW__FUSE_BIAS_ROTATE_NAME		"INAV__FLOW__FUSE_BIAS_ROTATE"
#define INAV__FLOW__FUSE_POS_ACTION_NAME		"INAV__FLOW__FUSE_POS_ACTION"
#define INAV__FLOW__FUSE_VEL_ACTION_NAME		"INAV__FLOW__FUSE_VEL_ACTION"
#define INAV__FLOW__FUSE_BIAS_ACTION_NAME		"INAV__FLOW__FUSE_BIAS_ACTION"
#define INAV__FLOW__QUALITY_WEIGHT_MIN_NAME 	"INAV__FLOW__QUALITY_WEIGHT_MIN"
#define INAV__FLOW__QUALITY_WEIGHT_SCALE_NAME 	"INAV__FLOW__QUALITY_WEIGHT_SCALE"

#define INAV__RF__FUSE_VEL_NORMAL_NAME			"INAV__RF__FUSE_VEL_NORMAL"
#define INAV__RF__FUSE_BIAS_NORMAL_NAME			"INAV__RF__FUSE_BIAS_NORMAL"
#define INAV__RF__FUSE_VEL_FLIP_NAME			"INAV__RF__FUSE_VEL_FLIP"
#define INAV__RF__FUSE_BIAS_FLIP_NAME			"INAV__RF__FUSE_BIAS_FLIP"
#define INAV__RF__FUSE_VEL_NOISE_NAME			"INAV__RF__FUSE_VEL_NOISE"
#define INAV__RF__FUSE_BIAS_NOISE_NAME			"INAV__RF__FUSE_BIAS_NOISE"
#define INAV__RF__VEL_NOISE_NAME				"INAV__RF__VEL_NOISE"

#define INAV__GPS__FUSE_POS_NORMAL_NAME			"INAV__GPS__FUSE_POS_NORMAL"
#define INAV__GPS__FUSE_VEL_NORMAL_NAME			"INAV__GPS__FUSE_VEL_NORMAL"
#define INAV__GPS__FUSE_BIAS_NORMAL_NAME		"INAV__GPS__FUSE_BIAS_NORMAL"

#define INAV__BARO__FUSE_POS_ACTION_NAME		"INAV__BARO__FUSE_POS_ACTION"
#define INAV__BARO__FUSE_VEL_ACTION_NAME		"INAV__BARO__FUSE_VEL_ACTION"
#define INAV__BARO__FUSE_BIAS_ACTION_NAME		"INAV__BARO__FUSE_BIAS_ACTION"
#define INAV__BARO__FUSE_POS_BREAK_NAME			"INAV__BARO__FUSE_POS_BREAK"
#define INAV__BARO__FUSE_VEL_BREAK_NAME			"INAV__BARO__FUSE_VEL_BREAK"
#define INAV__BARO__FUSE_BIAS_BREAK_NAME		"INAV__BARO__FUSE_BIAS_BREAK"
#define INAV__BARO__FUSE_POS_FLIP_NAME			"INAV__BARO__FUSE_POS_FLIP"
#define INAV__BARO__FUSE_VEL_FLIP_NAME			"INAV__BARO__FUSE_VEL_FLIP"
#define INAV__BARO__FUSE_BIAS_FLIP_NAME			"INAV__BARO__FUSE_BIAS_FLIP"
#define INAV__BARO__FUSE_POS_TAKEOFF_NAME		"INAV__BARO__FUSE_POS_TAKEOFF"
#define INAV__BARO__FUSE_VEL_TAKEOFF_NAME		"INAV__BARO__FUSE_VEL_TAKEOFF"
#define INAV__BARO__FUSE_BIAS_TAKEOFF_NAME		"INAV__BARO__FUSE_BIAS_TAKEOFF"
#define INAV__BARO__FUSE_POS_NORMAL_NAME		"INAV__BARO__FUSE_POS_NORMAL"
#define INAV__BARO__FUSE_VEL_NORMAL_NAME		"INAV__BARO__FUSE_VEL_NORMAL"
#define INAV__BARO__FUSE_BIAS_NORMAL_NAME		"INAV__BARO__FUSE_BIAS_NORMAL"
#define INAV__BARO__FUSE_POS_NOISE_NAME			"INAV__BARO__FUSE_POS_NOISE"
#define INAV__BARO__FUSE_VEL_NOISE_NAME			"INAV__BARO__FUSE_VEL_NOISE"
#define INAV__BARO__FUSE_BIAS_NOISE_NAME		"INAV__BARO__FUSE_BIAS_NOISE"
#define INAV__BARO__FUSE_POS_STEP_NAME			"INAV__BARO__FUSE_POS_STEP"
#define INAV__BARO__FUSE_VEL_STEP_NAME			"INAV__BARO__FUSE_VEL_STEP"
#define INAV__BARO__FUSE_BIAS_STEP_NAME			"INAV__BARO__FUSE_BIAS_STEP"
#define INAV__BARO__VEL_NOISE_NAME				"INAV__BARO__VEL_NOISE"
#define INAV__BARO__VEL_CHECK_ACC_FILTER_NAME	"INAV__BARO__VEL_CHECK_ACC_FILTER"
#define INAV__BARO__VEL_CHECK_VAL_NAME			"INAV__BARO__VEL_CHECK_VAL"
#define INAV__BARO__VEL_CHECK_TIMEOUT_NAME		"INAV__BARO__VEL_CHECK_TIMEOUT"

#define CON__COMM__ATT_RP_LIMIT_NAME			"CON__COMM__ATT_RP_LIMIT"
#define CON__COMM__RATE_RP_LIMIT_NAME			"CON__COMM__RATE_RP_LIMIT"
#define CON__COMM__RATE_RP_STAB_LIMIT_NAME		"CON__COMM__RATE_RP_STAB_LIMIT"
#define CON__COMM__RATE_YAW_LIMIT_NAME			"CON__COMM__RATE_YAW_LIMIT"
#define CON__COMM__VEL_XY_LIMIT_NAME			"CON__COMM__VEL_XY_LIMIT"
#define CON__COMM__VEL_Z_LIMIT_NAME				"CON__COMM__VEL_Z_LIMIT"
#define CON__COMM__VEL_Z_STAB_LIMIT_NAME		"CON__COMM__VEL_Z_STAB_LIMIT"
#define CON__COMM__POS_Z_LIMIT_NAME				"CON__COMM__POS_Z_LIMIT"
#define CON__COMM__ATT_RP_GAIN_NAME				"CON__COMM__ATT_RP_GAIN"
#define CON__COMM__RATE_YAW_GAIN_NAME			"CON__COMM__RATE_YAW_GAIN"
#define CON__COMM__VEL_Z_GAIN_NAME				"CON__COMM__VEL_Z_GAIN"
#define CON__COMM__RF_GAIN_NAME					"CON__COMM__RF_GAIN"
#define CON__COMM__RF_VEL_CHECK_NAME			"CON__COMM__RF_VEL_CHECK"

#define CON__TAKEOFF__SPIN_NAME					"CON__TAKEOFF__SPIN"
#define CON__TAKEOFF__ALT_NAME					"CON__TAKEOFF__ALT"
#define CON__TAKEOFF__VEL_LIMIT_NAME			"CON__TAKEOFF__VEL_LIMIT"
#define CON__TAKEOFF__RC_CHECK_NAME				"CON__TAKEOFF__RC_CHECK"

#define CON__LAND__VEL_NAME						"CON__LAND__VEL"
#define CON__LAND__CHECK_ACC_NAME				"CON__LAND__CHECK_ACC"
#define CON__LAND__CHECK_THR_NAME				"CON__LAND__CHECK_THR"
#define CON__LAND__CHECK_THR_TIME_NAME			"CON__LAND__CHECK_THR_TIME"

#define CON__ALTHOLD__ATT_RP_LIMIT_NAME			"CON__ALTHOLD__ATT_RP_LIMIT"
#define CON__ALTHOLD__ATT_RP_GAIN_NAME			"CON__ALTHOLD__ATT_RP_GAIN"
#define CON__ALTHOLD__RATE_RP_LIMIT_NAME		"CON__ALTHOLD__RATE_RP_LIMIT"

#define CON__FLIP__RISE_SPEED_NAME				"CON__FLIP__RISE_SPEED"
#define CON__FLIP__BURST_ATT_LIMIT_NAME			"CON__FLIP__BURST_ATT_LIMIT"
#define CON__FLIP__BURST_RATE_LIMIT_NAME		"CON__FLIP__BURST_RATE_LIMIT"
#define CON__FLIP__BURST_ATT_CHECK_NAME			"CON__FLIP__BURST_ATT_CHECK"
#define CON__FLIP__BURST_RATE_CHECK_NAME		"CON__FLIP__BURST_RATE_CHECK"
#define CON__FLIP__BREAK_ATT_NAME				"CON__FLIP__BREAK_ATT"
#define CON__FLIP__HOLD_ATT_NAME				"CON__FLIP__HOLD_ATT"
#define CON__FLIP__BREAK_GAIN_NAME				"CON__FLIP__BREAK_GAIN"
#define CON__FLIP__STAB_GAIN_NAME				"CON__FLIP__STAB_GAIN"
#define CON__FLIP__YAW_GAIN_NAME				"CON__FLIP__YAW_GAIN"
#define CON__FLIP__HOLD_TIMEOUT_NAME			"CON__FLIP__HOLD_TIMEOUT"
#define CON__FLIP__STABLE_TIMEOUT_NAME			"CON__FLIP__STABLE_TIMEOUT"

#define CON__POSHOLD__AIR_RES_BREAK_ANGLE_NAME	"CON__POSHOLD__AIR_RES_BREAK_ANGLE"
#define CON__POSHOLD__AIR_RES_COFF_NAME			"CON__POSHOLD__AIR_RES_COFF"

#define ATT__RATE_RP_LOW_FILTER_NAME			"ATT__RATE_RP_LOW_FILTER"
#define ATT__RATE_YAW_LOW_FILTER_NAME			"ATT__RATE_YAW_LOW_FILTER"

#define PID__ATT__ROLL_P_NAME					"PID__ATT__ROLL_P"
#define PID__ATT__PITCH_P_NAME					"PID__ATT__PITCH_P"
#define PID__ATT__YAW_P_NAME					"PID__ATT__YAW_P"

#define PID__RATE__ROLL_P_NAME					"PID__RATE__ROLL_P"
#define PID__RATE__ROLL_I_NAME					"PID__RATE__ROLL_I"
#define PID__RATE__ROLL_D_NAME					"PID__RATE__ROLL_D"
#define PID__RATE__ROLL_D_W_NAME				"PID__RATE__ROLL_D_W"
#define PID__RATE__ROLL_I_MAX_NAME				"PID__RATE__ROLL_I_MAX"
#define PID__RATE__ROLL_I_INIT_NAME				"PID__RATE__ROLL_I_INIT"

#define PID__RATE__PITCH_P_NAME					"PID__RATE__PITCH_P"
#define PID__RATE__PITCH_I_NAME					"PID__RATE__PITCH_I"
#define PID__RATE__PITCH_D_NAME					"PID__RATE__PITCH_D"
#define PID__RATE__PITCH_D_W_NAME				"PID__RATE__PITCH_D_W"
#define PID__RATE__PITCH_I_MAX_NAME				"PID__RATE__PITCH_I_MAX"
#define PID__RATE__PITCH_I_INIT_NAME			"PID__RATE__PITCH_I_INIT"

#define PID__RATE__YAW_P_NAME					"PID__RATE__YAW_P"
#define PID__RATE__YAW_I_NAME					"PID__RATE__YAW_I"
#define PID__RATE__YAW_D_NAME					"PID__RATE__YAW_D"
#define PID__RATE__YAW_I_MAX_NAME				"PID__RATE__YAW_I_MAX"
#define PID__RATE__YAW_I_INIT_NAME				"PID__RATE__YAW_I_INIT"

#define PID__RATE__FLIP_ROLL_P_NAME				"PID__RATE__FLIP_ROLL_P"
#define PID__RATE__FLIP_ROLL_I_NAME				"PID__RATE__FLIP_ROLL_I"
#define PID__RATE__FLIP_ROLL_D_NAME				"PID__RATE__FLIP_ROLL_D"

#define PID__RATE__FLIP_PITCH_P_NAME			"PID__RATE__FLIP_PITCH_P"
#define PID__RATE__FLIP_PITCH_I_NAME			"PID__RATE__FLIP_PITCH_I"
#define PID__RATE__FLIP_PITCH_D_NAME			"PID__RATE__FLIP_PITCH_D"

#define PID__RATE__FLIP_YAW_P_NAME				"PID__RATE__FLIP_YAW_P"
#define PID__RATE__FLIP_YAW_I_NAME				"PID__RATE__FLIP_YAW_I"
#define PID__RATE__FLIP_YAW_D_NAME				"PID__RATE__FLIP_YAW_D"

#define PID__POS__XY_P_NAME						"PID__POS__XY_P"
#define PID__POS__Z_P_NAME						"PID__POS__Z_P"

#define PID__VEL__XY_P_NAME						"PID__VEL__XY_P"
#define PID__VEL__XY_I_NAME						"PID__VEL__XY_I"
#define PID__VEL__XY_D_NAME						"PID__VEL__XY_D"
#define PID__VEL__XY_D_W_NAME					"PID__VEL__XY_D_W"
#define PID__VEL__XY_I_MAX_NAME					"PID__VEL__XY_I_MAX"
#define PID__VEL__X_I_INIT_NAME					"PID__VEL__X_I_INIT"
#define PID__VEL__Y_I_INIT_NAME					"PID__VEL__Y_I_INIT"

#define PID__VEL__BREAK_XY_P_NAME				"PID__VEL__BREAK_XY_P"
#define PID__VEL__BREAK_XY_I_NAME				"PID__VEL__BREAK_XY_I"

#define PID__VEL__Z_P_NAME						"PID__VEL__Z_P"
#define PID__VEL__Z_I_NAME						"PID__VEL__Z_I"
#define PID__VEL__Z_D_NAME						"PID__VEL__Z_D"
#define PID__VEL__Z_D_W_NAME					"PID__VEL__Z_D_W"
#define PID__VEL__Z_I_MAX_NAME					"PID__VEL__Z_I_MAX"
#define PID__VEL__Z_I_MIN_NAME					"PID__VEL__Z_I_MIN"
#define PID__VEL__Z_I_INIT_NAME					"PID__VEL__Z_I_INIT"

#define MOTOR__MAP_0_NAME						"MOTOR__MAP_0"
#define MOTOR__MAP_1_NAME						"MOTOR__MAP_1"
#define MOTOR__MAP_2_NAME						"MOTOR__MAP_2"
#define MOTOR__MAP_3_NAME						"MOTOR__MAP_3"
#define MOTOR__MAP_4_NAME						"MOTOR__MAP_4"
#define MOTOR__MAP_5_NAME						"MOTOR__MAP_5"
#define MOTOR__MAP_6_NAME						"MOTOR__MAP_6"
#define MOTOR__MAP_7_NAME						"MOTOR__MAP_7"

#define MOTOR__RP_MAX_NAME						"MOTOR__RP_MAX"
#define MOTOR__YAW_MAX_NAME						"MOTOR__YAW_MAX"
#define MOTOR__THR_MAX_NAME						"MOTOR__THR_MAX"
#define MOTOR__OUT_MAX_NAME						"MOTOR__OUT_MAX"
#define MOTOR__OUT_MIN_NAME						"MOTOR__OUT_MIN"

#define RC__DEADZONE_ROLL_NAME					"RC__DEADZONE_ROLL"
#define RC__DEADZONE_PITCH_NAME					"RC__DEADZONE_PITCH"
#define RC__DEADZONE_YAW_NAME					"RC__DEADZONE_YAW"
#define RC__DEADZONE_THR_NAME					"RC__DEADZONE_THR"
#define RC__MODE_HIGH_NAME						"RC__MODE_HIGH"
#define RC__MODE_MEDIUM_NAME					"RC__MODE_MEDIUM"
#define RC__MODE_LOW_NAME						"RC__MODE_LOW"

#define FS__RC_TIMEOUT_NAME						"FS__RC_TIMEOUT"
#define FS__LOW_BATT_NAME						"FS__LOW_BATT"
#define FS__ATT_LIMIT_NAME						"FS__ATT_LIMIT"
#define FS__IMU_TEMP_LIMIT_NAME					"FS__IMU_TEMP_LIMIT"

#define LOG__RUN_NAME							"LOG__RUN"
#define LOG__RATE_FLOW_NAME						"LOG__RATE_FLOW"
#define LOG__RATE_SENS_NAME						"LOG__RATE_SENS"
#define LOG__RATE_ALT_NAME						"LOG__RATE_ALT"
#define LOG__RATE_ATT_NAME						"LOG__RATE_ATT"
#define LOG__RATE_AHRS_NAME						"LOG__RATE_AHRS"
#define LOG__RATE_PIDRR_NAME					"LOG__RATE_PIDRR"
#define LOG__RATE_PIDRP_NAME					"LOG__RATE_PIDRP"
#define LOG__RATE_PIDRY_NAME					"LOG__RATE_PIDRY"
#define LOG__RATE_PIDVX_NAME					"LOG__RATE_PIDVX"
#define LOG__RATE_PIDVY_NAME					"LOG__RATE_PIDVY"
#define LOG__RATE_PIDVZ_NAME					"LOG__RATE_PIDVZ"
#define LOG__RATE_IMU_NAME						"LOG__RATE_IMU"
#define LOG__RATE_BAT_NAME						"LOG__RATE_BAT"
#define LOG__RATE_NAV_NAME						"LOG__RATE_NAV"
#define LOG__RATE_NAV2_NAME						"LOG__RATE_NAV2"
#define LOG__RATE_NAV3_NAME						"LOG__RATE_NAV3"
#define LOG__RATE_NAV4_NAME						"LOG__RATE_NAV4"
#define LOG__RATE_RCI_NAME						"LOG__RATE_RCI"
#define LOG__RATE_RCO_NAME						"LOG__RATE_RCO"
#define LOG__RATE_BARO_NAME						"LOG__RATE_BARO"
#define LOG__RATE_GPS_NAME						"LOG__RATE_GPS"
#define LOG__RATE_SYSTEM_NAME					"LOG__RATE_SYSTEM"
#define LOG__RATE_POSHOLD_NAME					"LOG__RATE_POSHOLD"
#define LOG__RATE_RF_NAME						"LOG__RATE_RF"

#endif
