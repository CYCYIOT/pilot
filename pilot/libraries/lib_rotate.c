#include "lib_rotate.h"

void rotate2(float val[2],uint8_t rotation)
{
    float  tmp;
    switch (rotation) {
		case ROTATION_NONE:
	        return;
	    case ROTATION_YAW_90:
	        tmp = val[0]; val[0] = -val[1];val[1] = tmp;
	        return;
		case ROTATION_YAW_135:
	        tmp = val[0];
			val[0] = -val[0] * 0.707f - val[1] * 0.707f;
			val[1] = tmp * 0.707 - val[1] * 0.707f;
	        return;
	    case ROTATION_YAW_180:
	        val[0] = -val[0]; val[1] = -val[1];
	        return;
	    case ROTATION_YAW_270:
	        tmp = val[0]; val[0] = val[1]; val[1] = -tmp;
	        return;
		default:
			return;
    }
}

void rotate3(float val[3],uint8_t rotation)
{
    float  tmp;
    switch (rotation) {
		case ROTATION_NONE:
	        return;
	    case ROTATION_YAW_90: {
	        tmp = val[0]; val[0] = -val[1];val[1] = tmp;
	        return;
	    }
		case ROTATION_YAW_135: {
	        tmp = val[0];
			val[0] = -val[0] * 0.707f - val[1] * 0.707f;
			val[1] = tmp * 0.707 - val[1] * 0.707f;
	        return;
	    }
	    case ROTATION_YAW_180:
	        val[0] = -val[0]; val[1] = -val[1];
	        return;
	    case ROTATION_YAW_270: {
	        tmp = val[0]; val[0] = val[1]; val[1] = -tmp;
	        return;
	    }
	    case ROTATION_ROLL_180: {
	        val[1] = -val[1]; val[2] = -val[2];
	        return;
	    }
	    case ROTATION_ROLL_180_YAW_90: {
	        tmp = val[0]; val[0] = val[1]; val[1] = tmp; val[2] = -val[2];
	        return;
	    }
	    case ROTATION_PITCH_180: {
	        val[0] = -val[0]; val[2] = -val[2];
	        return;
	    }
	    case ROTATION_ROLL_180_YAW_270: {
	        tmp = val[0]; val[0] = -val[1]; val[1] = -tmp; val[2] = -val[2];
	        return;
	    }
	    case ROTATION_ROLL_90: {
	        tmp = val[2]; val[2] = val[1]; val[1] = -tmp;
	        return;
	    }
	    case ROTATION_ROLL_90_YAW_90: {
	        tmp = val[2]; val[2] = val[1]; val[1] = -tmp;
	        tmp = val[0]; val[0] = -val[1]; val[1] = tmp;
	        return;
	    }
	    case ROTATION_ROLL_270: {
	        tmp = val[2]; val[2] = -val[1]; val[1] = tmp;
	        return;
	    }
	    case ROTATION_ROLL_270_YAW_90: {
	        tmp = val[2]; val[2] = -val[1]; val[1] = tmp;
	        tmp = val[0]; val[0] = -val[1]; val[1] = tmp;
	        return;
	    }
	    case ROTATION_PITCH_90: {
	        tmp = val[2]; val[2] = -val[0]; val[0] = tmp;
	        return;
	    }
	    case ROTATION_PITCH_270: {
	        tmp = val[2]; val[2] = val[0]; val[0] = -tmp;
	        return;
	    }
	    case ROTATION_PITCH_180_YAW_90: {
	        val[2] = -val[2];
	        tmp = -val[0]; val[0] = -val[1]; val[1] = tmp;
	        return;
	    }
	    case ROTATION_PITCH_180_YAW_270: {
	        val[0] = -val[0]; val[2] = -val[2];
	        tmp = val[0]; val[0] = val[1]; val[1] = -tmp;
	        return;
	    }
	    case ROTATION_ROLL_90_PITCH_90: {
	        tmp = val[2]; val[2] = val[1]; val[1] = -tmp;
	        tmp = val[2]; val[2] = -val[0]; val[0] = tmp;
	        return;
	    }
	    case ROTATION_ROLL_180_PITCH_90: {
	        val[1] = -val[1]; val[2] = -val[2];
	        tmp = val[2]; val[2] = -val[0]; val[0] = tmp;
	        return;
	    }
	    case ROTATION_ROLL_270_PITCH_90: {
	        tmp = val[2]; val[2] = -val[1]; val[1] = tmp;
	        tmp = val[2]; val[2] = -val[0]; val[0] = tmp;
	        return;
	    }
	    case ROTATION_ROLL_90_PITCH_180: {
	        tmp = val[2]; val[2] = val[1]; val[1] = -tmp;
	        val[0] = -val[0]; val[2] = -val[2];
	        return;
	    }
	    case ROTATION_ROLL_270_PITCH_180: {
	        tmp = val[2]; val[2] = -val[1]; val[1] = tmp;
	        val[0] = -val[0]; val[2] = -val[2];
	        return;
	    }
	    case ROTATION_ROLL_90_PITCH_270: {
	        tmp = val[2]; val[2] = val[1]; val[1] = -tmp;
	        tmp = val[2]; val[2] = val[0]; val[0] = -tmp;
	        return;
	    }
	    case ROTATION_ROLL_180_PITCH_270: {
	        val[1] = -val[1]; val[2] = -val[2];
	        tmp = val[2]; val[2] = val[0]; val[0] = -tmp;
	        return;
	    }
	    case ROTATION_ROLL_270_PITCH_270: {
	        tmp = val[2]; val[2] = -val[1]; val[1] = tmp;
	        tmp = val[2]; val[2] = val[0]; val[0] = -tmp;
	        return;
	    }
	    case ROTATION_ROLL_90_PITCH_180_YAW_90: {
	        tmp = val[2]; val[2] = val[1]; val[1] = -tmp;
	        val[0] = -val[0]; val[2] = -val[2];
	        tmp = val[0]; val[0] = -val[1]; val[1] = tmp;
	        return;
	    }
	    case ROTATION_ROLL_90_YAW_270: {
	        tmp = val[2]; val[2] = val[1]; val[1] = -tmp;
	        tmp = val[0]; val[0] = val[1]; val[1] = -tmp;
	        return;
	    }
    }
}

