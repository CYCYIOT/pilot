#ifndef _LIB_LPF2P_H_
#define _LIB_LPF2P_H_

#include "lib_math.h"

typedef struct{
	float cutoff_freq;
	float a_lp2p[3];
	float b_lp2p[3];
	float delay_element_1;
	float delay_element_2;
} lpf2p_s;


void lpf2p_init(lpf2p_s *filter_param, float sample_freq, float cutoff_freq);
float lpf2p_update(lpf2p_s *filter_param, float rawdata);


#endif


