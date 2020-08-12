#include "lib_lpf2p.h"

void lpf2p_init(lpf2p_s *filter_param, float sample_freq, float cutoff_freq)
{
	if(filter_param->cutoff_freq != cutoff_freq){
		filter_param->cutoff_freq = cutoff_freq;
	}else{
		return;
	}

	if (filter_param->cutoff_freq <= 0.0f) {
		return;
	}
	
	float fr = sample_freq/cutoff_freq;
	float ohm = tanf(M_PI_F/fr);
	float c = 1.0f+2.0f*cosf(M_PI_F/4.0f)*ohm + ohm*ohm;
	
	filter_param->b_lp2p[0] = ohm*ohm/c;
	filter_param->b_lp2p[1] = 2.0f*filter_param->b_lp2p[0];
	filter_param->b_lp2p[2] = filter_param->b_lp2p[0];
	
	filter_param->a_lp2p[0] = 0;
	filter_param->a_lp2p[1] = 2.0f*(ohm*ohm-1.0f)/c;
	filter_param->a_lp2p[2] = (1.0f-2.0f*cosf(M_PI_F/4.0f)*ohm+ohm*ohm)/c;

	filter_param->delay_element_1 = 0.0f;
	filter_param->delay_element_2 = 0.0f;
}

float lpf2p_update(lpf2p_s *filter_param, float rawdata)
{
	float delay_element_0;
	float output;

	if (filter_param->cutoff_freq <= 0.0f) {
		return rawdata;
	}

	delay_element_0 = rawdata - filter_param->delay_element_1 * filter_param->a_lp2p[1] - filter_param->delay_element_2 * filter_param->a_lp2p[2];
	if (!isfinite(delay_element_0)) {
		// don't allow bad values to propagate via the filter
		delay_element_0 = rawdata;
	}
	
	output = delay_element_0 * filter_param->b_lp2p[0] + filter_param->delay_element_1 * filter_param->b_lp2p[1] + filter_param->delay_element_2 * filter_param->b_lp2p[2];
    
	filter_param->delay_element_2 = filter_param->delay_element_1;
	filter_param->delay_element_1 = delay_element_0;

	// return the value.  Should be no need to check limits
	return output;
}


