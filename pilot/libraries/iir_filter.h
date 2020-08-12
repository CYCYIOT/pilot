#ifndef _IIR_FILTER_H_
#define _IIR_FILTER_H_

#define CHEBY1     		1
#define CHEBY2     		2
#define BTWORTH    		3
#define IIR_LOWPASS    	1
#define IIR_HIGHPASS   	2

#define IIR_MAX_NUM 50
#define IIR_TEMP_NUM 20 
typedef struct{
	double a[IIR_MAX_NUM];
	double b[IIR_MAX_NUM];
	double w[IIR_TEMP_NUM];
	int num_sections;
	int filter_type;
	int band_type;
} iir_filter;

void bworth_filter(iir_filter *filter,float p_fre,float s_fre,float sample_fre);
void cheby_filter(iir_filter *filter,float p_fre,float s_fre,float sample_fre);

void iir_test();
void iir_filter_init(iir_filter *filter,int filter_type,int band_type,float p_fre,float s_fre,float sample_fre);
double signal_iir_filter(iir_filter *filter,double input);

#endif
