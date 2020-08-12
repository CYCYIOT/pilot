#ifndef _FIR_FILTER_H_
#define _FIR_FILTER_H_
#define FIR_LOWPASS  1
#define FIR_HIGHPASS 2

#define FIR_MAX_ORDER  100
typedef struct{
	double h[FIR_MAX_ORDER];
	double buffer[FIR_MAX_ORDER];
	int index;
	int order;
	int band_type;
} fir_filter;

double signal_fir_filter(fir_filter *filter,double dataIn);
void fir(fir_filter *filter,float fre,float sample_fre);
void fir_filter_init(fir_filter *filter,int band_type,int order,float fre,float sample_fre);
void fir_test();
#endif
