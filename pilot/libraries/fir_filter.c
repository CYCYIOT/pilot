#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <stdbool.h>
#include <fcntl.h>
#include <string.h>
#include <unistd.h>

#include "fir_filter.h"

#define PI 3.1415926
fir_filter filter;

void fir_filter_init(fir_filter *filter,int band_type,int order,float fre,float sample_fre)
{
	filter->order = order;
	filter->band_type = band_type;
	fir(filter,fre,sample_fre);
}

void fir_coef(fir_filter *filter,float fln,float fhn)
{
	int i,j,n2,mid;
	double s,wc1,delay;
	if((filter->order%2) == 0)
	{
		n2 = filter->order/2 - 1;
		mid = 1;
	}
	else
	{
		n2 = filter->order/2;
		mid = 0;
	}
	delay = filter->order/2.0f;
	wc1 = 2.0 * PI * fln;
	switch(filter->band_type)
	{
		case FIR_LOWPASS:
			{
				for(i = 0;i <= n2;i++)
				{
					s = i - delay;
					filter->h[i] = (sin(wc1 * s)/(PI * s)) * (0.54 - 0.46 * cos(2 * i * PI/filter->order));
					filter->h[filter->order-i] = filter->h[i];
				}
				if(mid == 1)
					filter->h[filter->order/2] = wc1/PI;
			}
			break;
		case FIR_HIGHPASS:
			{
				for(i = 0;i <= n2;i++)
				{
					s = i - delay;
					filter->h[i] = (sin(PI * s) - sin(wc1 * s))/(PI * s) * (0.54 - 0.46 * cos(2 * i * PI/filter->order));
					filter->h[filter->order-i] = filter->h[i];
				}
				if(mid == 1)
					filter->h[filter->order/2] = 1.0 - wc1/PI;
			}
		break;	
	}
	for(i = 0;i <= n2;i++)
	{
		j = filter->order - i;
		printf("h(%2d) = %12.8lf = h(%2d)\n",i,filter->h[i],j);
	}
}

void fir(fir_filter *filter,float fre,float sample_fre)
{
	double fl = 0.0f,fh = 0.0f;
	if(filter->band_type == FIR_LOWPASS)
		fl = fre/sample_fre;
	else
		fl = fre/sample_fre;
	fir_coef(filter,fl,fh);
}
/*
double signal_fir_filter(fir_filter *filter,double dataIn)
{
	double dataOut = 0.0f;
	int i;
	if(filter->index < filter->order){
		filter->buffer[filter->index++] = dataIn;
		dataOut = dataIn;
	}
	else{
		for(i = filter->order;i > 0;i--)
			filter->buffer[i] = filter->buffer[i-1];
		filter->buffer[0] = dataIn;
		for(i = 0;i < filter->order;i++)
			dataOut += filter->h[i] * filter->buffer[i];	
	}
	return dataOut;
}
*/
double signal_fir_filter(fir_filter *filter,double dataIn)
{
	double dataOut = 0.0f;
	int i;
	
	for(i = filter->order;i > 0;i--)
		filter->buffer[i] = filter->buffer[i-1];
	filter->buffer[0] = dataIn;
	for(i = 0;i < filter->order;i++)
		dataOut += filter->h[i] * filter->buffer[i];	
	
	return dataOut;
}

void gain(double *b,double *a,int m,int n,double *x,double *y,int len,int sign)
{
	int i ,k;
	double ar,ai,br,bi,zr,zi,im,re,den,numr,numi,freq,temp;
	for(k = 0;k < len;k++)
	{
		
		freq = 1.0 * k/(len - 1);
		zr = cos(1.0 * PI * freq);
		zi = sin(-1.0 * PI * freq);	
		br = 0.0f;
		bi = 0.0f;
		for(i = m;i > 0;i--)
		{
			re = br;
			im = bi;
			br = (re + b[i]) * zr - im * zi;
			bi = (re + b[i]) * zi + im * zr;
		}
		ar = 0;
		ai = 0;
		for(i = n;i > 0;i--)
		{
			re = ar;
			im = ai;
			ar = (re + a[i]) * zr - im * zi;
			ai = (re + a[i]) * zi + im * zr;
		}
		br = br + b[0];
		ar = ar + 1.0;
		numr = ar * br + ai * bi;
		numi = ar * bi - ai * br;
		den = ar * ar + ai * ai;
		x[k] = numr/den;
		y[k] = numi/den;
		switch(sign)
		{
			case 1:
				temp = sqrt(x[k] * x[k] + y[k] * y[k]);
				y[k] = atan2(y[k],x[k]);
				x[k] = temp;
				break;
			case 2:
				temp = sqrt(x[k] * x[k] + y[k] * y[k]);
				y[k] = atan2(y[k],x[k]);
				x[k] = 20.0 * log10(temp);
		}
	}
}

void fir_test(fir_filter *filter)
{
	int i;
	double freq,c[100],x[300],y[300];
	FILE *fp;
	if((fp = fopen("test.dat","w")) == NULL)
	{
		printf("can not open this file\n");
		exit(1);
	}
	printf(" ************ FIR FILTER ***********\n\n");

	gain(filter->h,c,filter->order,1,x,y,300,2);
	for( i = 0;i < 300;i++)
	{
		freq = 2.0/3.0 * i;
		fprintf(fp,"%lf %lf \n",freq,x[i]);
	}
	fclose(fp);
}


