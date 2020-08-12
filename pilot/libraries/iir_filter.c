#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <stdbool.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>

#include "iir_filter.h"
#define PI 3.1415926
#define ALPHA_P   		3.0f
#define ALPHA_S   		50.0f
iir_filter filter;

void iir_filter_init(iir_filter *filter,int filter_type,int band_type,float p_fre,float s_fre,float sample_fre)	
{
	filter->filter_type = filter_type;
	filter->band_type = band_type;
	switch(filter_type){
		case BTWORTH:
			bworth_filter(filter,p_fre,s_fre,sample_fre);
			break;
		case CHEBY1:
		case CHEBY2:	
			cheby_filter(filter,p_fre,s_fre,sample_fre);
			break;
		}	
}
static double cosh1(double x)
{
	double z;
	z = log(x + sqrt(x * x - 1.0));
	return z;
}
static double warp(double f)
{
	double z;
	z = tan(PI * f);
	return z;
}
static int butter_order(double passband_fre,double stopband_fre,double sample_fre,double alpha_p,double alpha_s,int type)
{
	double omega_p,omega_s;
	double t1,t2,t3,t4;
	int order,num_sections;
	omega_p = tan(PI * passband_fre/sample_fre);
	omega_s = tan(PI * stopband_fre/sample_fre);
	t1 = pow(10.0,0.1 * alpha_s) - 1.0;
	t2 = pow(10.0,0.1 * alpha_p) - 1.0;
	if (type == IIR_LOWPASS){
		t3 = 2.0 * log10(omega_s/omega_p);
	}
	else{
		t3 = 2.0 * log10(omega_p/omega_s);
	}
	t4 = log10((t1/t2))/t3;
	order = (int)ceil(t4);
	if(order%2)
		order += 1;
	num_sections = order/2;
	return num_sections;
}
static int cheby_order(double passband_fre,double stopband_fre,double sample_fre,double alpha_p,double alpha_s,int type)
{
	double omega_p,omega_s;
	double t1,t2,t3,t4;
	int order,num_sections;
	omega_p = tan(PI * passband_fre/sample_fre);
	omega_s = tan(PI * stopband_fre/sample_fre);
	t1 = pow(10.0,0.1 * alpha_s) - 1.0;
	t2 = pow(10.0,0.1 * alpha_p) - 1.0;
	if(type == IIR_LOWPASS)
		t3 = cosh1(omega_s/omega_p);
	else
		t3 = cosh1(omega_p/omega_s);
	t4 = cosh1(sqrt(t1/t2))/t3;
	order = (int)ceil(t4);
	if(order%2)
		order += 1;
	num_sections = order/2;
	return num_sections;
}
static void bilinear(double d[],double c[],double b[],double a[],int n)
{
	
	int i,j,n1;
	double sum = 0.0,atmp = 0.0,scale = 0.0;
	double temp[256] = {0.0f};
	n1 = n + 1;
	for(j = 0;j <= n;j++)
		temp[j * n1 + 0] = 1.0;
	sum = 1.0;
	for(i = 1;i <= n;i++)
	{
		sum = sum * (double)(n - i + 1)/(double)i;
		temp[0 * n1 + i] = sum;
	}
	for(i = 1;i <= n;i++)
		for(j = 1;j <= n;j++)
		{
			temp[j * n1 + i] = temp[(j - 1) * n1 + i] - temp[j * n1 + i - 1] - temp[(j - 1) * n1 + i - 1];
		}
	for(i = n;i >= 0;i--)
	{
		b[i] = 0.0;
		atmp = 0.0;
		for(j = 0;j <= n;j++)
		{
			b[i] = b[i] + temp[j * n1 + i] * d[j];
			atmp = atmp + temp[j * n1 + i] * c[j];
		}
		scale = atmp;
		if(i != 0)
			a[i] = atmp;
	}

	for(i = 0;i <= n;i++)
	{
		b[i] = b[i]/scale;
		a[i] = a[i]/scale;
	}

	a[0] = 1.0;
}
static void bwtf(int ln,int k,int n,double *d,double *c)
{
	int i;
	double tmp;
	d[0] = 1.0f;
	c[0] = 1.0f;
	for(i = 1;i < n;i++)
	{
		d[i] = 0.0f;
		c[i] = 0.0f;
	}
	tmp = (k + 1)-(ln + 1.0)/2.0;
	if(tmp == 0.0f)
		c[1] = 1.0f;
	else
	{
		c[1] = -2.0f * cos((2 * (k + 1) + ln - 1) *PI/(2 * ln));
		c[2] = 1.0f;
	}
}
static void cheby1(int ln,int k,int n,double ep,double *d,double *c)
{
	int i;
	double gam,omega,sigma;
	gam = pow(((1.0 + sqrt(1.0 + ep * ep))/ep),1.0/ln);
	sigma = 0.5 * (1.0/gam - gam) * sin((2 * (k + 1) - 1) * PI/(2 * ln));
	omega = 0.5 * (1.0/gam + gam) * cos((2 * (k + 1) - 1) * PI/(2 * ln));
	for(i = 0;i <= n;i++)
	{
		d[i] = 0.0;
		c[i] = 0.0;
	}
	if(((ln%2) == 1) && ((k+1) == (ln + 1)/2))
	{
		d[0] = -sigma;
		c[0] = d[0];
		c[1] = 1.0;
	}
	else
	{
		c[0] = sigma * sigma + omega * omega;
		c[1] = -2.0 * sigma;
		c[2] = 1.0;
		d[0] = c[0];
		if(((ln%2) == 0) && (k == 0))
			d[0] = d[0]/sqrt(1.0 + ep * ep);
	}
}
static void cheby2(int ln,int k,int n,double ws,double att,double *d,double *c)
{
	int i;
	double pi,gam,alpha,beta,sigma,omega,scln,scld;
	pi = 4.0 * atan(1.0);
	gam = pow((att + sqrt(att * att - 1.0)),1.0/ln);
	alpha = 0.5 * (1.0/gam - gam) * sin((2 * (k + 1) - 1) * PI/(2 * ln));
	beta = 0.5 * (1.0/gam + gam) * cos((2 * (k + 1) - 1) * PI /(2 * ln));
	sigma = ws * alpha/(alpha * alpha + beta * beta);
	omega = -1.0 * ws * beta/(alpha * alpha + beta * beta);
	for(i = 0;i <= n;i++)
	{
		d[i] = 0.0f;
		c[i] = 0.0f;
	}
	if(((ln%2) == 1) && ((k+1) == (ln + 1)/2))
	{
		d[0] = -1.0f * sigma;
		c[0] = d[0];
		c[1] = 1.0f;
	}
	else
	{
		scln = sigma * sigma + omega * omega;
		scld = pow((ws/cos((2 * (k + 1) - 1) * pi/(2 * ln))),2);
		d[0] = scln * scld;
		d[2] = scln;
		c[0] = d[0];
		c[1] = -2.0f * sigma * scld;
		c[2] = scld;
	}
 
}
static void fblt(double d[],double c[],int n,int band,double fln,double fhn,double b[],double a[])
{	
	int i,m;
	double w1,tmp;
	w1 = tan(PI * fln);
	for(i = n;i >= 0;i--)
	{
		if((c[i] != 0.0) || (d[i] != 0.0))
			break;
	}
	m = i;
	switch(band)
	{
		case 1:
		case 2:
			if(band == 2)
			{
				for (i = 0; i <= m/2;i++)
				{
					tmp = d[i];
					d[i] = d[m-i];
					d[m-i] = tmp;
					tmp = c[i];
					c[i] = c[m-i];
					c[m-i] = tmp;
				}
			}
			for(i = 0;i <= m;i++)
			{
				d[i] = d[i]/pow(w1,i);
				c[i] = c[i]/pow(w1,i);
			}
			break;
	}
	bilinear(d,c,b,a,n);
}
void iirbcf(int ifilt, int band, int ns, int n, double f1, double f2,double db ,double b[], double a[])
{
//	int i,k;
	int k;
	double omega = 0.0f,lamda = 0.0f,epslon = 0.0f,fl = 0.0f,fh = 0.0f;
	double d[5],c[5];
	if((band == 1) || (band == 4))
		fl = f1;
	if((band == 2) || (band == 3))
		fl = f2;
	if(ifilt < 3)
	{
		switch(band)
		{
			case 1:
			case 2:
				omega = warp(f2)/warp(f1);
				break;
		}
			lamda = pow(10.0,(db/20.0));
			epslon = lamda/cosh(2 * ns * cosh1(omega));	
	}
		for(k = 0;k < ns;k++)
		{
			switch(ifilt)
			{
				case 1:
					cheby1(2 * ns,k,4,epslon,d,c);
					break;
				case 2:
					cheby2(2 * ns,k,4,omega,lamda,d,c);
					break;
				case 3:
					bwtf(2 * ns,k,4,d,c);
					break;
			}
		fblt(d,c,n,band,fl,fh,&b[k * (n + 1) + 0],&a[k * (n + 1) + 0]);
		}
		/*for(k = 0;k < ns;k++)
		{
			printf("\n section %d\n\n",k + 1);
			for( i = 0;i <= n;i++)
			{
				printf("   b[%d][%d] = %10.7lf",k,i,b[k * (n + 1) + i]);
				if(((i % 2) == 0) && (i != 0))
					printf("\n");
			}
			printf("\n");
			for( i = 0;i <= n;i++)
			{
				printf("   a[%d][%d] = %10.7lf",k,i,a[k * (n + 1) + i]);
				if(((i % 2) == 0) && (i != 0))
					printf("\n");
			}
			printf("\n");
		}*/
}
void bworth_filter(iir_filter *filter,float p_fre,float s_fre,float sample_fre)
{
	int num_sections;
	double f1 = 0.0f,f2 = 0.0f;
	if(filter->band_type == IIR_LOWPASS)
		f1 = p_fre/sample_fre;
	else
		f2 = s_fre/sample_fre;
	num_sections = butter_order(p_fre,s_fre,sample_fre,ALPHA_P,ALPHA_S,filter->band_type);
	filter->num_sections = num_sections;
	iirbcf(filter->filter_type,filter->band_type,filter->num_sections,2,f1,f2,0,filter->b,filter->a);
}
void cheby_filter(iir_filter *filter,float p_fre,float s_fre,float sample_fre)
{
	double f1 = 0.0f,f2 = 0.0f;
	int num_sections;
	if(filter->band_type == IIR_LOWPASS){
		f1 = p_fre/sample_fre;
		f2 = s_fre/sample_fre;
	}
	else{
		f1 = s_fre/sample_fre;
		f2 = p_fre/sample_fre;
	}	
	num_sections = cheby_order(p_fre,s_fre,sample_fre,ALPHA_P,ALPHA_S,filter->band_type);
	filter->num_sections = num_sections;
	iirbcf(filter->filter_type,filter->band_type,filter->num_sections,2,f1,f2,ALPHA_S,filter->b,filter->a);	
}

/*
void bworth_lowpass(iir_filter *filter,float p_fre,float s_fre,float sample_fre)
{
	int num_sections;
	double f1 = 0.0f,f2 = 0.0f;
	f1 = p_fre/sample_fre;
	num_sections = butter_order(p_fre,s_fre,ALPHA_P,ALPHA_S,filter->band_type);
	filter->num_sections = num_sections;
	iirbcf(filter->filter_type,filter->band_type,filter->num_sections,2,f1,f2,0,filter->b,filter->a);
}
void bworth_highpass(iir_filter *filter,float p_fre,float s_fre,float sample_fre)
{
	double f1 = 0.0f,f2 = 0.0f;
	f2 = PASSBAND_FRE/SAMPLE_FRE;
	num_sections = butter_order(PASSBAND_FRE,STOPBAND_FRE,ALPHA_P,ALPHA_S,HIGHPASS);
	iirbcf(BTWORTH,HIGHPASS,num_sections,2,f1,f2,0,b,a);
}
void cheby1_lowpass(iir_filter *filter,float p_fre,float s_fre,float sample_fre)
{
	double f1 = 0.0f,f2 = 0.0f;
	f1 = PASSBAND_FRE/SAMPLE_FRE;
	f2 = STOPBAND_FRE/SAMPLE_FRE;
	num_sections = cheby_order(PASSBAND_FRE,STOPBAND_FRE,ALPHA_P,ALPHA_S,LOWPASS);
	iirbcf(CHEBY1,LOWPASS,num_sections,2,f1,f2,ALPHA_S,b,a);
	
}
void cheby1_highpass(iir_filter *filter,float p_fre,float s_fre,float sample_fre)
{
	double f1 = 0.0f,f2 = 0.0f;
	num_sections = cheby_order(PASSBAND_FRE,STOPBAND_FRE,ALPHA_P,ALPHA_S,HIGHPASS);
	f1 = STOPBAND_FRE/SAMPLE_FRE;
	f2 = PASSBAND_FRE/SAMPLE_FRE;
	iirbcf(CHEBY1,HIGHPASS,num_sections,2,f1,f2,ALPHA_S,b,a);
}

void cheby2_lowpass(iir_filter *filter,float p_fre,float s_fre,float sample_fre)
{
	double f1 = 0.0f,f2 = 0.0f;
	num_sections = cheby_order(PASSBAND_FRE,STOPBAND_FRE,ALPHA_P,ALPHA_S,LOWPASS);
	f1 = PASSBAND_FRE/SAMPLE_FRE;
	f2 = STOPBAND_FRE/SAMPLE_FRE;
	iirbcf(CHEBY2,LOWPASS,num_sections,2,f1,f2,ALPHA_S,b,a);	
}
void cheby2_highpass(iir_filter *filter,float p_fre,float s_fre,float sample_fre)
{
	double f1 = 0.0f,f2 = 0.0f;
	num_sections = cheby_order(PASSBAND_FRE,STOPBAND_FRE,ALPHA_P,ALPHA_S,HIGHPASS);
	f1 = STOPBAND_FRE/SAMPLE_FRE;
	f2 = PASSBAND_FRE/SAMPLE_FRE;
	iirbcf(CHEBY2,HIGHPASS,num_sections,2,f1,f2,ALPHA_S,b,a);
}
*/
double signal_iir_filter(iir_filter *filter,double input)
{
	int section = 0;
	static double wn = 0.0f,yn = 0.0f;	
	
	for(section = 0;section < filter->num_sections;section++){
		wn = input - filter->a[section * 3 + 1] * filter->w[section * 2 + 0] - filter->a[section * 3 + 2] * filter->w[section * 2 + 1];
		yn = filter->b[section * 3 + 0] * wn + filter->b[section * 3 + 1] * filter->w[section * 2 + 0] + filter->b[section * 3 + 2] * filter->w[section * 2 + 1];
		filter->w[section * 2 + 1] = filter->w[section * 2 + 0];
		filter->w[section * 2 + 0] = wn;
		input = yn;
	}
	return yn;
}

void gainc(double b[], double a[], int n, int ns, double x[], double y[], int len, int sign)
{
	int i,j,k,nl;
	double ar,ai,br,bi,zr,zi,im,re,den,numr,numi,freq,temp;
	double hr,hi,tr,ti;
	nl = n + 1;
	for(k = 0;k < len;k++)
	{
		freq = k * 0.5 /(len - 1);
		zr = cos(- 8.0 * atan(1.0) * freq);
		zi = sin(- 8.0 * atan(1.0) * freq);
		x[k] = 1.0;
		y[k] = 0.0;
		for(j = 0;j < ns;j++)
		{
			br = 0.0;
			bi = 0.0;
			for(i = n;i > 0;i--)
			{
				re = br;
				im = bi;
				br = (re + b[j * nl + i]) * zr - im * zi;
				bi = (re + b[j * nl + i]) * zi + im * zr;
			}
			ar = 0.0;
			ai = 0.0;
			for(i = n;i > 0;i--)
			{
				re = ar;
				im = ai;
				ar = (re + a[j * nl + i]) * zr - im *zi;
				ai = (re + a[j * nl + i]) * zi + im *zr;
			}
			br = br + b[j * nl + 0];
			ar = ar + 1.0;
			numr = ar * br + ai * bi;
			numi = ar * bi - ai * br;
			den = ar * ar + ai * ai;
			hr = numr / den;
			hi = numi / den;
			tr = x[k] * hr - y[k] * hi;
			ti = x[k] * hi + y[k] * hr;
			x[k] = tr;
			y[k] = ti;
		}
		switch(sign)
		{
		case 1:
			temp = sqrt(x[k] * x[k] + y[k] * y[k]);
			if(temp != 0)
				y[k] = atan2(y[k],x[k]);
			else
				y[k] = 0.0;
			x[k] = temp;
			break;
		case 2:
			temp = x[k] * x[k] + y[k] * y[k];
			if(temp != 0)
				y[k] = atan2(y[k],x[k]);
			else
			{
				temp = 1.0e-40;
				y[k] = 0.0;
			}
			x[k] = 10.0 * log10(temp);
		}

	}

}

void iir_test(iir_filter *filter)
{
	int i;
	double x[300] = {0},y[300] = {0},freq;
	FILE *fp;
	if((fp = fopen("test.dat","w")) == NULL)
		{
			printf("can not open file\n");
			exit(0);
		}
	gainc(filter->b,filter->a,2,filter->num_sections,x,y,300,2);
	for( i = 0;i < 300;i++)
	{
		freq = (float)i * 0.5 /300.0 ;
		fprintf(fp,"%lf %lf\n",freq,x[i]);
	}
	fclose(fp);
}
