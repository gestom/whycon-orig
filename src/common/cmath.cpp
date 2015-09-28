#include "cmath.h"

double max(double a, double b)
{
	if (a < b) return b;
	return a;
}

double min(double a, double b)
{
	if (b < a) return b;
	return a;
}

int log2(int value)
{
	int r=0;
	while (value > 0){
		value = value/2;
		r++;
	}
	return r-1;
}									 

int exp2(int value)
{
	int r=1;
	for (int i = 0;i<value;i++){
		r=r*2;
	}
	return r;
}

float normalizeAngle(float a)
{
	while (a > +M_PI) a+=-2*M_PI;
	while (a < -M_PI) a+=+2*M_PI;
	return a;
}
