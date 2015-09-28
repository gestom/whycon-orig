#include "CPoint.h"

CPoint::CPoint()
{
	x = 0;
	y = 0;
}

CPoint::CPoint(float iX,float iY)
{
	x = iX;
	y = iY;
}

void CPoint::set(float iX,float iY)
{
	x = iX;
	y = iY;
}


CPoint::~CPoint()
{
}

float CPoint::distance(CPoint point)
{
	float result = sqrt((point.y-y)*(point.y-y)+(point.x-x)*(point.x-x));
	return result;
}

CPoint CPoint::operator + (CPoint point)
{
	CPoint result(x+point.x,y+point.y);
	return result;
}

CPoint CPoint::operator - (CPoint point)
{
	CPoint result(x-point.x,y-point.y);
	return result;
}

CPoint CPoint::operator - ()
{
	CPoint result(-x,-y);
	return result;
}

CPoint CPoint::operator / (float divisor)
{
	CPoint result;
	if (divisor!=0) { 
		result.x = x/divisor;
		result.y = y/divisor;
	}
	return result;
}

float CPoint::operator * (CPoint point)
{
	return x*point.x + y*point.y;
}

CPoint CPoint::operator * (float multiplier)
{
	CPoint result(x*multiplier,y*multiplier);
	return result;
}

