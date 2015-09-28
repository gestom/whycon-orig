/*
 * File name: CPoint.h
 * Date:      2005/10/25 12:49
 * Author:    
 */

#ifndef __CPOINT_H__
#define __CPOINT_H__

#include <math.h>

class CPoint
{
	public:
		CPoint();
		CPoint(float x,float y);
		void set(float x,float y);
		~CPoint();

		float distance(CPoint point);
		CPoint operator + (CPoint point);
		CPoint operator - (CPoint point);
		CPoint operator - ();
		CPoint operator / (float divisor);
		float operator * (CPoint point);
		CPoint operator * (float multiplier);
		float x;
		float y;
};

#endif

/* end of CPoint.h */
