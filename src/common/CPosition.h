/*
 * File name: CPosition.h
 * Date:      2005/10/25 12:49
 * Author:    
 */
#include "CPoint.h"

#ifndef __CPOSITION_H__
#define __CPOSITION_H__

class CPosition: public CPoint
{
	public:
		CPosition();
		CPosition(float x,float y,float phi);
		void set(float x,float y,float phi);
		~CPosition();

		CPosition operator + (CPosition pos);
		CPosition operator - (CPosition pos);
		CPosition operator - ();
		CPosition operator / (float divisor);
		CPosition operator * (float multiplier);
		void normalizePhi();

		float phi;
};

#endif

/* end of CPosition.h */
