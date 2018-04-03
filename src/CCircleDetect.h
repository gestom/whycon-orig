/*
 * File name: CCircleDetect.h
 * Date:      2010
 * Author:   Tom Krajnik 
 */

#ifndef __CCIRCLEDETECT_H__
#define __CCIRCLEDETECT_H__

#include "CRawImage.h"
#include <math.h>
#define MAX_SEGMENTS 10000
#define COLOR_PRECISION 32
#define COLOR_STEP 8

typedef struct
{
	float x;
	float y;
	float angle,horizontal;
	int size;
	int maxy,maxx,miny,minx;
	int mean;
	int type;
	float roundness;
	float bwRatio;
	bool round;
	bool valid;
	float m0,m1;
	float v0,v1;
}SSegment;

class CCircleDetect
{
	public:
		CCircleDetect(int wi,int he);
		~CCircleDetect();
		void bufferCleanup(SSegment init);
		int adjustDimensions(int wi,int he);
		SSegment findSegment(CRawImage* image, SSegment init);
		bool examineSegment(CRawImage* image,SSegment *segmen,int ii,float areaRatio);
		void reconfigure(float ict,float fct,float art,float cdtr,float cdta);

		bool changeThreshold();
		bool debug,draw,drawAll;
	private:

		bool track;
		int maxFailed;
		int numFailed;
		int threshold; 

		int minSize; 
		int lastThreshold; 
		int thresholdBias; 
		int maxThreshold; 

		int thresholdStep;
		float circularTolerance;
		float circularityTolerance;
		float ratioTolerance;
		float centerDistanceToleranceRatio;
		int centerDistanceToleranceAbs;

		SSegment segmentArray[MAX_SEGMENTS];
		bool lastTrackOK;
		float outerAreaRatio,innerAreaRatio,areasRatio;
		int queueStart,queueEnd,queueOldStart,numSegments;
		static int width,height,len;
		int expand[4];
		unsigned char *ptr;
		int sizer,sizerAll;
		float diameterRatio;
		bool ownBuffer;
		static int *buffer;
		static int *queue;
};

#endif

/* end of CCircleDetect.h */
