/*
 * File name: CCircleDetect.h
 * Date:      2010
 * Author:   Tom Krajnik 
 */

#ifndef __CCIRCLEDETECT_H__
#define __CCIRCLEDETECT_H__

#include "CRawImage.h"
#include "CTimer.h"
#include <math.h>
#include "cmath.h"
#define COLOR_PRECISION 32
#define COLOR_STEP 8
#define INNER 0
#define OUTER 1
#define MAX_PATTERNS 50 

typedef struct{
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
	float r0,r1;
	int ID;
}SSegment;

class CCircleDetect
{

	public:
		CCircleDetect(int wi,int he,int ID = -1);
		~CCircleDetect();
		void identifySegment(SSegment* segment);
		void bufferCleanup(SSegment init);
		SSegment findSegment(CRawImage* image, SSegment init);
		bool examineSegment(CRawImage* image,SSegment *segmen,int ii,float areaRatio);
		SSegment calcSegment(SSegment segment,int size,long int x,long int y,long int cm0,long int cm1,long int cm2);

		void clearCalibMask();
		void applyCalibMask(CRawImage* image);
		void addCalibMask();

		int loadCircleID(const char* id);
		bool changeThreshold();
		bool draw,drawAll,lastTrackOK;
		int debug;
		bool localSearch;
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
		bool enableCorrections;

		int ID;
		SSegment inner;
		SSegment outer;
		float outerAreaRatio,innerAreaRatio,areasRatio;
		int queueStart,queueEnd,queueOldStart,numSegments;
		int width,height,len,siz;
		int expand[4];
		unsigned char *ptr;
		CTimer timer;
		int tima,timb,timc,timd,sizer,sizerAll;
		float diameterRatio;
		bool ownBuffer;
		static int *buffer;
		static int *queue;
		static int *mask;
		static int maskNum;
		float idx[MAX_PATTERNS];
		float idy[MAX_PATTERNS];
		int numberIDs;
};

#endif

/* end of CCircleDetect.h */
