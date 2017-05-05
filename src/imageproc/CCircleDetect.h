/*
 * File name: CCircleDetect.h
 * Date:      2014
 * Author:   Tom Krajnik, Matias Nitsche
 * Description: implements computationally efficient and precist circular pattern detection. Algorithm is described in Chapter 3 of the article [1]. 
 * Licence: if you use this class for your research, please cite [1]. 
 * References: [1] Krajnik, Nitsche et al.: A practical multirobot localization system. Journal of Intelligent and Robotic Systems, 2014.
 */

#ifndef __CCIRCLEDETECT_H__
#define __CCIRCLEDETECT_H__

#include "CNecklace.h"
#include "CRawImage.h"
#include "CTimer.h"
#include <math.h>
#include "cmath.h"
#define COLOR_PRECISION 32
#define COLOR_STEP 8
#define INNER 0
#define OUTER 1
#define MAX_PATTERNS 50 

//used for circle identification 
#define ID_SAMPLES 360
#define ID_BITS 9
#define HAMMING_DISTANCE 4


//this structure contains information related to image coordinates and dimensions of the detected pattern
typedef struct{
	float x;			//center in image coordinates
	float y;			//center in image coordinates
	float angle,horizontal;		//orientation (not really used in this case, see the SwarmCon version of this software) 
	int size;			//number of pixels
	int maxy,maxx,miny,minx;	//bounding box dimensions
	int mean;			//mean brightness
	int type;			//black or white ?
	float roundness;		//result of the first roundness test, see Eq. 2 of paper [1]
	float bwRatio;			//ratio of white to black pixels, see Algorithm 2 of paper [1] 
	bool round;			//segment passed the initial roundness test
	bool valid;			//marker passed all tests and will be passed to the transformation phase
	float m0,m1;			//eigenvalues of the pattern's covariance matrix, see Section 3.3 of [1]
	float v0,v1;			//eigenvectors of the pattern's covariance matrix, see Section 3.3 of [1]
	float r0,r1;			//ratio of inner vs outer ellipse dimensions (used to establish ID, see the SwarmCon version of this class)
	int ID;				//pattern ID (experimental, see the SwarmCon version of this class)
}SSegment;

class CCircleDetect
{

	public:
		//constructor, wi and he correspond to the image dimensions 
		CCircleDetect(int wi,int he,int ID = -1);

		//deallocate the detector's structures
		~CCircleDetect();

		//main detection method, implements Algorithm 2 of [1] 
		SSegment findSegment(CRawImage* image, SSegment init);

		//local pattern search - implements Algorithm 1 of [1]
		bool examineSegment(CRawImage* image,SSegment *segmen,int ii,float areaRatio);

		//calculate the pattern dimensions by means of eigenvalue decomposition, see 3.3 of [1]
		SSegment calcSegment(SSegment segment,int size,long int x,long int y,long int cm0,long int cm1,long int cm2);

		//establish circle ID (not used, see the SwarmCon version of this class)
		int identifySegment(SSegment* inner,CRawImage* image);
		//cleanup the shared buffers - see 3.6 of [1] 
		void bufferCleanup(SSegment init);

		//load descriptions for circle ID's
		int loadCircleID(const char* id);

		//change threshold if circle not detected, see 3.2 of [1]
		bool changeThreshold();

		//flags to draw results - used for debugging
		bool draw,drawAll,lastTrackOK;

		//debug level
		int debug;

		//used when selecting the circle by mouse click 
		bool localSearch;

		//attempt to identify segments 
		bool identify;
	private:
		//see the constructor in CCircleDetection.cpp for description of the following parameters
		CNecklace *decoder;
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
