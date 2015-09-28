/*
 * File name: CTransformation.h
 * Date:      2005/11/07 18:10
 * Author:    
 */

#ifndef __CTRANSFORMATION_H__
#define __CTRANSFORMATION_H__

#include <stdio.h>
#include <math.h>
#include <unistd.h>
#include "CCircleDetect.h"

typedef enum{
	TRANSFORM_NONE,
	TRANSFORM_2D,
	TRANSFORM_3D,
	TRANSFORM_4D,
	TRANSFORM_INV,
	TRANSFORM_NUMBER
}ETransformType;

typedef struct{
	float x,y,z,d;
	float pitch,roll,yaw;
	float roundness;
	float bwratio;
	float error;
	float esterror;
	int ID;
}STrackedObject;

typedef struct{
	float simlar[3][3];
	STrackedObject orig;
}S3DTransform;

class CTransformation
{
	public:
		CTransformation(int widthi,int heighti,float diam,bool fullUnbarreli = false);
		~CTransformation();

		float barrelX(float x,float y);
		float barrelY(float x,float y);
		float unbarrelX(float x,float y);
		float unbarrelY(float x,float y);
		float transformX(float x,float y);
		float transformY(float x,float y);
		void transformXY(float *ix,float *iy);
		void transformXYerr(float *ix,float *iy);

		void unbarrel(unsigned char* src,unsigned char* dst);
		STrackedObject transform(SSegment segment,bool unbarrel);
		STrackedObject eigen(double data[]);
		int calibrate2D(STrackedObject *inp,float dimX,float dimY,float robotRadius = 0,float robotHeight =0,float cameraHeight = 1.0);
		int calibrate3D(STrackedObject *o,float gridDimX,float gridDimY);
		int calibrate4D(STrackedObject *o,float gridDimX,float gridDimY);

		S3DTransform calibrate3D(STrackedObject o0,STrackedObject o1,STrackedObject o2,float gridDimX,float gridDimY);
		STrackedObject crossPrd(STrackedObject o0,STrackedObject o1,STrackedObject o2,float gridDimX,float gridDimY);

		ETransformType transformType;
		void saveCalibration(const char *str);
		void loadCalibration(const char *str);
		float distance(STrackedObject o1,STrackedObject o2);
		STrackedObject transformInv(STrackedObject o[]);

	private:
		STrackedObject  normalize(STrackedObject o);
		float establishError(STrackedObject o);
		STrackedObject transform2D(STrackedObject o);
		STrackedObject transform3D(STrackedObject o,int num = 4);
		STrackedObject transform4D(STrackedObject o);
		float *xArray;
		float *yArray;
		float *gArrayX;
		float *gArrayY;
		int *pArray;

		float hom[9];
		float trf4D[16];
		float gDimX,gDimY;
		S3DTransform D3transform[4];
		int width,height;
		bool fullUnbarrel;
		bool unbarrelInitialized;
		float trackedObjectDiameter;
		float kc[6];
		float kcerr[6];
		float fcerr[2];
		float fc[2];
		float cc[2];
		float error2D;
		STrackedObject c2D[4];
};

#endif
/* end of CTransformation.h */
