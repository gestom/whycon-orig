/*
 * File name: CTransformation.h
 * Date:      2014
 * Author:   Tom Krajnik, Matias Nitsche
 * Description: Transforms detected ellipse position and dimensions to arbitrary 3D or 2D coordinate frame. The method is described in Chapter 4 of the article [1]. 
 * Licence: if you use this class for your research, please cite [1]. 
 * References: [1] Krajnik, Nitsche et al.: A practical multirobot localization system. Journal of Intelligent and Robotic Systems, 2014.
 */

#ifndef __CTRANSFORMATION_H__
#define __CTRANSFORMATION_H__

#include <stdio.h>
#include <math.h>
#include <unistd.h>
#include <opencv2/opencv.hpp>
#include "CCircleDetect.h"

using namespace cv;
using namespace std;

/*which transform to use*/
typedef enum{
	TRANSFORM_NONE,			//camera-centric
	TRANSFORM_2D,			//3D->2D homography 
	TRANSFORM_3D,			//3D user-defined - linear combination of four translation/rotation transforms 
	TRANSFORM_4D,			//3D user-defined - full 4x3 matrix
	TRANSFORM_INV,			//for testing purposes
	TRANSFORM_NUMBER
}ETransformType;

typedef struct{
	float x,y,z,d;			//position and distance from the camera
	float pitch,roll,yaw;		//angles - precision is unknown
	float roundness;		//segment roundness as calculated by 5 of [1]
	float bwratio;			//black/white area ratio
	int ID;				//ID - not used here
}STrackedObject;

typedef struct{				//rotation/translation model of the 3D transformation
	float simlar[3][3];		//rotation description	
	STrackedObject orig;		//translation vector
}S3DTransform;

class CTransformation
{
	public:
		/*initialization: width and height of the image, diameter of the pattern, 'unbarrel the entire image' flag*/
		CTransformation(int widthi,int heighti,float diam, const char* calibDefPath);
		~CTransformation();

		// parameters dynamic reconfigure
		void reconfigure(float circleDiam);

		//update of intrinsic and distortion camera params
		void updateParams(Mat intri, Mat dist);

		/*image to canonical coordinates (unbarrel + focal center and length)*/
		void transformXY(float *ix,float *iy);

		/*calculate marker 3D or 2D coordinates in user-defined coordinate system from the segment description provided by the CCircleDetector class, see 4.1-4.4 of [1] */
		STrackedObject transform(SSegment segment);
		
		/*calculate the pattern 3D position from its ellipse characteristic equation, see 4.3 of [1]*/
		STrackedObject eigen(double data[]);
		
		/*establish the user-defined coordinate system from four calibration patterns - see 4.4 of [1]*/
		int calibrate2D(STrackedObject *inp,float dimX,float dimY,float robotRadius = 0,float robotHeight =0,float cameraHeight = 1.0);
		int calibrate3D(STrackedObject *o,float gridDimX,float gridDimY);
		S3DTransform calibrate3D(STrackedObject o0,STrackedObject o1,STrackedObject o2,float gridDimX,float gridDimY);

		/*supporting methods*/
		ETransformType transformType;
		void saveCalibration(const char *str);
		void loadCalibration(const char *str);
		float distance(STrackedObject o1,STrackedObject o2);

	private:
		STrackedObject  normalize(STrackedObject o);
		STrackedObject transform2D(STrackedObject o);
		STrackedObject transform3D(STrackedObject o,int num = 4);
		STrackedObject transform4D(STrackedObject o);

		float hom[9];
		float trf4D[16];
		float gDimX,gDimY;
		S3DTransform D3transform[4];
		int width,height;
		float trackedObjectDiameter;

		Mat intrinsic;
		Mat distCoeffs;
};

#endif
/* end of CTransformation.h */
