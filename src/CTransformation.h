/*
 * File name: CTransformation.h
 * Date:      2014
 * Author:   Tom Krajnik, Matias Nitsche
 * Description: Transforms detected ellipse position and dimensions to arbitrary 3D or 2D coordinate frame. The method is described in Chapter 4 of the article [1]. 
 * Licence: if you use this class for your research, please cite [1]. 
 * References: [1] Krajnik, Nitsche et al.: A practical multirobot localization system. Journal of Intelligent and Robotic Systems, 2014.
 */

#ifndef CTRANSFORMATION_H
#define CTRANSFORMATION_H

#include <stdio.h>
#include <math.h>
#include <unistd.h>
#include <opencv2/opencv.hpp>
#include "SStructDefs.h"


using namespace cv;
using namespace std;

class CTransformation
{
    public:
        /*init: width & height of the image, diameter of the pattern, path to saved calibration file*/
        CTransformation(int widthi,int heighti,float diam, const char* calibDefPath);
        ~CTransformation();

        // parameters dynamic reconfigure
        void reconfigure(float circleDiam);

        // update of intrinsic and distortion camera params
        void updateParams(Mat intri, Mat dist);

        // get back image coords from canonical coords
        void reTransformXY(float *x, float *y,float *z);

        /*image to canonical coordinates (unbarrel + focal center and length)*/
        void transformXY(float *ix,float *iy);

        /*calculate marker 3D or 2D coordinates in user-defined coordinate system from the segment description provided by the CCircleDetector class, see 4.1-4.4 of [1] */
        STrackedObject transform(SSegment segment);

        /*calculate the pattern 3D position from its ellipse characteristic equation, see 4.3 of [1]*/
        STrackedObject calcEigen(double data[]);

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
        STrackedObject normalize(STrackedObject o);
        STrackedObject transform2D(STrackedObject o);
        STrackedObject transform3D(STrackedObject o,int num = 4);
        STrackedObject transform4D(STrackedObject o);

        S3DTransform D3transform[4];
        float trf4D[16];
        float hom[9];
        float gDimX,gDimY;
        float trackedObjectDiameter;
        int width,height;

        Mat intrinsic;
        Mat distCoeffs;
};

#endif
/* end of CTransformation.h */
