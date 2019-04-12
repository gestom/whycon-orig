#ifndef CCIRCLEDETECT_H
#define CCIRCLEDETECT_H

#include <math.h>
#include "CRawImage.h"
#include "CTransformation.h"
#include "CNecklace.h"
#include "SStructDefs.h"

/*TODO note #07*/
#define COLOR_PRECISION 32
#define COLOR_STEP 8
#define INNER 0
#define OUTER 1
#define MAX_PATTERNS 50


extern CTransformation *trans;  // allows to transform from image to metric coordinates
extern CNecklace *decoder;      // Necklace code decoder

class CCircleDetect {
    public:
        //constructor, wi and he correspond to the image dimensions 
        CCircleDetect(int wi,int he, bool id, int bits, int samples, int dist);

        //deallocate the detector's structures
        ~CCircleDetect();

        // dynamic reconfigure of parameters
        void reconfigure(float ict,float fct,float art,float cdtr,float cdta, bool id, int minS);

        //main detection method, implements Algorithm 2 of [1] 
        SMarker findSegment(CRawImage* image, SSegment init);

        //local pattern search - implements Algorithm 1 of [1]
        bool examineSegment(CRawImage* image,SSegment *segmen,int ii,float areaRatio);

        //calculate the pattern dimensions by means of eigenvalue decomposition, see 3.3 of [1]
        SSegment calcSegment(SSegment segment,int size,long int x,long int y,long int cm0,long int cm1,long int cm2);

        //cleanup the shared buffers - see 3.6 of [1] 
        void bufferCleanup(SSegment init);

        /*TODO note #21*/
        //load descriptions for circle ID's
        //int loadCircleID(const char* id);

        //change threshold if circle not detected, see 3.2 of [1]
        bool changeThreshold();

        //normalise the angle
        float normalizeAngle(float a);

        // adjust the dimensions of the image, when the image size changes
        int adjustDimensions(int wi, int he);

        void ambiguityAndObtainCode(CRawImage *image);
        void ambiguityPlain();

        bool draw,lastTrackOK;      // flags to draw results - used for debugging
        int debug;                  // debug level
        bool localSearch;           // used when selecting the circle by mouse click
        bool identify;              // attempt to identify segments

        

    private:
        //see the constructor in CCircleDetection.cpp for description of the following parameters
        int idBits;
        int idSamples;
        int hammingDist;
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
        int step;
        SSegment inner;
        SSegment outer;
        float outerAreaRatio,innerAreaRatio,areasRatio;
        int queueStart,queueEnd,queueOldStart,numSegments;
        int width,height,len,siz;
        int expand[4];
        unsigned char *ptr;
        int tima,timb,timc,timd,sizer,sizerAll;
        float diameterRatio;
        bool ownBuffer;
        static int *buffer;
        static int *queue;
        // static int *mask;
        // static int maskNum;
        float idx[MAX_PATTERNS];
        float idy[MAX_PATTERNS];
        int numberIDs;

        STrackedObject trackedObject;
};

#endif

/* end of CCircleDetect.h */
