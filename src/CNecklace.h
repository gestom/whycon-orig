#ifndef __CNECKLACE_H__
#define __CNECKLACE_H__

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "CCircleDetect.h"
#include "CTransformation.h"
#include "CRawImage.h"

typedef struct
{
    int id;
    int rotation;
    int hamming;
}SNecklace;

class CNecklace{
    public:
        CNecklace(int bits,int samples,int minimalHamming = 1);
        ~CNecklace();
        SNecklace get(int sequence, bool probabilistic=true, float confidence=1.0);
        int verifyHamming(int a[],int bits,int len);
        float observationEstimation(float confidence);

        int identifySegment(SSegment *segment, STrackedObject *object, CRawImage *image);

        bool debugSegment;

    private:
        SNecklace unknown;
        SNecklace *idArray;
        float* probArray;
        int maxID;
        int idSamples;          // samples to determine black/white signal
        int length;             // number of ID bits
        int idLength; 
        bool debug;

        int getEstimatedID();
        int getHamming(int a, int b);
        int getMinimalHamming(int a,int len);
};
#endif
