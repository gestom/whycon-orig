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
        SNecklace get(int sequence, bool probabilistic=false, float confidence=1.0);
        int verifyHamming(int a[],int bits,int len);
        float observationEstimation(float confidence);

        int identifySegment(SSegment *segment, STrackedObject *object, CRawImage *image);

        bool debugSegment;      // for debugging identifySegment()

    private:
        SNecklace unknown;      // default unknown ID
        SNecklace *idArray;     // precalculated IDs
        float* probArray;       // probability for each ID
        int maxID;              // max ID used for Bayes probability
        int idSamples;          // samples to determine black/white signal
        int length;             // number of ID bits
        int idLength;           // amount of all possible IDs
        bool debug;             // for debugging rest of the class

        int getEstimatedID();
        int getHamming(int a, int b);
        int getMinimalHamming(int a,int len);
};
#endif
