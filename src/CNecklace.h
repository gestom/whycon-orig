#ifndef __CNECKLACE_H__
#define __CNECKLACE_H__

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

typedef struct
{
    int id;
    int rotation;
    int hamming;
}SNecklace;

class CNecklace{
    public:
        CNecklace(int bits,int minimalHamming = 1);
        ~CNecklace();
        // param probabilistic changed from true to false, I dunno why it doesn't work with true
        SNecklace get(int sequence, bool probabilistic=false, float confidence=1.0);
        int verifyHamming(int a[],int bits,int len);
        float observationEstimation(float confidence);

    private:
        int maxID;
        bool debug;
        int length;
        int idLength; 
        SNecklace *idArray;
        float* probArray;
        SNecklace unknown;
        int getEstimatedID();
        int getHamming(int a, int b);
        int getMinimalHamming(int a,int len);
};
#endif
