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
    float confidence;
}SNecklace;

class CNecklace{
    public:
        CNecklace(int bits,int minimalHamming);
        ~CNecklace();
        SNecklace get(int sequence, bool probabilistic=true, float confidence=1.0);
        SNecklace getEstimatedID();
        int printAll(int a[]);
		int verifyHamming(int a[],int bits,int len);
        float observationEstimation(float confidence);



    private:
        int maxID;
        bool debug;
        int length;
        int idLength; 
        SNecklace *idArray;
        SNecklace unknown;
        int* finalList;
	int getHamming(int a, int b);
	int getMinimalHamming(int a,int len);
};
#endif
