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
bool debug = false;

class CNecklace{
    public:
        CNecklace(int bits,int minimalHamming);
        ~CNecklace();
        SNecklace get(int sequence);
        int printAll(int a[]);
	int verifyHamming(int a[],int bits,int len);


    private:
        // int bitshift(int a);
        int length;
        int idLength; 
        SNecklace *idArray;
        SNecklace unknown;
        int* finalList;
	int getHamming(int a, int b);
	int getMinimalHamming(int a,int len);
};
#endif

CNecklace::CNecklace(int bits,int minimalHamming = 1)
{
    length = bits;
    idLength = pow(2,length); 
    idArray = (SNecklace*)malloc(sizeof(SNecklace)*idLength);

    int currentID = 0;
    int tempID,bit,rotations;
    int minHam = 1000;
    int ham = 1000;

    /*for every possible id*/
    for (int id = 0;id<idLength;id++)
    {
        /*check if there is a lower number that could be created by bitshifting it*/
        tempID  = id;
        rotations = 0;
        int cached [length - 1];
        bool isSymmetrical = false;
	minHam = 1000;
	if (debug) printf("Testing %i\n",tempID);
        do{
	    ham = getMinimalHamming(tempID,id);
	    if (minHam > ham) minHam = ham;
            bit = tempID%2;
            tempID=tempID/2+bit*pow(2,length-1);

            if(bit || id == 0)
            {
                for (int i = 0; i < rotations && !isSymmetrical; i++)
                {
                    // check for symmetry
                    if (cached[i] == tempID){
                        isSymmetrical = true;
                    }
                }
            }
            cached[rotations] = tempID; 
	    //ham = getMinimalHamming(tempID,id);
	    //if (minHam > ham) minHam = ham;
        //}while (rotations++<length-1 && id <= tempID && !isSymmetrical);
        }while (rotations++<length-1 && !isSymmetrical);
	if (minHam < minimalHamming) isSymmetrical = true;
        if(isSymmetrical)
        {
            idArray[id].id = -1;
            idArray[id].rotation = -1;
        } 
	else if (id > tempID)
        {
            if(idArray[tempID].id != -1)
            {
                idArray[id] = idArray[tempID];
                idArray[id].rotation +=rotations;
                idArray[id].rotation = 1;
            }
        }
	else
        {
	    if (debug) printf("Adding %i\n",currentID);
            idArray[id].id =currentID++;
            idArray[id].rotation = 0;
            idArray[id].hamming = minHam;
        }
    }

    idArray[idLength-1].id = 0;
    idArray[idLength-1].rotation = 0;
    unknown.id = -1;
    unknown.rotation = -1;

}

CNecklace::~CNecklace()
{
    free(idArray);
}

int CNecklace::printAll(int a[])
{

    int count = 0;

    for (int i = 0; i <= idLength; i++)
    {
        if(get(i).id == count){
            a[count++] = i;
            printf("%i %i\n", get(i).id, i);
        }
    }
    return count;
}

int CNecklace::getHamming(int a, int b)
{
	int aa = a;
	int bb = b;
	int ham = 0;
	do {
		if (a%2 != b%2) ham++;
		a = a/2;
		b = b/2;
	}while (a > 0 || b > 0);
	if (debug) printf("Hamming %i %i is %i\n",aa,bb,ham);
	return ham;
}

int CNecklace::getMinimalHamming(int a,int len)
{
	int minDist = 10000;
	for (int i = 1;i<len;i++){
		if (get(i).rotation == 0){
			int m = getHamming(a,i);
			if (minDist > m){
			       	minDist = m;
				//if (minDist < 3) printf("%i is same as %i\n",a,i);
			}
		
		}
	}
	if (debug) printf("Minimal hamming of %i is %i\n",a,minDist);
	return minDist;
}

int CNecklace::verifyHamming(int a[],int bits,int len)
{
	int overAll = 10000;
	for (int i = 0;i<len;i++){
		for (int j = 0;j<len;j++)
		{
			int minimal = 10000;
			if (i!=j)
			{
				int bit;
				int tempID = a[j];
				int distance;
				if (debug) printf("Testing %i vs %i\n",a[i],a[j]);
				for (int r = 0;r<bits;r++){
					distance = getHamming(a[i],tempID);
					if (debug) printf("Test %i %i %i\n",a[i],tempID,distance);
					if (minimal > distance) minimal = distance;
					bit = tempID%2;
					tempID=tempID/2+bit*pow(2,length-1);
				}
			}
			if (debug) printf("%i vs %i has %i\n",a[i],a[j],minimal);
			if (overAll > minimal) overAll = minimal;
		}
	}
	return overAll;
}

SNecklace CNecklace::get(int sequence)
{
    if (sequence > 0 && sequence < idLength) return idArray[sequence];
    return unknown;
}

int main(int argc,char* argv[])
{
	int minimalHamming = 1;
	if (argc > 2) minimalHamming = atoi(argv[2]); 
	CNecklace c = CNecklace(atoi(argv[1]),minimalHamming);
	int a[1000];

	int number = c.printAll(a);
	if (c.verifyHamming(a,atoi(argv[1]),number) < minimalHamming) fprintf(stderr,"Hamming distance too low!\n");
}
