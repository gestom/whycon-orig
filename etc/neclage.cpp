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
        CNecklace(int bits);
        ~CNecklace();
        SNecklace get(int sequence);
        void printAll();


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

CNecklace::CNecklace(int bits)
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
	printf("Testing %i\n",tempID);
        do{
	    int ham = getMinimalHamming(tempID,id);
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
	    ham = getMinimalHamming(tempID,id);
	    if (minHam > ham) minHam = ham;
        //}while (rotations++<length-1 && id <= tempID && !isSymmetrical);
        }while (rotations++<length-1 && !isSymmetrical);
	if (minHam < 3) isSymmetrical = true;
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
	    printf("Adding %i\n",currentID);
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

void CNecklace::printAll(){

    int count = 0;

    for (int i = 0; i <= idLength; i++)
    {
        if(get(i).id == count){
            count++;
            printf("%i %i %i\n", get(i).id, i,get(i).hamming);
        }
    }
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
	printf("Hamming %i %i is %i\n",aa,bb,ham);
	return ham;
}

int CNecklace::getMinimalHamming(int a,int len)
{
	int minDist = 10000;
	for (int i = 1;i<len;i++){
		if (get(i).rotation == 0){
			int m = getHamming(a,i);
			if (minDist > m) minDist = m;
		}
	}
	printf("Minimal hamming of %i is %i\n",a,minDist);
	return minDist;
}

SNecklace CNecklace::get(int sequence)
{
    if (sequence > 0 && sequence < idLength) return idArray[sequence];
    return unknown;
}

int main(int argc,char* argv[])
{
    CNecklace c = CNecklace(atoi(argv[1]));

    c.printAll();

}
    
