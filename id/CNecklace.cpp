#include "CNecklace.h"

CNecklace::CNecklace(int bits,int minimalHamming = 1)
{
    length = bits;
    idLength = pow(2,length); 
    idArray = (SNecklace*)malloc(sizeof(SNecklace)*idLength);

    int currentID = 0;
    int tempID,bit,rotations;
    int minHam = 1000;
    int hamindex = 1000;
    int ham = 1000;
    debug = false;

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
            hamindex = getMinimalHamming(tempID,id);
            ham = getHamming(tempID,hamindex);
            if (minHam > ham){
                minHam = ham;
                if (minHam == 0){
                    idArray[id].id = idArray[hamindex].id; 
                    idArray[id].rotation = idArray[hamindex].rotation + rotations; 
                    if (debug) printf("AAA %i %i %i \n",id,idArray[id].id,idArray[id].rotation);
                }
            }
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
        }while (rotations++<length-1 && !isSymmetrical);
        //		if (minHam < minimalHamming) isSymmetrical = true;
        /*else if (id > tempID)
          {
          if(idArray[tempID].id != -1)
          {
          idArray[id] = idArray[tempID];
          idArray[id].rotation +=rotations;
          idArray[id].rotation = 1;
          }
          }
          else*/
        if (minHam >= minimalHamming && !isSymmetrical)
        {
            if (debug) printf("Adding %i %i\n",currentID,id);
            idArray[id].id =currentID++;
            idArray[id].rotation = 0;
            idArray[id].hamming = minHam;
        }
        else if (minHam > 0)
        {
            idArray[id].id = -1; 
            idArray[id].rotation = -1;
            idArray[id].hamming = minHam;
        }
        if(isSymmetrical)
        {
            idArray[id].id = -1;
            idArray[id].rotation = -1;
        } 
    }

    //idArray[idLength-1].id = 0;
    //idArray[idLength-1].rotation = 0;
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
        if(get(i).rotation == 0){
            a[count++] = i;
            //            printf("%i %i\n", get(i).id, i);
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
    int mindex = 10000;
    for (int i = 1;i<len;i++){
        if (get(i).rotation == 0){
            int m = getHamming(a,i);
            if (minDist > m){
                minDist = m;
                mindex = i;
                //if (minDist < 3) printf("%i is same as %i\n",a,i);
            }

        }
    }
    if (debug) printf("Minimal hamming of %i is %i\n",a,minDist);
    return mindex;
    //return minDist;
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
