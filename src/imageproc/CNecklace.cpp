#include "CNecklace.h"

CNecklace::CNecklace(int bits)
{
	length = bits;
	idLength = pow(2,length); 
	idArray = (int*)malloc(sizeof(int)*idLength);

	int currentID = 0;
	for (int i = 0;i<idLength;i++)
	{
		int idsm = bitshift(i);
		if (idsm == i){
		       	idArray[i]=currentID++;
			printf("%i %i\n",idArray[i],i);
		}else 	idArray[i] = idArray[idsm];
	}
}

CNecklace::~CNecklace()
{
	free(idArray);
}


int CNecklace::bitshift(int a)
{
	int num  = a;
	for (int i = 0;i<length-1;i++){
		int bit = num%2;
		num=num/2+bit*pow(2,length-1);
		//printf("BB: %i %i %i\n",a,bit,num);
		if (a > num) return num;
	}
	return a;
}

int CNecklace::get(int sequence)
{
	if (sequence > 0 && sequence < idLength) return idArray[sequence];
	return -1;
}
