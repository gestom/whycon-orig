#include "CNecklace.h"

CNecklace::CNecklace(int bits)
{
	length = bits;
	idLength = pow(2,length); 
	idArray = (SNecklace*)malloc(sizeof(SNecklace)*idLength);

	int currentID = 0;
	int tempID,bit,rotations;

	/*for every possible id*/
	for (int id = 0;id<idLength;id++)
	{
		/*check if there is a lower number that could be created by bitshifting it*/
		tempID  = id;
		rotations = 0;
		do{
			bit = tempID%2;
			tempID=tempID/2+bit*pow(2,length-1);
		//	printf("%i %i %i\n",tempID,id,bit);
		}
		while (rotations++<length-1 && id <= tempID);
		if (id > tempID){
				idArray[id] = idArray[tempID];
				idArray[id].rotation += rotations;
		}else{
			idArray[id].id =currentID++;
			idArray[id].rotation = 0;
		}
		printf("IDD: %i %i %i\n",id,idArray[id].id,idArray[id].rotation);
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

SNecklace CNecklace::get(int sequence)
{
	if (sequence > 0 && sequence < idLength) return idArray[sequence];
	return unknown;
}
