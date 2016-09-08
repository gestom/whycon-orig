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
		int cached [length - 1];
		bool isSymmetrical = false;

		do{
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

		}while (rotations++<length-1 && id <= tempID && !isSymmetrical);

		if(isSymmetrical)
		{
			idArray[id].id = -1;
			idArray[id].rotation = -1;
		} else if (id > tempID)
		{
			if(idArray[tempID].id != -1)
			{
				idArray[id] = idArray[tempID];
				idArray[id].rotation +=rotations;
			}
		}else
		{
			idArray[id].id =currentID++;
			idArray[id].rotation = 0;
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

SNecklace CNecklace::get(int sequence)
{
	if (sequence > 0 && sequence < idLength) return idArray[sequence];
	return unknown;
}