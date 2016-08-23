#ifndef __CNECKLACE_H__
#define __CNECKLACE_H__

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

typedef struct
{
	int id;
	int rotation;
}SNecklace;

class CNecklace{
	public:
		CNecklace(int bits);
		~CNecklace();
		SNecklace get(int sequence);

	private:
		int length;
		int idLength; 
		SNecklace *idArray;
		SNecklace unknown;
};
#endif

