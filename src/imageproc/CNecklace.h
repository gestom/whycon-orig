#ifndef __CNECKLACE_H__
#define __CNECKLACE_H__

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

class CNecklace{
	public:
		CNecklace(int bits);
		~CNecklace();
		int get(int sequence);

	private:
		int length;
		int idLength; 
		int bitshift(int a);
		int *idArray;
};
#endif

