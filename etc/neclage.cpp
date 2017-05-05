#include "CNecklace.h"


int main(int argc,char* argv[])
{
	int minimalHamming = 1;
	if (argc > 2) minimalHamming = atoi(argv[2]); 
	CNecklace c = CNecklace(atoi(argv[1]),minimalHamming);
	int a[1000];

	int number = c.printAll(a);
	if (c.verifyHamming(a,atoi(argv[1]),number) < minimalHamming) fprintf(stderr,"Hamming distance too low!\n");
}
