#include <stdio.h>
#include <stdlib.h>
#include <math.h>

int bitshift(int a,int len)
{
	int num  = a;
	for (int i = 0;i<len;i++){
		int bit = num%2;
		num=num/2+bit*pow(2,len);
		//printf("BB: %i %i %i\n",a,bit,num);
		if (a > num) return num;
	}
	return a;
}

int main(int argc,char* argv[])
{
	int len = atoi(argv[1]);
	int idLength = pow(2,len); 
	int id[idLength];
	int currentID = 0;
	for (int i = 0;i<idLength;i++)
	{
		int idsm = bitshift(i,len-1);
		if (idsm == i){
		       	id[i]=currentID++;
			printf("%i %i\n",id[i],i);
		}else 	id[i] = id[idsm];
	}
//	for (int i = 0;i<idLength;i++) printf("%i %i\n",i,id[i]);
	return 0;
}
