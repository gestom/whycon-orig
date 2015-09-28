#ifndef CMESSAGECLIENT_H
#define CMESSAGECLIENT_H

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include "CMessage.h"
#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include "CDump.h" 

/**
@author Tom Krajnik
*/
class CMessageClient
{
public:
  CMessageClient();
  ~CMessageClient();

  int init(const char *ip,const char* port,bool requirements[]);
  int checkForData(double odo[],bool but[],int rotat[]);
  int sendMessage(CMessage* message);

private:
  int checkForInts(int data[],unsigned int len);
  int checkForBools(bool data[],unsigned int len);
  int checkForDoubles(double data[],unsigned int len);

  int mySocket;
  TLogModule module;
};

#endif
