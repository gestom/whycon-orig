#include "CMessageClient.h"

#define NETWORK_BLOCK MSG_WAITALL

CMessageClient::CMessageClient()
{
	module = LOG_MODULE_CLIENT;
}

CMessageClient::~CMessageClient()
{}

int CMessageClient::sendMessage(CMessage* message)
{
  message->pack();
  if (message->type == MSG_STOP && message->value[0] != 0) message->type = MSG_NONE;
  dump->Inform(module,"Message sending %s %i %i %i %i",message->getStrType(),message->value[0],message->value[1],message->value[2],message->value[3]);
  if (send(mySocket,message->buf,MESSAGE_LENGTH,MSG_NOSIGNAL) == MESSAGE_LENGTH) return 0; else
  {
//    dump->Error(module,"Network error\r\n");
    return -1;
  }
}

int CMessageClient::init(const char *ip,const char* port,bool requirements[])
{
  int result = -1;
  mySocket = socket(AF_INET, SOCK_STREAM, 0);
  CMessage message;
  if (mySocket > 0)
  {
    struct sockaddr_in server_addr;
    struct hostent *host_info;
    host_info =  gethostbyname(ip);
    if (host_info != NULL)
    {
      server_addr.sin_family = host_info->h_addrtype;
      memcpy((char *) &server_addr.sin_addr.s_addr,host_info->h_addr_list[0], host_info->h_length);
      server_addr.sin_port = htons(atoi(port));
      result = connect(mySocket,(struct sockaddr*) &server_addr,sizeof(server_addr));
    }
    if (result == 0)
    {
	    if (send(mySocket,requirements,4*sizeof(bool),MSG_NOSIGNAL) == 4*sizeof(bool)) return 0; else
	    {
		    fprintf(stderr,"Network error when sending info\r\n");
		    return -1;
	    }
	    fprintf(stdout,"Connection established.\r\n");
//	    message.type = MSG_START;
//	    sendMessage(&message);
    }
  }
  return result;
}

int CMessageClient::checkForInts(int data[],unsigned int len)
{
  int result;
  int lengthReceived = recv(mySocket,data,len*sizeof(int),NETWORK_BLOCK);
  if ((unsigned int)lengthReceived == len*sizeof(int)) result = 0;
  return result;
}

int CMessageClient::checkForBools(bool data[],unsigned int len)
{
  int result;
  int lengthReceived = recv(mySocket,data,len*sizeof(bool),NETWORK_BLOCK);
  if (lengthReceived == (int)(len*sizeof(bool))) result = 0;
  return result;
}

int CMessageClient::checkForDoubles(double data[],unsigned int len)
{
  int result;
  int lengthReceived = recv(mySocket,data,len*sizeof(double),NETWORK_BLOCK);
  if (lengthReceived == (int)(len*sizeof(double))) result = 0;
  return result;
}

int CMessageClient::checkForData(double odo[],bool but[],int rotat[])
{
	int result = 0;
	result +=checkForDoubles(odo,10);
	result +=checkForBools(but,10);
	result +=checkForInts(rotat,2);
	return result;
}

