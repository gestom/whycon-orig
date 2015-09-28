/*
 * File name: CPositionsServer.h
 * Date:      2013/06/06
 * Author:   Tom Krajnik 
 */

#ifndef __CPOSITIONSERVER_H__
#define __CPOSITIONSERVER_H__

#include <sys/types.h>
#include <stdio.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>
#include <netdb.h>

#include "CTransformation.h"
#include <semaphore.h>
#include <pthread.h>

#define MAX_CONNECTIONS 100
#define NUM_OBJECTS 100

typedef enum{
	SC_NONE,
	SC_CALIBRATE,
	SC_NUMBER
}EServerCommand;

void* serverLoop(void* serv);

class CPositionServer{
	public:

		CPositionServer();
		~CPositionServer();
		int init(const char* port);
		int updatePosition(STrackedObject object,int i,int64_t updateTime);
		int sendInfo(int socket,char *buffer);
		EServerCommand getCommand();
		int closeConnection(int socket);
		void setNumOfPatterns(int numF,int numO,int64_t frameTime);
		void finishCalibration();
		void clearToSend();
		int addConnection(int socket);
		int removeConnection(int socket);

		bool stop;		
		int serverSocket;

		int sockets[MAX_CONNECTIONS];
		int numConnections;

		/*detected object data*/
		STrackedObject object[NUM_OBJECTS];
		int64_t lastDetectionArray[NUM_OBJECTS];
		int numObjects,numFound;

		/*semaphores - */
		sem_t connectSem;
		bool debug;
		pthread_t* thread;

		/*specific for communication with the pheromone server*/
		float fieldWidth,fieldLength,cameraHeight,robotHeight,robotDiameter;
		EServerCommand command;
		bool calibration;
		bool calibrationFinished;
		int64_t updateTime;
};
#endif
/* end of CPositionServer.h */

