#include "CPositionServer.h"

#define NETWORK_BLOCK MSG_WAITALL

CPositionServer::CPositionServer()
{
	sem_init(&connectSem,0,1);	
	debug = true;
	numFound = numObjects = 0;
	command = SC_NONE;
	calibration = false;
	calibrationFinished = false;
	cameraHeight=fieldWidth=fieldLength=1.0;
	robotHeight=robotDiameter=0;
	stop = false;
	updateTime=0;
	numConnections=0;
	for (int i = 0;i<NUM_OBJECTS;i++)lastDetectionArray[i] = -1;
}

CPositionServer::~CPositionServer()
{
	stop = true;
	sem_wait(&connectSem);
	for (int i = 0;i<numConnections;i++) removeConnection(i);
	sem_post(&connectSem);
	close(serverSocket);
}

int CPositionServer::addConnection(int socket)
{
	sem_wait(&connectSem);
	if (numConnections < MAX_CONNECTIONS) sockets[numConnections++]=socket;
	sem_post(&connectSem);
	return numConnections;
}

int CPositionServer::removeConnection(int connid)
{
	int result = -1;
	if (connid >= 0 && connid < numConnections)
	{
		close(sockets[connid]);
		sockets[connid]=sockets[numConnections-1];
		numConnections--;
		result = numConnections;
	}
	return result;
}

void* connectLoop(void *serv)
{
	struct sockaddr_in clientAddr;
	socklen_t addrLen = sizeof(clientAddr);
	CPositionServer* server = (CPositionServer*) serv;
	int newServer = 0;
	bool debug = server->debug;
	while (server->stop == false)
	{
		newServer = accept(server->serverSocket, (struct sockaddr *)&clientAddr,&addrLen);
		if (newServer > -1){
			if (debug) fprintf(stdout,"Incoming connection from %s.",inet_ntoa(clientAddr.sin_addr));
			if (debug) fprintf(stdout,"Incoming connection accepted on socket level %i.",newServer);
			server->addConnection(newServer);
		}else{
			if (debug) fprintf(stderr,"Accept on listening socked failed.");
		}
	}
	close(server->serverSocket);
	return NULL;
}

int CPositionServer::init(const char* port)
{
	int used_port = atoi(port);
	struct sockaddr_in mySocketAddr;
	mySocketAddr.sin_family = AF_INET;
	mySocketAddr.sin_addr.s_addr = htonl(INADDR_ANY);
	mySocketAddr.sin_port = htons(used_port);
	serverSocket = socket(AF_INET, SOCK_STREAM, 0);
	if (serverSocket < 0)
	{
		if (debug) fprintf(stderr,"Cannot create socket ");
		return -1;
	}
	int yes = 1;
	if (setsockopt(serverSocket, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes)) == -1) {
		perror("setsockopt");
		exit(1);
	}
	if (setsockopt(serverSocket, IPPROTO_TCP, TCP_NODELAY, &yes, sizeof(yes)) == -1) {
		perror("setsockopt");
		exit(1);
	}
	if (bind(serverSocket,( struct sockaddr *)&mySocketAddr,sizeof(mySocketAddr)) < 0)
	{
		if (debug) fprintf(stderr,"Cannot bind socket.");
		return -2;
	}
	if (listen(serverSocket,1) < 0)
	{
		if (debug) fprintf(stderr,"cannot make socket listen.");
	}
	thread=(pthread_t*)malloc(sizeof(pthread_t));
	pthread_create(thread,NULL,&connectLoop,(void*)this);
	return 0;
}

int CPositionServer::sendInfo(int connid,char* buffer)
{
	if (connid <0 && connid >= numConnections) return -1;
	int socket = sockets[connid];

	if (send(socket,buffer,strlen(buffer),MSG_NOSIGNAL) != (int)strlen(buffer)) {
		fprintf(stderr,"Could not send data - assuming that client disconnected. \n");
		removeConnection(connid);
	}

	/*check for incoming commands - specific for communitation with the artificial pheromone system*/
	int numBytes = 0;
	ioctl(socket,FIONREAD,&numBytes);
	if (numBytes > 0){
		char data[numBytes];
		memset(data,0,numBytes);
		int lengthReceived = recv(socket,data,numBytes,NETWORK_BLOCK);
		if (lengthReceived > 0){
			char *token;
			token = strtok(data, "\n");
			while( token != NULL ) 
			{
				if (strncmp(token,"Calibrate",9)==0)
				{
					sscanf(token,"Calibrate %i %f %f %f %f %f\n",&numObjects,&fieldLength,&fieldWidth,&cameraHeight,&robotDiameter,&robotHeight);
					printf("Calibrate command received\n");
					command = SC_CALIBRATE;
				}
				token = strtok(NULL, "\n");
			}
		}
	}
	return 0;
}

void CPositionServer::clearToSend()
{
	char buffer[10000];

	/*send robot positions*/
	sprintf(buffer,"Detected %i of %i at %ld. \n",numFound,numObjects,updateTime);
	STrackedObject o;
	for (int i=0;i<numObjects;i++){
		o=object[i];
		sprintf(buffer,"%sRobot %03i %.3f %.3f %.3f %ld \n",buffer,o.ID,o.x,o.y,o.yaw*180/M_PI,lastDetectionArray[i]);
	}
	if (calibrationFinished)
	{
		sprintf(buffer,"%sCalibrated\n",buffer);
		calibrationFinished = false;
	}
	sem_wait(&connectSem);
	for (int i = 0;i<numConnections;i++) sendInfo(i,buffer);
	sem_post(&connectSem);
}

EServerCommand CPositionServer::getCommand()
{
	EServerCommand result = command;
	command = SC_NONE;
	return result;
}

void CPositionServer::finishCalibration()
{
	calibrationFinished = true;
}

void CPositionServer::setNumOfPatterns(int numF,int numO,int64_t updateTim)
{
		numFound = numF;
		numObjects = numO;
		updateTime = updateTim;
		if (numObjects > NUM_OBJECTS) numObjects = NUM_OBJECTS;
		if (numObjects < 0) numObjects = 0;
}

int CPositionServer::updatePosition(STrackedObject o,int num,int64_t updateTime)
{
	if (num < numObjects && num >= 0){
		object[num] = o;
		lastDetectionArray[num] = updateTime;
	}
	return 0;
}

int CPositionServer::closeConnection(int socket)
{
	close(socket);
	return 0;
}

