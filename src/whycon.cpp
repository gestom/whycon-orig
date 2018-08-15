#define _LARGEFILE_SOURCE
#define _FILE_OFFSET_BITS 64

#include <stdlib.h>
#include <string>
#include <SDL/SDL.h>
#include <opencv2/opencv.hpp>
#include "CGui.h"
#include "CTimer.h"
#include "CCircleDetect.h"
#include "CTransformation.h"

// ROS libraries
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <dynamic_reconfigure/server.h>
#include <whycon_ros/whyconConfig.h>
#include <whycon_ros/MarkerArray.h>
#include <whycon_ros/Marker.h>

using namespace cv;

int  imageWidth = 640;		//default camera resolution
int  imageHeight = 480;		//default camera resolution
float circleDiameter = 0.122;	//default black circle diameter [m];
float fieldLength = 1.00;	//X dimension of the coordinate system
float fieldWidth = 1.00;	//Y dimension of the coordinate system
int  screenWidth= 1920;		//max GUI width
int  screenHeight = 1080;	//max GUI height

/*robot detection variables*/
bool identify = false;		//identify ID of tags ?
int numBots = 0;		//num of robots to track
int numFound = 0;		//num of robots detected in the last step
int numStatic = 0;		//num of non-moving robots  
CCircleDetect *detectorArray[MAX_PATTERNS];	//detector array (each pattern has its own detector)
SSegment currentSegmentArray[MAX_PATTERNS];	//segment array (detected objects in image space)
SSegment lastSegmentArray[MAX_PATTERNS];	//segment position in the last step (allows for tracking)

SSegment currInnerSegArr[MAX_PATTERNS];
STrackedObject objInnerArr[MAX_PATTERNS];

STrackedObject objectArray[MAX_PATTERNS];	//object array (detected objects in metric space)
CTransformation *trans;				//allows to transform from image to metric coordinates

//circle identification
int idBits = 0;			//num of ID bits
int idSamples = 360;		//num of samples to identify ID
int hammingDist = 1;		//hamming distance of ID code

/*variables related to (auto) calibration*/
const int calibrationSteps = 20;			//how many measurements to average to estimate calibration pattern position (manual calib)
const int autoCalibrationSteps = 30; 			//how many measurements to average to estimate calibration pattern position (automatic calib)  
const int autoCalibrationPreSteps = 10;			//how many measurements to discard before starting to actually auto-calibrating (automatic calib)  
int calibNum = 5;					//number of objects acquired for calibration (5 means calibration winished inactive)
STrackedObject calib[5];				//array to store calibration patterns positions
STrackedObject calibTmp[calibrationSteps];		//array to store several measurements of a given calibration pattern
int calibStep = calibrationSteps+2;			//actual calibration step (num of measurements of the actual pattern)
bool autocalibrate = false;				//is the autocalibration in progress ?
ETransformType lastTransformType = TRANSFORM_2D;	//pre-calibration transform (used to preserve pre-calibation transform type)
int wasBots = 1;					//pre-calibration number of robots to track (used to preserve pre-calibation number of robots to track)

/*program flow control*/
bool saveVideo = false;		//save video to output folder?
bool saveLog = false;		//save log to output folder?
bool stop = false;		//stop and exit ?
int moveVal = 1;		//how many frames to process ?
int moveOne = moveVal;		//how many frames to process now (setting moveOne to 0 or lower freezes the video stream) 

/*GUI-related stuff*/
CGui* gui;			//drawing, events capture
bool useGui = true;		//use graphic interface at all?
int guiScale = 1;		//in case camera resolution exceeds screen one, gui is scaled down
SDL_Event event;		//store mouse and keyboard events
int keyNumber = 10000;		//number of keys pressed in the last step	
Uint8 lastKeys[1000];		//keys pressed in the previous step
Uint8 *keys = NULL;		//pressed keys
bool displayHelp = false;	//displays some usage hints
bool drawCoords = true;		//draws coordinatess at the robot's positions
int runs = 0;			//number of gui updates/detections performed 
int evalTime = 0;		//time required to detect the patterns
FILE *robotPositionLog = NULL;	//file to log robot positions

// communication input (camera), ROS publishers
CRawImage *image;
ros::Publisher markers_pub;

// etc file paths
std::string fontPath;
std::string calibResPath;
std::string calibDefPath;

// intrisic and distortion params from camera_info
Mat distCoeffs = Mat(1,5, CV_32FC1);	
Mat intrinsic = Mat(3,3, CV_32FC1);

/*manual calibration can be initiated by pressing 'r' and then clicking circles at four positions (0,0)(fieldLength,0)...*/
void manualcalibration()
{
	if (currentSegmentArray[0].valid){
		STrackedObject o = objectArray[0];
		moveOne = moveVal;

		//object found - add to a buffer
		if (calibStep < calibrationSteps) calibTmp[calibStep++] = o;

		//does the buffer contain enough data to calculate the object position
		if (calibStep == calibrationSteps){
			o.x = o.y = o.z = 0;
			for (int k = 0;k<calibrationSteps;k++){
				o.x += calibTmp[k].x;
				o.y += calibTmp[k].y;
				o.z += calibTmp[k].z;
			}
			o.x = o.x/calibrationSteps;	
			o.y = o.y/calibrationSteps;	
			o.z = o.z/calibrationSteps;
			if (calibNum < 4){
				calib[calibNum++] = o;
			}

			//was it the last object needed to establish the transform ?
			if (calibNum == 4){
				//calculate and save transforms
				trans->calibrate2D(calib,fieldLength,fieldWidth);
				trans->calibrate3D(calib,fieldLength,fieldWidth);
				calibNum++;
				numBots = wasBots;
				trans->saveCalibration(calibDefPath.c_str());
				trans->transformType = lastTransformType;
				detectorArray[0]->localSearch = false;
			}
			calibStep++;
		}
	}
}

/*finds four outermost circles and uses them to set-up the coordinate system - [0,0] is left-top, [0,fieldLength] next in clockwise direction*/
void autocalibration()
{
	bool saveVals = true;
	for (int i = 0;i<numBots;i++){
		if (detectorArray[i]->lastTrackOK == false) saveVals=false;
	}
	if (saveVals){
		int index[] = {0,0,0,0};	
		int maxEval = 0;
		int eval = 0;
		int sX[] = {-1,+1,-1,+1};
		int sY[] = {+1,+1,-1,-1};
		for (int b = 0;b<4;b++){
			maxEval = -10000000;
			for (int i = 0;i<numBots;i++){
				eval = 	sX[b]*currentSegmentArray[i].x +sY[b]*currentSegmentArray[i].y;
				if (eval > maxEval){
					maxEval = eval;
					index[b] = i;
				}
			}
		}
		printf("INDEX: %i %i %i %i\n",index[0],index[1],index[2],index[3]);
		for (int i = 0;i<4;i++){
			if (calibStep <= autoCalibrationPreSteps) calib[i].x = calib[i].y = calib[i].z = 0;
			calib[i].x+=objectArray[index[i]].x;
			calib[i].y+=objectArray[index[i]].y;
			calib[i].z+=objectArray[index[i]].z;
		}
		calibStep++;
		if (calibStep == autoCalibrationSteps){
			for (int i = 0;i<4;i++){
				calib[i].x = calib[i].x/(autoCalibrationSteps-autoCalibrationPreSteps);
				calib[i].y = calib[i].y/(autoCalibrationSteps-autoCalibrationPreSteps);
				calib[i].z = calib[i].z/(autoCalibrationSteps-autoCalibrationPreSteps);
			}
			trans->calibrate2D(calib,fieldLength,fieldWidth);
			trans->calibrate3D(calib,fieldLength,fieldWidth);
			calibNum++;
			numBots = wasBots;
			trans->saveCalibration(calibDefPath.c_str());
			trans->transformType = lastTransformType;
			autocalibrate = false;
		}
	}
}

/*initialize logging*/
bool initializeLogging()
{
	char logFileName[1000];
	char timeStr[100];
	time_t timeNow;
	time(&timeNow);
	strftime(timeStr, sizeof(timeStr), "%Y-%m-%d_%H-%M-%S",localtime(&timeNow));
	sprintf(logFileName,"output/WhyCon_%s.txt",timeStr);
	robotPositionLog = fopen(logFileName,"w");
	if (robotPositionLog == NULL){
		fprintf(stderr,"Cannot write to log file %s. Does the \"output\" directory exist?\n",logFileName);
		return false;
	}
	return true;
}

/*process events coming from GUI*/
void processKeys()
{
	//process mouse - mainly for manual calibration - by clicking four circles at the corners of the operational area 
	while (SDL_PollEvent(&event)){
		if (event.type == SDL_MOUSEBUTTONDOWN){
			if (calibNum < 4 && calibStep > calibrationSteps){
				 calibStep = 0;
				 trans->transformType = TRANSFORM_NONE;
			}
			if (numBots > 0){
				currentSegmentArray[numBots-1].x = event.motion.x*guiScale; 
				currentSegmentArray[numBots-1].y = event.motion.y*guiScale;
				currentSegmentArray[numBots-1].valid = true;
				detectorArray[numBots-1]->localSearch = true;
			}
		}
	}

	//process keys 
	keys = SDL_GetKeyState(&keyNumber);
	bool shiftPressed = keys[SDLK_RSHIFT] || keys[SDLK_LSHIFT];
 
	//program control - (s)top, (p)ause+move one frame and resume
	if (keys[SDLK_ESCAPE]) stop = true;
	if (keys[SDLK_SPACE] && lastKeys[SDLK_SPACE] == false){ moveOne = 100000000; moveVal = 10000000;};
	if (keys[SDLK_p] && lastKeys[SDLK_p] == false) {moveOne = 1; moveVal = 0;}

	if (keys[SDLK_m] && lastKeys[SDLK_m] == false) printf("SAVE %03f %03f %03f %03f %03f %03f\n",objectArray[0].x,objectArray[0].y,objectArray[0].z,objectArray[0].d,currentSegmentArray[0].m0,currentSegmentArray[0].m1);
	if (keys[SDLK_n] && lastKeys[SDLK_n] == false) printf("SEGM %03f %03f %03f\n",currentSegmentArray[0].x,currentSegmentArray[0].y,currentSegmentArray[0].m0);
	if (keys[SDLK_s] && lastKeys[SDLK_s] == false) image->saveBmp();

	//initiate autocalibration (searches for 4 outermost circular patterns and uses them to establisht the coordinate system)
	if (keys[SDLK_a] && lastKeys[SDLK_a] == false) { calibStep = 0; lastTransformType=trans->transformType; wasBots = numBots; autocalibrate = true;trans->transformType=TRANSFORM_NONE;}; 

	//manual calibration (click the 4 calibration circles with mouse)
	if (keys[SDLK_r] && lastKeys[SDLK_r] == false) { calibNum = 0; wasBots=numBots; numBots = 1;}

	//debugging - toggle drawing coordinates and debugging results results
	if (keys[SDLK_l] && lastKeys[SDLK_l] == false) drawCoords = drawCoords == false;
	if (keys[SDLK_v] && lastKeys[SDLK_v] == false) for (int i = 0;i<numBots;i++) detectorArray[i]->drawAll = detectorArray[i]->drawAll==false;
	if (keys[SDLK_d] && lastKeys[SDLK_d] == false)
	{ 
			for (int i = 0;i<numBots;i++){
			detectorArray[i]->draw = detectorArray[i]->draw==false;
			detectorArray[i]->debug = detectorArray[i]->debug==false;
		}
	}

	//transformations to use - in our case, the relevant transform is '2D'
	if (keys[SDLK_1] && lastKeys[SDLK_1] == false) trans->transformType = TRANSFORM_NONE;
	if (keys[SDLK_2] && lastKeys[SDLK_2] == false) trans->transformType = TRANSFORM_2D;
	if (keys[SDLK_3] && lastKeys[SDLK_3] == false) trans->transformType = TRANSFORM_3D;

	//todo camera low-level settings 

	//display help
	if (keys[SDLK_h] && lastKeys[SDLK_h] == false) displayHelp = displayHelp == false; 

	//adjust the number of robots to be searched for
	if (keys[SDLK_PLUS]) numBots++;
	if (keys[SDLK_EQUALS]) numBots++;
	if (keys[SDLK_MINUS]) numBots--;
	if (keys[SDLK_KP_PLUS]) numBots++;
	if (keys[SDLK_KP_MINUS]) numBots--;

	//store the key states
	memcpy(lastKeys,keys,keyNumber);
}

// dynamic parameter reconfiguration
void reconfigureCallback(whycon_ros::whyconConfig &config, uint32_t level) 
{
	ROS_INFO("Reconfigure Request: %d %lf %d %lf %lf %lf %lf %lf", config.numBots, config.circleDiameter, config.identify, config.initialCircularityTolerance, config.finalCircularityTolerance, config.areaRatioTolerance,config.centerDistanceToleranceRatio,config.centerDistanceToleranceAbs);
	numBots = (config.numBots > MAX_PATTERNS) ? MAX_PATTERNS : config.numBots;
	trans->reconfigure(config.circleDiameter);
	for (int i = 0;i<MAX_PATTERNS;i++) detectorArray[i]->reconfigure(config.initialCircularityTolerance, config.finalCircularityTolerance, config.areaRatioTolerance,config.centerDistanceToleranceRatio,config.centerDistanceToleranceAbs, config.identify, config.minSize);
	fieldLength = config.fieldLength;
	fieldWidth = config.fieldWidth;
}

void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& msg)
{
	if(msg->K[0] != intrinsic.at<float>(0,0) || msg->K[2] != intrinsic.at<float>(0,2) || msg->K[4] != intrinsic.at<float>(1,1) ||  msg->K[5] != intrinsic.at<float>(1,2)){
		for(int i = 0; i < 5; i++) distCoeffs.at<float>(i) = msg->D[i];
		int tmpIdx = 0;
		for(int i = 0; i < 3; i++){
			for(int j = 0; j < 3; j++){
				intrinsic.at<float>(i, j) = msg->K[tmpIdx++];
			}
		}
		trans->updateParams(intrinsic, distCoeffs);
	}
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
	//setup timers to assess system performance
	CTimer timer;
	timer.reset();
	timer.start();

	CTimer globalTimer;
	globalTimer.reset();
	globalTimer.start();

	// check if readjusting of camera is needed
	if (image->bpp != msg->step/msg->width || image->width != msg->width || image->height != msg->height){
		delete image;
		ROS_INFO("Readjusting image format from %ix%i %ibpp, to %ix%i %ibpp.",image->width,image->height,image->bpp,msg->width,msg->height,msg->step/msg->width);
		image = new CRawImage(msg->width,msg->height,msg->step/msg->width);
	}

	memcpy(image->data,(void*)&msg->data[0],msg->step*msg->height);
	
	numFound = numStatic = 0;
	timer.reset();

	// track the robots found in the last attempt 
	for (int i = 0;i<numBots;i++){
		if (currentSegmentArray[i].valid){
			lastSegmentArray[i] = currentSegmentArray[i];
			currentSegmentArray[i] = detectorArray[i]->findSegment(image,lastSegmentArray[i]);
			currInnerSegArr[i] = detectorArray[i]->getInnerSegment();
		}
	}

	// search for untracked (not detected in the last frame) robots 
	for (int i = 0;i<numBots;i++){
		if (currentSegmentArray[i].valid == false){
			lastSegmentArray[i].valid = false;
			currentSegmentArray[i] = detectorArray[i]->findSegment(image,lastSegmentArray[i]);
			currInnerSegArr[i] = detectorArray[i]->getInnerSegment();
		}
		if (currentSegmentArray[i].valid == false) break;		//does not make sense to search for more patterns if the last one was not found
	}

	// perform transformations from camera to world coordinates
	for (int i = 0;i<numBots;i++){
		if (currentSegmentArray[i].valid){
			int step = image->bpp;
			int pos;
			pos = ((int)currentSegmentArray[i].x+((int)currentSegmentArray[i].y)*image->width);
		        image->data[step*pos+0] = 255;
		        image->data[step*pos+1] = 0;
		        image->data[step*pos+2] = 0;
		        pos = ((int)currInnerSegArr[i].x+((int)currInnerSegArr[i].y)*image->width);
		        image->data[step*pos+0] = 0;
		        image->data[step*pos+1] = 255;
		        image->data[step*pos+2] = 0;

			objectArray[i] = trans->transform(currentSegmentArray[i]);
			objInnerArr[i] = trans->transform(currInnerSegArr[i]);
			
			float newX0 = -objectArray[i].y;
			float newY0 = -objectArray[i].z;
			float newZ0 = objectArray[i].x;
			float newX1 = -objInnerArr[i].y;
			float newY1 = -objInnerArr[i].z;
			float newZ1 = objInnerArr[i].x;
		
			trans->reTransformXY(&newX0, &newY0, &newZ0);
			trans->reTransformXY(&newX1, &newY1, &newZ1);
			
			pos = ((int)newX0+((int)newY0*image->width));
			if (pos > 0 && pos < image->width*image->height){
				image->data[step*pos+0] = 255;
				image->data[step*pos+1] = 0;
				image->data[step*pos+2] = 0;
			};
			pos = ((int)newX1+((int)newY1*image->width));
			if (pos > 0 && pos < image->width*image->height){
				image->data[step*pos+0] = 0;
				image->data[step*pos+1] = 255;
				image->data[step*pos+2] = 0;
			};

			float outerDist = sqrt((newX0-currentSegmentArray[i].x)*(newX0-currentSegmentArray[i].x)+(newY0-currentSegmentArray[i].y)*(newY0-currentSegmentArray[i].y));
			float innerDist = sqrt((newX1-currInnerSegArr[i].x)*(newX1-currInnerSegArr[i].x)+(newY1-currInnerSegArr[i].y)*(newY1-currInnerSegArr[i].y));
			float outerCenter0 = sqrt(currentSegmentArray[i].x*currentSegmentArray[i].x+currentSegmentArray[i].y*currentSegmentArray[i].y);
			float outerCenter1 = sqrt(newX0*newX0+newY0*newY0);
			float innerCenter0 = sqrt(currInnerSegArr[i].x*currInnerSegArr[i].x+currInnerSegArr[i].y*currInnerSegArr[i].y);
			float innerCenter1 = sqrt(newX1*newX1+newY1*newY1);
			printf("o %03.5f i %03.5f %03.5f\n",outerDist,innerDist,outerDist-innerDist);
			printf("image coords dist\n");
			printf("o %03.5f i %03.5f %03.5f\n",outerCenter0,innerCenter0,outerCenter0-innerCenter0);
			printf("o %03.5f i %03.5f %03.5f\n",outerCenter1,innerCenter1,outerCenter1-innerCenter1);
			printf("o %03.5f %03.5f %03.5f\n",outerCenter0,outerCenter1,outerCenter0-outerCenter1);
			printf("i %03.5f %03.5f %03.5f\n",innerCenter0,innerCenter1,innerCenter0-innerCenter1);
		
			float xi0 = currentSegmentArray[i].x;
			float yi0 = currentSegmentArray[i].y;
			float xi1 = currInnerSegArr[i].x;
			float yi1 = currInnerSegArr[i].y;
			trans->transformXY(&xi0,&yi0);
			trans->transformXY(&xi1,&yi1);
			trans->transformXY(&newX0,&newY0);
			trans->transformXY(&newX1,&newY1);
			float oDist0 = sqrt(xi0*xi0+yi0*yi0);
			float oDist1 = sqrt(newX0*newX0+newY0*newY0);
			float iDist0 = sqrt(xi1*xi1+yi1*yi1);
			float iDist1 = sqrt(newX1*newX1+newY1*newY1);
			printf("camera coords dist\n");
			printf("o %03.5f i %03.5f %03.5f\n",oDist0,iDist0,oDist0-iDist0);
			printf("o %03.5f i %03.5f %03.5f\n",oDist1,iDist1,oDist1-iDist1);
			printf("o %03.5f %03.5f %03.5f\n",oDist0,oDist1,oDist0-oDist1);
			printf("i %03.5f %03.5f %03.5f\n",iDist0,iDist1,iDist0-iDist1);
			printf("\n");

			numFound++;
			if (currentSegmentArray[i].x == lastSegmentArray[i].x) numStatic++;
		}
	}
//	if(numFound > 0) ROS_INFO("Pattern detection time: %i us. Found: %i Static: %i.",globalTimer.getTime(),numFound,numStatic);
	evalTime = timer.getTime();

	// publishing information about tags 
	whycon_ros::MarkerArray markerArray;
	markerArray.header = msg->header;

	for (int i = 0;i<numBots && useGui && drawCoords;i++){
		if (currentSegmentArray[i].valid){
			//printf("ID %d\n", currentSegmentArray[i].ID);
			whycon_ros::Marker marker;
	
			marker.id = currentSegmentArray[i].ID;
			marker.u = currentSegmentArray[i].x;
			marker.v = currentSegmentArray[i].y;
			marker.size = currentSegmentArray[i].size;
			
			// Convert to ROS standard Coordinate System
			marker.position.position.x = -objectArray[i].y;
			marker.position.position.y = -objectArray[i].z;
			marker.position.position.z = objectArray[i].x;

			marker.rotation.x = objectArray[i].pitch;
			marker.rotation.y = objectArray[i].roll;
			marker.rotation.z = objectArray[i].yaw;

			markerArray.markers.push_back(marker);
		}
	}

	if(markerArray.markers.size() > 0) markers_pub.publish(markerArray);

	//draw stuff on the GUI 
	if (useGui){
		gui->drawImage(image);
		gui->drawTimeStats(evalTime,numBots);
		gui->displayHelp(displayHelp);
		gui->guideCalibration(calibNum,fieldLength,fieldWidth);
	}
	for (int i = 0;i<numBots && useGui && drawCoords;i++){
		if (currentSegmentArray[i].valid) gui->drawStats(currentSegmentArray[i].minx-30,currentSegmentArray[i].maxy,objectArray[i],trans->transformType == TRANSFORM_2D);
	}

	//establishing the coordinate system by manual or autocalibration
	if (autocalibrate && numFound == numBots) autocalibration();
	if (calibNum < 4) manualcalibration();

	/* empty for-cycle that isn't used even in master orig version
	for (int i = 0;i<numBots;i++){
		//if (currentSegmentArray[i].valid) printf("Object %i %03f %03f %03f %03f %03f\n",i,objectArray[i].x,objectArray[i].y,objectArray[i].z,objectArray[i].error,objectArray[i].esterror);
	}*/

	//gui->saveScreen(runs);
	if (useGui) gui->update();
	if (useGui) processKeys();
}

int main(int argc,char* argv[])
{
	// initialization of image transport, img processing, and ROS
	ros::init(argc, argv, "whycon_ros");
	ros::NodeHandle n("~");
	image_transport::ImageTransport it(n);
	image = new CRawImage(imageWidth,imageHeight, 3);
	
	// loading params and args from launch file
	fontPath = argv[1];
	calibDefPath = argv[2];
	n.param("useGui", useGui, true);
	n.param("saveLog", saveLog, false);
	n.param("saveVideo", saveVideo, false);
	n.param("idBits", idBits, 5);
	n.param("idSamples", idSamples, 360);
	n.param("hammingDist", hammingDist, 1);
	
	if (saveLog) initializeLogging();
	
	moveOne = moveVal;
	moveOne  = 0;

	// determine gui size so that it fits the screen
	while (imageHeight/guiScale > screenHeight || imageHeight/guiScale > screenWidth) guiScale = guiScale*2;

	// initialize GUI, image structures, coordinate transformation modules
	if (useGui) gui = new CGui(imageWidth,imageHeight,guiScale, fontPath.c_str());
	trans = new CTransformation(imageWidth,imageHeight,circleDiameter, calibDefPath.c_str());
	trans->transformType = TRANSFORM_NONE;		//in our case, 2D is the default

	// initialize the circle detectors - each circle has its own detector instance 
	for (int i = 0;i<MAX_PATTERNS;i++) detectorArray[i] = new CCircleDetect(imageWidth,imageHeight,identify, idBits, idSamples, hammingDist);
	image->getSaveNumber();

	// initialize dynamic reconfiguration feedback
	dynamic_reconfigure::Server<whycon_ros::whyconConfig> server;
	dynamic_reconfigure::Server<whycon_ros::whyconConfig>::CallbackType dynSer;
	dynSer = boost::bind(&reconfigureCallback, _1, _2);
	server.setCallback(dynSer);

	// subscribe to camera topic, publish topis with card position, rotation and ID
	ros::Subscriber subInfo = n.subscribe("/camera/camera_info", 1, cameraInfoCallback);
	image_transport::Subscriber subImg = it.subscribe("/camera/image_raw", 1, imageCallback);
	markers_pub = n.advertise<whycon_ros::MarkerArray>("/whycon_ros/markers", 1);

	// ROS infinite loop that refreshes data in topics, checks if stop signal was sent
	while (ros::ok()){
		ros::spinOnce();
		usleep(30000);
		if(stop) break;
	}

	// cleaning up
	if (robotPositionLog != NULL) fclose(robotPositionLog);
	delete image;
	if (useGui) delete gui;
	for (int i = 0;i<MAX_PATTERNS;i++) delete detectorArray[i];
	delete trans;
	
	return 0;
}
