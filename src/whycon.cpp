#define _LARGEFILE_SOURCE
#define _FILE_OFFSET_BITS 64

#include <stdlib.h>
#include <string>
#include "CGui.h"
#include "CTimer.h"
#include "CCircleDetect.h"
#include "CTransformation.h"
#include <SDL/SDL.h>

// ROS libraries
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <dynamic_reconfigure/server.h>
#include <whycon_ros/whyconConfig.h>
#include <whycon_ros/MarkerArray.h>
#include <whycon_ros/Marker.h>

//default camera resolution
int  imageWidth= 640;
int  imageHeight = 480;

//default black circle diameter [m] 
float circleDiameter = 0.122;

// X and Y dimensions of the coordinate system
float fieldLength = 1.00;
float fieldWidth = 1.00;

//Max GUI dimensions 
int  screenWidth= 1920;
int  screenHeight = 1080;

/*robot detection variables*/
bool identify = false;		//identify ID of tags ?
int numBots = 0;		//num of robots to track
int numFound = 0;		//num of robots detected in the last step
int numStatic = 0;		//num of non-moving robots  
CCircleDetect *detectorArray[MAX_PATTERNS];	//detector array (each pattern has its own detector)
SSegment currentSegmentArray[MAX_PATTERNS];	//segment array (detected objects in image space)
SSegment lastSegmentArray[MAX_PATTERNS];	//segment position in the last step (allows for tracking)
STrackedObject objectArray[MAX_PATTERNS];	//object array (detected objects in metric space)
CTransformation *trans;				//allows to transform from image to metric coordinates

//circle identification
int idBits = 0;			//num of ID bits
int idSamples = 360;		//num of samples to identify ID
int hammingDist = 1;		//hamming distance of ID code

/*variables related to (auto) calibration*/
const int calibrationSteps = 20;			//how many measurements to average to estimate calibration pattern position (manual calib)
const int autoCalibrationSteps = 30; 			//how many measurements to average to estimate calibration pattern position (automatic calib)  
const int autoCalibrationPreSteps = 10;		//how many measurements to discard before starting to actually auto-calibrating (automatic calib)  
int calibNum = 5;				//number of objects acquired for calibration (5 means calibration winished inactive)
STrackedObject calib[5];			//array to store calibration patterns positions
STrackedObject calibTmp[calibrationSteps];	//array to store several measurements of a given calibration pattern
int calibStep = calibrationSteps+2;		//actual calibration step (num of measurements of the actual pattern)
bool autocalibrate = false;			//is the autocalibration in progress ?
ETransformType lastTransformType = TRANSFORM_2D;//pre-calibration transform (used to preserve pre-calibation transform type)
int wasBots = 1;				//pre-calibration number of robots to track (used to preserve pre-calibation number of robots to track)

/*program flow control*/
bool saveVideo;			//save video to output folder?
bool saveLog;			//save log to output folder?
bool stop = false;		//stop and exit ?
int moveVal = 1;		//how many frames to process ?
int moveOne = moveVal;		//how many frames to process now (setting moveOne to 0 or lower freezes the video stream) 

/*GUI-related stuff*/
CGui* gui;			//drawing, events capture
bool useGui = true;			//use graphic interface at all?
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

// communication input (camera), ROS publishers & image transport
CRawImage *image;
image_transport::Publisher imdebug;
ros::Publisher markers_pub;

// etc file paths
std::string fontPath;
std::string calibResPath;
std::string calibDefPath;

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
			trans->calibrate4D(calib,fieldLength,fieldWidth);
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

	if (keys[SDLK_m] && lastKeys[SDLK_m] == false) printf("SAVE %03f %03f %03f %03f %03f %03f %03f\n",objectArray[0].x,objectArray[0].y,objectArray[0].z,objectArray[0].error,objectArray[0].d,currentSegmentArray[0].m0,currentSegmentArray[0].m1);
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

	//camera low-level settings 
	/* will solve later
	if (keys[SDLK_c] && shiftPressed == false) camera->changeContrast(-1);
	if (keys[SDLK_c] && shiftPressed == true) camera->changeContrast(1);
	if (keys[SDLK_g] && shiftPressed == false) camera->changeGain(-1);
	if (keys[SDLK_g] && shiftPressed == true) camera->changeGain(1);
	if (keys[SDLK_e] && shiftPressed == false) camera->changeExposition(-1);
	if (keys[SDLK_e] && shiftPressed == true) camera->changeExposition(1);
	if (keys[SDLK_b] && shiftPressed == false) camera->changeBrightness(-1);
	if (keys[SDLK_b] && shiftPressed == true) camera->changeBrightness(1);
	*/

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
	for (int i = 0;i<MAX_PATTERNS;i++) detectorArray[i]->reconfigure(config.initialCircularityTolerance, config.finalCircularityTolerance, config.areaRatioTolerance,config.centerDistanceToleranceRatio,config.centerDistanceToleranceAbs, config.identify);
	/*outerDimUser = config.userDiameter/100.0;
	outerDimMaster = config.masterDiameter/100.0;
	distanceTolerance = config.distanceTolerance/100.0;*/

	fieldLength = config.fieldLength;
	fieldWidth = config.fieldWidth;
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

	/* logging will be implemented later
	int frameID =0;
	int64_t frameTime = 0;
	*/
	
	// check if readjusting of camera is needed
	if (image->bpp != msg->step/msg->width || image->width != msg->width || image->height != msg->height){
		delete image;
		ROS_INFO("Readjusting image format from %ix%i %ibpp, to %ix%i %ibpp.",image->width,image->height,image->bpp,msg->width,msg->height,msg->step/msg->width);
		image = new CRawImage(msg->width,msg->height,msg->step/msg->width);
	}

	memcpy(image->data,(void*)&msg->data[0],msg->step*msg->height);
	
	numFound = numStatic = 0;
	timer.reset();
	// logging later // frameTime = globalTimer.getRealTime();
	
	// track the robots found in the last attempt 
	for (int i = 0;i<numBots;i++){
		if (currentSegmentArray[i].valid){
			lastSegmentArray[i] = currentSegmentArray[i];
			currentSegmentArray[i] = detectorArray[i]->findSegment(image,lastSegmentArray[i]);
		}
	}

	// search for untracked (not detected in the last frame) robots 
	for (int i = 0;i<numBots;i++){
		if (currentSegmentArray[i].valid == false){
			lastSegmentArray[i].valid = false;
			currentSegmentArray[i] = detectorArray[i]->findSegment(image,lastSegmentArray[i]);
		}
		if (currentSegmentArray[i].valid == false) break;		//does not make sense to search for more patterns if the last one was not found
	}

	// perform transformations from camera to world coordinates
	for (int i = 0;i<numBots;i++){
		if (currentSegmentArray[i].valid){
			objectArray[i] = trans->transform(currentSegmentArray[i],false);
			numFound++;
			if (currentSegmentArray[i].x == lastSegmentArray[i].x) numStatic++;
		}
	}
	if(numFound > 0) ROS_INFO("Pattern detection time: %i us. Found: %i Static: %i.",globalTimer.getTime(),numFound,numStatic);
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
	}
	*/

	/* will be implemented later
	if (camera->cameraType == CT_WEBCAM){
		//for real camera, continue with capturing of another frame even if not all robots have been found
		moveOne = moveVal;
		for (int i = 0;i<numBots;i++){
		       	//printf("Frame %i Object %03i %03i %.5f %.5f %.5f \n",frameID,i,currentSegmentArray[i].ID,objectArray[i].x,objectArray[i].y,objectArray[i].yaw);
		       	if (robotPositionLog != NULL) fprintf(robotPositionLog,"Frame %i Time %ld Object %03i %03i %.5f %.5f %.5f \n",frameID,frameTime,i,currentSegmentArray[i].ID,objectArray[i].x,objectArray[i].y,objectArray[i].yaw);
		}
		if (moveVal > 0) frameID++;
	}else{
		//for postprocessing, try to find all robots before loading next frame
		if (numFound ==  numBots)
		{
			//gui->saveScreen(runs++);
			for (int i = 0;i<numBots;i++){
			       		//printf("Frame %i Object %03i %03i %.5f %.5f %.5f \n",frameID,i,currentSegmentArray[i].ID,objectArray[i].x,objectArray[i].y,objectArray[i].yaw);
					if (robotPositionLog != NULL) fprintf(robotPositionLog,"Frame %i Time %ld Object %03i %03i %.5f %.5f %.5f \n",frameID,frameTime,i,currentSegmentArray[i].ID,objectArray[i].x,objectArray[i].y,objectArray[i].yaw);
			}
			moveOne = moveVal; 
			if (moveVal > 0) frameID++;
		}else{
			if (moveOne-- < -100) moveOne = moveVal;
		}
	}
	*/

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
	calibResPath = argv[2];
	calibDefPath = argv[3];
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
	trans = new CTransformation(imageWidth,imageHeight,circleDiameter, calibResPath.c_str(), calibDefPath.c_str(), true);
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
	image_transport::Subscriber subimg = it.subscribe("/cv_camera/image_raw", 1, imageCallback);
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
