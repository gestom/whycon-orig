#include <ros/ros.h>
#include "CTimer.h"
#include "CCircleDetect.h"
#include "CTransformation.h"
#include <image_transport/image_transport.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseStamped.h>
#include <dynamic_reconfigure/server.h>
#include <whycon_ros/whyconConfig.h>


image_transport::Publisher imdebug;
ros::Publisher command_pub;
ros::Publisher pose_pub;

CCircleDetect *detector;
CTransformation *photoTf;
CTransformation *commandTf;

SSegment currentSegment;
SSegment lastSegment;
CRawImage *grayImage;
CRawImage *depthImage;

int  defaultImageWidth= 640;
int  defaultImageHeight = 480;

int circleDetections = 0;
int maxCircleDetections = 10;
int numCommands = 5;
int lastCommand = -1;
int command = -1;
float angle = 0;
float  distanceTolerance = 0.2;
float outerDimUser = 0.05;
float outerDimMaster = 0.07;

const char *commandName[] = {
	"INFO_TERMINAL",
	"PAUSE_WALK",
	"COLLECT",
	"PATROL",
	"WAIT",
	"UNKNOWN"
};

//parameter reconfiguration
void reconfigureCallback(whycon_ros::whyconConfig &config, uint32_t level) 
{
	ROS_INFO("Reconfigure Request: %lf %lf %lf %lf %lf %lf %lf", config.userDiameter, config.masterDiameter, config.initialCircularityTolerance, config.finalCircularityTolerance, config.areaRatioTolerance,config.centerDistanceToleranceRatio,config.centerDistanceToleranceAbs);
	outerDimUser = config.userDiameter/100.0;
	outerDimMaster = config.masterDiameter/100.0;
	distanceTolerance = config.distanceTolerance/100.0;
	detector->reconfigure(config.initialCircularityTolerance, config.finalCircularityTolerance, config.areaRatioTolerance,config.centerDistanceToleranceRatio,config.centerDistanceToleranceAbs);
}


void grayImageCallback(const sensor_msgs::ImageConstPtr& msg)
{
	if (grayImage->bpp != msg->step/msg->width || grayImage->width != msg->width || grayImage->height != msg->height){
		delete grayImage;
		ROS_DEBUG("Readjusting grayImage format from %ix%i %ibpp, to %ix%i %ibpp.",grayImage->width,grayImage->height,grayImage->bpp,msg->width,msg->height,msg->step/msg->width);
		grayImage = new CRawImage(msg->width,msg->height,msg->step/msg->width);
	}

	memcpy(grayImage->data,(void*)&msg->data[0],msg->step*msg->height);

	lastSegment = currentSegment;
	currentSegment = detector->findSegment(grayImage,lastSegment);
	 
	if (currentSegment.valid)
	{
		STrackedObject o;
		if (currentSegment.bwRatio < 4.0)
		{	
			//command card
			angle = (currentSegment.angle/M_PI+1)/2*(numCommands-1);
			command = floor(angle);
			angle = fabs(angle-command-0.5);
			if (angle > 0.2) command = numCommands; else command++;
			o = commandTf->transform(currentSegment);
		}
		else
		{
			//photo card
			command = 0;
			o = photoTf->transform(currentSegment);
		}
			
		int ptr = depthImage->bpp*(((int)currentSegment.y)*depthImage->width+(int)currentSegment.x);
		float distance = 0.001*(depthImage->data[ptr]+depthImage->data[ptr+1]*255);
		geometry_msgs::PoseStamped pose;
		pose.pose.position.x = -o.y;
		pose.pose.position.y = -o.z;
		pose.pose.position.z = o.x;
		ROS_INFO("Circle detected at %.2f %.2f %.3f %.3f error %.3f - action %s %f %i!",-o.y,-o.z,o.x,distance,fabs(1-distance/o.x),commandName[command],angle,circleDetections);
		if (fabs(1-distance/o.x)<distanceTolerance){
			pose_pub.publish(pose);	
			if (command == lastCommand) circleDetections++; else circleDetections = 0;
			lastCommand = command;
		}else{
			circleDetections=0;
			command = lastCommand = -1;
		}
	} else {
		//printf("No circle visible\n");
		circleDetections=0;
		command = lastCommand = -1;
	}
	if (circleDetections == maxCircleDetections)
	{
		 //printf("Action %s. \n",commandName[command]);
		 std_msgs::String commandString;
		 commandString.data = commandName[command];
		 command_pub.publish(commandString);
	}
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "whycon_orig_ros");
	ros::NodeHandle n;
	image_transport::ImageTransport it(n);
	grayImage = new CRawImage(defaultImageWidth,defaultImageHeight,4);
	depthImage = new CRawImage(defaultImageWidth,defaultImageHeight,4);
	detector = new CCircleDetect(defaultImageWidth,defaultImageHeight);

	//initialize dynamic reconfiguration feedback
	dynamic_reconfigure::Server<whycon_ros::whyconConfig> server;
	dynamic_reconfigure::Server<whycon_ros::whyconConfig>::CallbackType dynSer;
	dynSer = boost::bind(&reconfigureCallback, _1, _2);
	server.setCallback(dynSer);


	photoTf = new CTransformation(outerDimUser);
	commandTf = new CTransformation(outerDimMaster);
	image_transport::Subscriber subimGray = it.subscribe("/head_xtion/rgb/image_mono", 1, grayImageCallback);
	//image_transport::Subscriber subimDepth = it.subscribe("/head_xtion/depth/image_raw", 1, depthImageCallback);
	command_pub = n.advertise<std_msgs::String>("/socialCardReader/commands", 1);
	pose_pub = n.advertise<geometry_msgs::PoseStamped>("/socialCardReader/cardposition", 1);

	while (ros::ok()){
		ros::spinOnce();
		usleep(30000);
	}
}

