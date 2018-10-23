#ifndef __CWhycon_H__                                                              
#define __CWhycon_H__

#define _LARGEFILE_SOURCE                                                          
#define _FILE_OFFSET_BITS 64

#include <stdlib.h>
#include <string>
#include <opencv2/opencv.hpp>
#include <SDL/SDL.h>

// WhyCon libs
#include "CGui.h"
#include "CTimer.h"
#include "CCircleDetect.h"
#include "CTransformation.h"
#include "CNecklace.h"

// ROS libraries
#include <ros/ros.h>
#include <tf/tf.h>
#include <image_transport/image_transport.h>
#include <dynamic_reconfigure/server.h>
#include <whycon_ros/whyconConfig.h>
#include <whycon_ros/MarkerArray.h>
#include <whycon_ros/Marker.h>

using namespace cv;


class CWhycon {

    public:

        int imageWidth;         // default camera resolution
        int imageHeight;        // default camera resolution
        float circleDiameter;   // default black circle diameter [m];
        float fieldLength;      // X dimension of the coordinate system
        float fieldWidth;       // Y dimension of the coordinate system

        /*robot detection variables*/
        bool identify;          // whether to identify ID
        int numBots;            // num of robots to track
        int numFound;           // num of robots detected in the last step
        int numStatic;          // num of non-moving robots
        int maxPatterns;        // maximum number of patterns

        //circle identification
        int idBits;             // num of ID bits
        int idSamples;          // num of samples to identify ID
        int hammingDist;        // hamming distance of ID code

        /*program flow control*/
        //bool saveVideo = false;   //save video to output folder?
        bool stop;          // stop and exit ?
        int moveVal;        // how many frames to process ?
        int moveOne;        // how many frames to process now (setting moveOne to 0 or lower freezes the video stream)

        CWhycon();      // constructor sets up essential variables
        ~CWhycon();     // destructor
        void init(char *fPath, char *calPath);      // creates nessesary objects and segment detectors

        void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& msg);
        void imageCallback(const sensor_msgs::ImageConstPtr& msg);

        // dynamic parameter reconfiguration
        static void reconfigureCallback(CWhycon *whycon, whycon_ros::whyconConfig& config, uint32_t level);

    private:

        /*GUI-related stuff*/
        CGui* gui;              // drawing, events capture
        bool useGui;            // use graphic interface at all?
        int guiScale;           // in case camera resolution exceeds screen one, gui is scaled down
        SDL_Event event;        // store mouse and keyboard events
        int keyNumber;          // number of keys pressed in the last step       
        Uint8 lastKeys[1000];   // keys pressed in the previous step
        Uint8 *keys;            // pressed keys
        bool displayHelp;       // displays some usage hints
        bool drawCoords;        // draws coordinatess at the robot's positions
        int runs;               // number of gui updates/detections performed 
        int evalTime;           // time required to detect the patterns
        int screenWidth;        // max GUI width
        int screenHeight;       // max GUI height

        /*variables related to (auto) calibration*/
        const int calibrationSteps = 20;            // how many measurements to average to estimate calibration pattern position (manual calib)
        const int autoCalibrationSteps = 30;        // how many measurements to average to estimate calibration pattern position (automatic calib)  
        const int autoCalibrationPreSteps = 10;     // how many measurements to discard before starting to actually auto-calibrating (automatic calib)  
        int calibNum;                       // number of objects acquired for calibration (5 means calibration winished inactive)
        STrackedObject calib[5];            // array to store calibration patterns positions
        STrackedObject *calibTmp;           // array to store several measurements of a given calibration pattern
        int calibStep;                      // actual calibration step (num of measurements of the actual pattern)
        bool autocalibrate;                 // is the autocalibration in progress ?
        ETransformType lastTransformType;   // pre-calibration transform (used to preserve pre-calibation transform type)
        int wasBots;                        // pre-calibration number of robots to track (used to preserve pre-calibation number of robots to track)

        /*robot detection variables*/
        STrackedObject *objectArray;       // object array (detected objects in metric space)
        SSegment *currInnerSegArray;       // inner segment array
        SSegment *currentSegmentArray;     // segment array (detected objects in image space)
        SSegment *lastSegmentArray;        // segment position in the last step (allows for tracking)
        CCircleDetect **detectorArray;     // detector array (each pattern has its own detector)

        CTransformation *trans;                         // allows to transform from image to metric coordinates
        CNecklace *decoder;                             // Necklace code decoder

        ros::NodeHandle *n;                     // ROS node
        ros::Subscriber subInfo;                // camera info subscriber
        image_transport::Subscriber subImg;     // image raw subscriber
        ros::Publisher markers_pub;             // publisher of MarkerArray
        CRawImage *image;                       // encapsulation of image raw data

        dynamic_reconfigure::Server<whycon_ros::whyconConfig> server;                   // server for dynamic reconfigure
        dynamic_reconfigure::Server<whycon_ros::whyconConfig>::CallbackType dynSer;     // callback server to handle reconfigure function

        std::string fontPath;           // path to GUI font
        std::string calibDefPath;       // path to user defined coordinate calibration

        // intrisic and distortion params from camera_info
        Mat intrinsic = Mat::ones(3,3, CV_32FC1);
        Mat distCoeffs = Mat::ones(1,5, CV_32FC1);

        /*manual calibration can be initiated by pressing 'r' and then clicking circles at four positions (0,0)(fieldLength,0)...*/
        void manualcalibration();

        /*finds four outermost circles and uses them to set-up the coordinate system - [0,0] is left-top, [0,fieldLength] next in clockwise direction*/
        void autocalibration();

        /*process events coming from GUI*/
        void processKeys();

};

#endif
