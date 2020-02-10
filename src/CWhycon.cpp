#include "CWhycon.h"

/*manual calibration can be initiated by pressing 'r' and then clicking circles at four positions (0,0)(fieldLength,0)...*/
void CWhycon::manualcalibration(){
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
                numMarkers = wasMarkers;
                trans->saveCalibration(calibDefPath.c_str());
                trans->transformType = lastTransformType;
                detectorArray[0]->localSearch = false;
            }
            calibStep++;
        }
    }
}

/*finds four outermost circles and uses them to set-up the coordinate system - [0,0] is left-top, [0,fieldLength] next in clockwise direction*/
void CWhycon::autocalibration(){
    bool saveVals = true;
    for (int i = 0;i<numMarkers;i++){
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
            for (int i = 0;i<numMarkers;i++){
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
            numMarkers = wasMarkers;
            trans->saveCalibration(calibDefPath.c_str());
            trans->transformType = lastTransformType;
            autocalibrate = false;
        }
    }
}

/*process events coming from GUI*/
void CWhycon::processKeys(){
    //process mouse - mainly for manual calibration - by clicking four circles at the corners of the operational area 
    while (SDL_PollEvent(&event)){
        if (event.type == SDL_MOUSEBUTTONDOWN){
            if (calibNum < 4 && calibStep > calibrationSteps){
                calibStep = 0;
                trans->transformType = TRANSFORM_NONE;
            }
            if (numMarkers > 0){
                currentSegmentArray[numMarkers-1].x = event.motion.x*guiScale; 
                currentSegmentArray[numMarkers-1].y = event.motion.y*guiScale;
                currentSegmentArray[numMarkers-1].valid = true;
                detectorArray[numMarkers-1]->localSearch = true;
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
    if (keys[SDLK_n] && lastKeys[SDLK_n] == false) printf("SEGM %03f %03f %03f %03f\n",currentSegmentArray[0].x,currentSegmentArray[0].y,currentSegmentArray[0].m0,currentSegmentArray[0].m1);
    if (keys[SDLK_s] && lastKeys[SDLK_s] == false) image->saveBmp();

    //initiate autocalibration (searches for 4 outermost circular patterns and uses them to establisht the coordinate system)
    if (keys[SDLK_a] && lastKeys[SDLK_a] == false) { calibStep = 0; lastTransformType=trans->transformType; wasMarkers = numMarkers; autocalibrate = true;trans->transformType=TRANSFORM_NONE;}; 

    //manual calibration (click the 4 calibration circles with mouse)
    if (keys[SDLK_r] && lastKeys[SDLK_r] == false) { calibNum = 0; wasMarkers=numMarkers; numMarkers = 1;}

    //debugging - toggle drawing coordinates and debugging results results
    if (keys[SDLK_l] && lastKeys[SDLK_l] == false) drawCoords = drawCoords == false;
    if (keys[SDLK_d] && lastKeys[SDLK_d] == false){ 
        for (int i = 0;i<numMarkers;i++){
            detectorArray[i]->draw = detectorArray[i]->draw==false;
            detectorArray[i]->debug = detectorArray[i]->debug==false;
            decoder->debugSegment = decoder->debugSegment==false;
        }
    }

    //transformations to use - in our case, the relevant transform is '2D'
    if (keys[SDLK_1] && lastKeys[SDLK_1] == false) trans->transformType = TRANSFORM_NONE;
    if (keys[SDLK_2] && lastKeys[SDLK_2] == false) trans->transformType = TRANSFORM_2D;
    if (keys[SDLK_3] && lastKeys[SDLK_3] == false) trans->transformType = TRANSFORM_3D;

    // TODO camera low-level settings 

    //display help
    if (keys[SDLK_h] && lastKeys[SDLK_h] == false) displayHelp = displayHelp == false; 

    //adjust the number of robots to be searched for
    if (keys[SDLK_PLUS]) numMarkers++;
    if (keys[SDLK_EQUALS]) numMarkers++;
    if (keys[SDLK_MINUS]) numMarkers--;
    if (keys[SDLK_KP_PLUS]) numMarkers++;
    if (keys[SDLK_KP_MINUS]) numMarkers--;

    //store the key states
    memcpy(lastKeys,keys,keyNumber);
}

void CWhycon::cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& msg){
    if(msg->K[0] == 0){
        ROS_FATAL("ERROR: Camera is not calibrated! Shutting down!");
        ros::shutdown();
    }
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

void CWhycon::imageCallback(const sensor_msgs::ImageConstPtr& msg){
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
        ROS_INFO("Readjusting image format from %ix%i %ibpp, to %ix%i %ibpp.",
                image->width, image->height, image->bpp, msg->width, msg->height, msg->step/msg->width);
        image = new CRawImage(msg->width,msg->height,msg->step/msg->width);
        if(useGui){
            while(image->height/guiScale > screenHeight || image->height/guiScale > screenWidth) guiScale = guiScale*2;
            if(gui == NULL){
                gui = new CGui(msg->width, msg->height, guiScale, fontPath.c_str());
            }else{
                delete gui;
                gui = new CGui(msg->width, msg->height, guiScale, fontPath.c_str());
            }
        }
    }

    memcpy(image->data,(void*)&msg->data[0],msg->step*msg->height);

    numFound = numStatic = 0;
    timer.reset();

    // track the robots found in the last attempt 
    for (int i = 0;i<numMarkers;i++){
        if (currentSegmentArray[i].valid){
            lastSegmentArray[i] = currentSegmentArray[i];
            currentSegmentArray[i] = detectorArray[i]->findSegment(image,lastSegmentArray[i]);
            currInnerSegArray[i] = detectorArray[i]->getInnerSegment();
        }
    }

    // search for untracked (not detected in the last frame) robots 
    for (int i = 0;i<numMarkers;i++){
        if (currentSegmentArray[i].valid == false){
            lastSegmentArray[i].valid = false;
            currentSegmentArray[i] = detectorArray[i]->findSegment(image,lastSegmentArray[i]);
            currInnerSegArray[i] = detectorArray[i]->getInnerSegment();
        }
        if (currentSegmentArray[i].valid == false) break;		//does not make sense to search for more patterns if the last one was not found
    }

    // perform transformations from camera to world coordinates
    for (int i = 0;i<numMarkers;i++){
        if (currentSegmentArray[i].valid){
            int step = image->bpp;
            int pos;
            pos = ((int)currentSegmentArray[i].x+((int)currentSegmentArray[i].y)*image->width);
            image->data[step*pos+0] = 255;
            image->data[step*pos+1] = 0;
            image->data[step*pos+2] = 0;
            pos = ((int)currInnerSegArray[i].x+((int)currInnerSegArray[i].y)*image->width);
            image->data[step*pos+0] = 0;
            image->data[step*pos+1] = 255;
            image->data[step*pos+2] = 0;

            objectArray[i] = trans->transform(currentSegmentArray[i]);

            if(identify){
                int segmentID = decoder->identifySegment(&currentSegmentArray[i], &objectArray[i], image) + 1;
                // if (debug) printf("SEGMENT ID: %i\n", segmentID);
                if (segmentID > -1){
                    // objectArray[i].yaw = currentSegmentArray[i].angle;
                    currentSegmentArray[i].ID = segmentID;
                }else{
                    currentSegmentArray[i].angle = lastSegmentArray[i].angle;
                    currentSegmentArray[i].ID = lastSegmentArray[i].ID;
                }
            }else{
                float dist1 = sqrt((currInnerSegArray[i].x-objectArray[i].segX1)*(currInnerSegArray[i].x-objectArray[i].segX1)+(currInnerSegArray[i].y-objectArray[i].segY1)*(currInnerSegArray[i].y-objectArray[i].segY1));
                float dist2 = sqrt((currInnerSegArray[i].x-objectArray[i].segX2)*(currInnerSegArray[i].x-objectArray[i].segX2)+(currInnerSegArray[i].y-objectArray[i].segY2)*(currInnerSegArray[i].y-objectArray[i].segY2));
                if(dist1 < dist2){
                    currentSegmentArray[i].x = objectArray[i].segX1;
                    currentSegmentArray[i].y = objectArray[i].segY1;
                    objectArray[i].x = objectArray[i].x1;
                    objectArray[i].y = objectArray[i].y1;
                    objectArray[i].z = objectArray[i].z1;
                    objectArray[i].pitch = objectArray[i].pitch1;
                    objectArray[i].roll = objectArray[i].roll1;
                    objectArray[i].yaw = objectArray[i].yaw1;
                }else{
                    currentSegmentArray[i].x = objectArray[i].segX2;
                    currentSegmentArray[i].y = objectArray[i].segY2;
                    objectArray[i].x = objectArray[i].x2;
                    objectArray[i].y = objectArray[i].y2;
                    objectArray[i].z = objectArray[i].z2;
                    objectArray[i].pitch = objectArray[i].pitch2;
                    objectArray[i].roll = objectArray[i].roll2;
                    objectArray[i].yaw = objectArray[i].yaw2;
                }
                currentSegmentArray[i].angle = 0;
            }

            numFound++;
            if (currentSegmentArray[i].x == lastSegmentArray[i].x) numStatic++;

        }
    }
    // if(numFound > 0) ROS_INFO("Pattern detection time: %i us. Found: %i Static: %i.",globalTimer.getTime(),numFound,numStatic);
    evalTime = timer.getTime();

    // publishing information about tags 
    whycon_ros::MarkerArray markerArray;
    markerArray.header = msg->header;

    for (int i = 0;i<numMarkers;i++){
        if (currentSegmentArray[i].valid){
            // printf("ID %d\n", currentSegmentArray[i].ID);
            whycon_ros::Marker marker;

            marker.id = currentSegmentArray[i].ID;
            marker.u = currentSegmentArray[i].x;
            marker.v = currentSegmentArray[i].y;
            marker.size = currentSegmentArray[i].size;

            // Convert to ROS standard Coordinate System
            marker.position.position.x = -objectArray[i].y;
            marker.position.position.y = -objectArray[i].z;
            marker.position.position.z = objectArray[i].x;

            // Convert YPR to Quaternion
            /*tf::Quaternion q;
            q.setRPY(objectArray[i].roll, objectArray[i].pitch, objectArray[i].yaw);
            marker.position.orientation.x = q.getX();
            marker.position.orientation.y = q.getY();
            marker.position.orientation.z = q.getZ();
            marker.position.orientation.w = q.getW();*/

            tf::Vector3 axis_vector(objectArray[i].pitch, objectArray[i].roll, objectArray[i].yaw);
            tf::Vector3 up_vector(0.0, 0.0, 1.0);
            tf::Vector3 right_vector = axis_vector.cross(up_vector);
            right_vector.normalized();
            tf::Quaternion quat(right_vector, -1.0*acos(axis_vector.dot(up_vector)));
            quat.normalize();
            geometry_msgs::Quaternion marker_orientation;
            tf::quaternionTFToMsg(quat, marker_orientation);

            marker.position.orientation = marker_orientation;

            marker.yaw = currentSegmentArray[i].angle;

            markerArray.markers.push_back(marker);
        }
    }

    if(markerArray.markers.size() > 0) markers_pub.publish(markerArray);

    //draw stuff on the GUI 
    if (useGui){
        gui->drawImage(image);
        gui->drawTimeStats(evalTime,numMarkers);
        gui->displayHelp(displayHelp);
        gui->guideCalibration(calibNum,fieldLength,fieldWidth);
    }
    for (int i = 0;i<numMarkers && useGui && drawCoords;i++){
        if (currentSegmentArray[i].valid) gui->drawStats(currentSegmentArray[i].minx-30,currentSegmentArray[i].maxy,objectArray[i],trans->transformType == TRANSFORM_2D);
    }

    //establishing the coordinate system by manual or autocalibration
    if (autocalibrate && numFound == numMarkers) autocalibration();
    if (calibNum < 4) manualcalibration();

    /* empty for-cycle that isn't used even in master orig version
       for (int i = 0;i<numMarkers;i++){
    //if (currentSegmentArray[i].valid) printf("Object %i %03f %03f %03f %03f %03f\n",i,objectArray[i].x,objectArray[i].y,objectArray[i].z,objectArray[i].error,objectArray[i].esterror);
    }*/

    //gui->saveScreen(runs);
    if (useGui) gui->update();
    if (useGui) processKeys();
}

// dynamic parameter reconfiguration
void CWhycon::reconfigureCallback(CWhycon *whycon, whycon_ros::whyconConfig& config, uint32_t level){
    ROS_INFO("[Reconfigure Request]\n"
            "numMarkers %d circleDiam %lf identify %d\n"
            "initCircularityTolerance %lf finalCircularityTolerance %lf\n"
            "areaRatioTolerance %lf centerDistTolerance %lf centerDistToleranceAbs %lf\n",
            config.numMarkers, config.circleDiameter, config.identify,\
            config.initialCircularityTolerance, config.finalCircularityTolerance,\
            config.areaRatioTolerance,config.centerDistanceToleranceRatio,config.centerDistanceToleranceAbs);

    whycon->numMarkers = (config.numMarkers > whycon->maxMarkers) ? whycon->maxMarkers : config.numMarkers;
    whycon->fieldLength = config.fieldLength;
    whycon->fieldWidth = config.fieldWidth;
    whycon->identify = config.identify;

    whycon->trans->reconfigure(config.circleDiameter);

    for (int i = 0;i<whycon->maxMarkers;i++) whycon->detectorArray[i]->reconfigure(\
            config.initialCircularityTolerance, config.finalCircularityTolerance,\
            config.areaRatioTolerance,config.centerDistanceToleranceRatio,\
            config.centerDistanceToleranceAbs, config.identify, config.minSize);
}

// cleaning up
CWhycon::~CWhycon(){
    ROS_DEBUG("Releasing memory.");
    free(calibTmp);
    free(objectArray);
    free(currInnerSegArray);
    free(currentSegmentArray);
    free(lastSegmentArray);

    delete image;
    if (useGui) delete gui;
    for (int i = 0;i<maxMarkers;i++) delete detectorArray[i];
    free(detectorArray);
    delete trans;
    delete decoder;
    delete n;
}

CWhycon::CWhycon(){
}

void CWhycon::init(char *fPath, char *calPath){
    n = new ros::NodeHandle("~");
    image_transport::ImageTransport it(*n);
    image = new CRawImage(imageWidth,imageHeight, 3);

    // loading params and args from launch file
    fontPath = fPath;
    calibDefPath = calPath;
    n->param("useGui", useGui, true);
    n->param("idBits", idBits, 5);
    n->param("idSamples", idSamples, 360);
    n->param("hammingDist", hammingDist, 1);
    n->param("maxMarkers", maxMarkers, 50);

    moveOne = moveVal;
    moveOne  = 0;
    calibTmp = (STrackedObject*) malloc(calibrationSteps * sizeof(STrackedObject));

    objectArray = (STrackedObject*) malloc(maxMarkers * sizeof(STrackedObject));
    currInnerSegArray = (SSegment*) malloc(maxMarkers * sizeof(SSegment));
    currentSegmentArray = (SSegment*) malloc(maxMarkers * sizeof(SSegment));
    lastSegmentArray = (SSegment*) malloc(maxMarkers * sizeof(SSegment));

    // determine gui size so that it fits the screen
    while (imageHeight/guiScale > screenHeight || imageHeight/guiScale > screenWidth) guiScale = guiScale*2;

    // initialize GUI, image structures, coordinate transformation modules
    if (useGui) gui = new CGui(imageWidth,imageHeight,guiScale, fontPath.c_str());
    trans = new CTransformation(imageWidth,imageHeight,circleDiameter, calibDefPath.c_str());
    trans->transformType = TRANSFORM_NONE;		//in our case, 2D is the default

    detectorArray = (CCircleDetect**) malloc(maxMarkers * sizeof(CCircleDetect*));

    // initialize the circle detectors - each circle has its own detector instance 
    for (int i = 0;i<maxMarkers;i++) detectorArray[i] = new CCircleDetect(imageWidth,imageHeight,identify, idBits, idSamples, hammingDist);
    image->getSaveNumber();

    decoder = new CNecklace(idBits,idSamples,hammingDist);

    // initialize dynamic reconfiguration feedback
    dynSer = boost::bind(&CWhycon::reconfigureCallback, this, _1, _2);
    server.setCallback(dynSer);

    // subscribe to camera topic, publish topis with card position, rotation and ID
    subInfo = n->subscribe("/camera/camera_info", 1, &CWhycon::cameraInfoCallback, this);
    subImg = it.subscribe("/camera/image_raw", 1, &CWhycon::imageCallback, this);
    markers_pub = n->advertise<whycon_ros::MarkerArray>("markers", 1);

    while (ros::ok()){
        ros::spinOnce();
        usleep(30000);
        if(stop) break;
    }
}

int main(int argc,char* argv[]){
    ros::init(argc, argv, "whycon_ros");

    CWhycon *whycon = new CWhycon();
    whycon->init(argv[1], argv[2]);

    delete whycon;

    return 0;
}
