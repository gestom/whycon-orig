#include "CWhycon.h"

/*manual calibration can be initiated by pressing 'r' and then clicking circles at four positions (0,0)(fieldLength,0)...*/
void CWhycon::manualcalibration(){
    if (currentMarkerArray[0].valid){
        STrackedObject o = currentMarkerArray[0].obj;
        moveOne = moveVal;

        //object found - add to a buffer
        if (calibStep < calibrationSteps) calibTmp[calibStep++] = o;

        //does the buffer contain enough data to calculate the object position
        if (calibStep == calibrationSteps){
            o.x = o.y = o.z = 0;
            for (int k = 0; k < calibrationSteps; k++){
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
                trans->calibrate2D(calib, fieldLength, fieldWidth);
                trans->calibrate3D(calib, fieldLength, fieldWidth);
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
    for (int i = 0; i < numMarkers; i++){
        if (detectorArray[i]->lastTrackOK == false) saveVals = false;
    }
    if (saveVals){
        int index[] = {0,0,0,0};	
        int maxEval = 0;
        int eval = 0;
        int sX[] = {-1,+1,-1,+1};
        int sY[] = {+1,+1,-1,-1};
        for (int b = 0; b < 4; b++){
            maxEval = -10000000;
            for (int i = 0; i < numMarkers; i++){
                eval = 	sX[b] * currentMarkerArray[i].seg.x + sY[b] * currentMarkerArray[i].seg.y;
                if (eval > maxEval){
                    maxEval = eval;
                    index[b] = i;
                }
            }
        }
        printf("INDEX: %i %i %i %i\n", index[0], index[1], index[2], index[3]);
        for (int i = 0; i < 4; i++){
            if (calibStep <= autoCalibrationPreSteps) calib[i].x = calib[i].y = calib[i].z = 0;
            calib[i].x += currentMarkerArray[index[i]].obj.x;
            calib[i].y += currentMarkerArray[index[i]].obj.y;
            calib[i].z += currentMarkerArray[index[i]].obj.z;
        }
        calibStep++;
        if (calibStep == autoCalibrationSteps){
            for (int i = 0; i < 4; i++){
                calib[i].x = calib[i].x/(autoCalibrationSteps-autoCalibrationPreSteps);
                calib[i].y = calib[i].y/(autoCalibrationSteps-autoCalibrationPreSteps);
                calib[i].z = calib[i].z/(autoCalibrationSteps-autoCalibrationPreSteps);
            }
            trans->calibrate2D(calib, fieldLength, fieldWidth);
            trans->calibrate3D(calib, fieldLength, fieldWidth);
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
                currentMarkerArray[numMarkers-1].seg.x = event.motion.x * guiScale; 
                currentMarkerArray[numMarkers-1].seg.y = event.motion.y * guiScale;
                currentMarkerArray[numMarkers-1].valid = true;
                detectorArray[numMarkers-1]->localSearch = true;
            }
        }
    }

    //process keys 
    keys = SDL_GetKeyState(&keyNumber);
    bool shiftPressed = keys[SDLK_RSHIFT] || keys[SDLK_LSHIFT];

    //program control - (s)top, (p)ause+move one frame and resume
    if (keys[SDLK_ESCAPE]) stop = true;
    if (keys[SDLK_SPACE] && lastKeys[SDLK_SPACE] == false){
        moveOne = 100000000;
        moveVal = 10000000;
    }

    if (keys[SDLK_p] && lastKeys[SDLK_p] == false) {
        moveOne = 1;
        moveVal = 0;
    }

    if (keys[SDLK_m] && lastKeys[SDLK_m] == false) printf("SAVE %03f %03f %03f %03f %03f %03f\n", 
            currentMarkerArray[0].obj.x, currentMarkerArray[0].obj.y, currentMarkerArray[0].obj.z, currentMarkerArray[0].obj.d,
            currentMarkerArray[0].seg.m0, currentMarkerArray[0].seg.m1);
    if (keys[SDLK_n] && lastKeys[SDLK_n] == false) printf("SEGM %03f %03f %03f %03f\n", 
            currentMarkerArray[0].seg.x, currentMarkerArray[0].seg.y, currentMarkerArray[0].seg.m0, currentMarkerArray[0].seg.m1);
    if (keys[SDLK_s] && lastKeys[SDLK_s] == false) image->saveBmp();

    //initiate autocalibration (searches for 4 outermost circular patterns and uses them to establisht the coordinate system)
    if (keys[SDLK_a] && lastKeys[SDLK_a] == false) {
        calibStep = 0;
        lastTransformType = trans->transformType;
        wasMarkers = numMarkers;
        autocalibrate = true;
        trans->transformType = TRANSFORM_NONE;
    } 

    //manual calibration (click the 4 calibration circles with mouse)
    if (keys[SDLK_r] && lastKeys[SDLK_r] == false) {
        calibNum = 0;
        wasMarkers = numMarkers;
        numMarkers = 1;
    }

    //debugging - toggle drawing coordinates and debugging results results
    if (keys[SDLK_l] && lastKeys[SDLK_l] == false) drawCoords = drawCoords == false;
    if (keys[SDLK_d] && lastKeys[SDLK_d] == false){ 
        for (int i = 0;i<numMarkers;i++){
            detectorArray[i]->draw = detectorArray[i]->draw == false;
            detectorArray[i]->debug = detectorArray[i]->debug == false;
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
    memcpy(lastKeys, keys, keyNumber);
}

void CWhycon::cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& msg){
    if(msg->K[0] == 0){
        ROS_ERROR_ONCE("ERROR: Camera is not calibrated!");
        return;
    }else if(msg->K[0] != intrinsic.at<float>(0,0) || msg->K[2] != intrinsic.at<float>(0,2) || msg->K[4] != intrinsic.at<float>(1,1) ||  msg->K[5] != intrinsic.at<float>(1,2)){
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
    // setup timers to assess system performance
    CTimer timer;

    // check if readjusting of camera is needed
    if (image->bpp != msg->step/msg->width || image->width != msg->width || image->height != msg->height){
        ROS_INFO("Readjusting image format from %ix%i %ibpp, to %ix%i %ibpp.",
                image->width, image->height, image->bpp, msg->width, msg->height, msg->step / msg->width);
        delete image;
        image = new CRawImage(msg->width, msg->height, msg->step / msg->width);
        if(useGui){
            guiScale = 1;
            while(image->height / guiScale > screenHeight || image->height / guiScale > screenWidth) guiScale = guiScale * 2;
            if(gui != NULL) delete gui;
            gui = new CGui(msg->width, msg->height, guiScale, fontPath.c_str());
        }
    }
    memcpy(image->data, (void*)&msg->data[0], msg->step * msg->height);

    numFound = numStatic = 0;
    timer.reset();
    timer.start();

    // track the markers found in the last attempt
    for (int i = 0; i < numMarkers; i++){
        if (currentMarkerArray[i].valid){
            lastMarkerArray[i] = currentMarkerArray[i];
            currentMarkerArray[i] = detectorArray[i]->findSegment(image, lastMarkerArray[i].seg);
        }
    }

    // search for untracked (not detected in the last frame) markers
    for (int i = 0; i < numMarkers; i++){
        if (currentMarkerArray[i].valid == false){
            lastMarkerArray[i].valid = false;
            lastMarkerArray[i].seg.valid = false;
            currentMarkerArray[i] = detectorArray[i]->findSegment(image, lastMarkerArray[i].seg);
        }
        if (currentMarkerArray[i].seg.valid == false) break;  //does not make sense to search for more patterns if the last one was not found
    }

    for (int i = 0; i < numMarkers; i++){
        if (currentMarkerArray[i].valid){
            if(identify && currentMarkerArray[i].seg.ID <= -1){
                currentMarkerArray[i].seg.angle = lastMarkerArray[i].seg.angle;
                currentMarkerArray[i].seg.ID = lastMarkerArray[i].seg.ID;
            }
            numFound++;
            if (currentMarkerArray[i].seg.x == lastMarkerArray[i].seg.x) numStatic++;
        }
    }

    evalTime = timer.getTime();

    // Generate information about markers
    whycon_ros::MarkerArray markerArray;
    markerArray.header = msg->header;

    for (int i = 0; i < numMarkers; i++){
        if (currentMarkerArray[i].valid){
            whycon_ros::Marker marker;

            marker.id = currentMarkerArray[i].seg.ID;
            marker.size = currentMarkerArray[i].seg.size;
            marker.u = currentMarkerArray[i].seg.x;
            marker.v = currentMarkerArray[i].seg.y;
            marker.angle = currentMarkerArray[i].obj.angle;

            // Convert to ROS standard Coordinate System
            marker.position.position.x = -currentMarkerArray[i].obj.y;
            marker.position.position.y = -currentMarkerArray[i].obj.z;
            marker.position.position.z = currentMarkerArray[i].obj.x;

            geometry_msgs::Quaternion orientation;
            orientation.x = currentMarkerArray[i].obj.qx;
            orientation.y = currentMarkerArray[i].obj.qy;
            orientation.z = currentMarkerArray[i].obj.qz;
            orientation.w = currentMarkerArray[i].obj.qw;
            marker.position.orientation = orientation;

            // Euler angles
            marker.rotation.x = currentMarkerArray[i].obj.theta;
            marker.rotation.y = currentMarkerArray[i].obj.phi;
            marker.rotation.z = currentMarkerArray[i].obj.psi;

            markerArray.markers.push_back(marker);
        }
    }

    // Generate RVIZ visualization marker
    visualization_msgs::MarkerArray visualArray;

    if(pubVisual){
        for (int i = 0; i < numMarkers; i++){
            if (currentMarkerArray[i].valid){
                visualization_msgs::Marker visualMarker;
                visualMarker.header = msg->header;
                visualMarker.ns = "whycon";
                visualMarker.id = (identify) ? markerArray.markers[i].id : i;
                visualMarker.type = visualization_msgs::Marker::SPHERE;
                visualMarker.action = visualization_msgs::Marker::MODIFY;

                visualMarker.pose = markerArray.markers[i].position;
                visualMarker.scale.x = 0.5;//circleDiameter;  // meters
                visualMarker.scale.y = 0.25;//circleDiameter;
                visualMarker.scale.z = 0.01;
                visualMarker.color.r = 0.0;
                visualMarker.color.g = 1.0;
                visualMarker.color.b = 0.0;
                visualMarker.color.a = 1.0;
                visualMarker.lifetime = ros::Duration(0.2);  // sec

                visualArray.markers.push_back(visualMarker);
            }
        }
    }

    // publishing detected markers
    if(markerArray.markers.size() > 0) markers_pub.publish(markerArray);
    if(pubVisual && visualArray.markers.size() > 0) visual_pub.publish(visualArray);

    // draw stuff on the GUI 
    if (useGui){
        gui->drawImage(image);
        gui->drawTimeStats(evalTime, numMarkers);
        gui->displayHelp(displayHelp);
        gui->guideCalibration(calibNum, fieldLength, fieldWidth);
    }
    for (int i = 0; i < numMarkers && useGui && drawCoords; i++){
        if (currentMarkerArray[i].valid)
            /*TODO note #08*/
            gui->drawStats(currentMarkerArray[i].seg.minx-30, currentMarkerArray[i].seg.maxy, currentMarkerArray[i].obj, trans->transformType == TRANSFORM_2D);
    }

    // establishing the coordinate system by manual or autocalibration
    if (autocalibrate && numFound == numMarkers) autocalibration();
    if (calibNum < 4) manualcalibration();

    if(useGui){
        gui->update();
        processKeys();
    }
}

// dynamic parameter reconfiguration
void CWhycon::reconfigureCallback(CWhycon *whycon, whycon_ros::whyconConfig& config, uint32_t level){
    ROS_INFO("[Reconfigure Request]\n"
            "numMarkers %d\ncircleDiam %lf\nidentify %d\n"
            "initCircularityTolerance %lf\nfinalCircularityTolerance %lf\n"
            "areaRatioTolerance %lf\ncenterDistTolerance %lf\ncenterDistToleranceAbs %lf\n",
            config.numMarkers, config.circleDiameter, config.identify,\
            config.initialCircularityTolerance, config.finalCircularityTolerance,\
            config.areaRatioTolerance,config.centerDistanceToleranceRatio,config.centerDistanceToleranceAbs);

    whycon->numMarkers = (config.numMarkers > whycon->maxMarkers) ? whycon->maxMarkers : config.numMarkers;
    whycon->fieldLength = config.fieldLength;
    whycon->fieldWidth = config.fieldWidth;
    whycon->identify = config.identify;
    whycon->circleDiameter = config.circleDiameter / 100.0;

    trans->reconfigure(config.circleDiameter);

    for (int i = 0; i < whycon->maxMarkers; i++) whycon->detectorArray[i]->reconfigure(\
            config.initialCircularityTolerance, config.finalCircularityTolerance,\
            config.areaRatioTolerance,config.centerDistanceToleranceRatio,\
            config.centerDistanceToleranceAbs, config.identify, config.minSize);
}

// cleaning up
CWhycon::~CWhycon(){
    ROS_DEBUG("Releasing memory.");
    free(calibTmp);
    free(currentMarkerArray);
    free(lastMarkerArray);

    delete image;
    if (useGui) delete gui;
    delete trans;
    delete decoder;
    for (int i = 0;i<maxMarkers;i++) delete detectorArray[i];
    free(detectorArray);
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
    n->param("pubVisual", pubVisual, true);
    n->param("idBits", idBits, 5);
    n->param("idSamples", idSamples, 360);
    n->param("hammingDist", hammingDist, 1);
    n->param("maxMarkers", maxMarkers, 50);

    moveOne = moveVal;
    moveOne  = 0;
    calibTmp = (STrackedObject*) malloc(calibrationSteps * sizeof(STrackedObject));

    currentMarkerArray = (SMarker*) malloc(maxMarkers * sizeof(SMarker));
    lastMarkerArray = (SMarker*) malloc(maxMarkers * sizeof(SMarker));

    // determine gui size so that it fits the screen
    while (imageHeight/guiScale > screenHeight || imageHeight/guiScale > screenWidth) guiScale = guiScale*2;

    // initialize GUI, image structures, coordinate transformation modules
    if (useGui) gui = new CGui(imageWidth,imageHeight,guiScale, fontPath.c_str());

    detectorArray = (CCircleDetect**) malloc(maxMarkers * sizeof(CCircleDetect*));

    // initialize the circle detectors - each circle has its own detector instance 
    for (int i = 0; i < maxMarkers; i++) detectorArray[i] = new CCircleDetect(imageWidth, imageHeight, identify, idBits, idSamples, hammingDist);
    image->getSaveNumber();

    trans = new CTransformation(imageWidth, imageHeight, circleDiameter, calibDefPath.c_str());
    trans->transformType = TRANSFORM_NONE;      //in our case, 2D is the default
    decoder = new CNecklace(idBits, hammingDist);

    // initialize dynamic reconfiguration feedback
    dynSer = boost::bind(&CWhycon::reconfigureCallback, this, _1, _2);
    server.setCallback(dynSer);

    // subscribe to camera topic, publish topis with card position, rotation and ID
    subInfo = n->subscribe("/camera/camera_info", 1, &CWhycon::cameraInfoCallback, this);
    subImg = it.subscribe("/camera/image_raw", 1, &CWhycon::imageCallback, this);
    markers_pub = n->advertise<whycon_ros::MarkerArray>("/whycon_ros/markers", 1);
    if(pubVisual) visual_pub = n->advertise<visualization_msgs::MarkerArray>( "/whycon_ros/visual", 1);

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
