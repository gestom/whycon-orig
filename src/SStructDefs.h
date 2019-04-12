#ifndef SSTRUCTDEFS_H
#define SSTRUCTDEFS_H


typedef struct{
    float angle;    // axis rotation angle
    int id;         // marker decoded ID
    int edgeIndex;
} SDecoded;


// this structure contains information related to image coordinates and dimensions of the detected pattern
typedef struct{
    float x;                    // center in image coordinates
    float y;                    // center in image coordinates
    float angle,horizontal;     // orientation (not really used in this case, see the SwarmCon version of this software)
    int size;                   // number of pixels
    int maxy,maxx,miny,minx;    // bounding box dimensions
    int mean;                   // mean brightness
    int type;                   // black or white ?
    float roundness;            // result of the first roundness test, see Eq. 2 of paper [1]
    float bwRatio;              // ratio of white to black pixels, see Algorithm 2 of paper [1]
    bool round;                 // segment passed the initial roundness test
    bool valid;                 // marker passed all tests and will be passed to the transformation phase
    float m0,m1;                // eigenvalues of the pattern's covariance matrix, see Section 3.3 of [1]
    float v0,v1;                // eigenvectors of the pattern's covariance matrix, see Section 3.3 of [1]
    float r0,r1;                // ratio of inner vs outer ellipse dimensions (used to establish ID, see the SwarmCon version of this class)
    int ID;                     // pattern ID (experimental, see the SwarmCon version of this class)
} SSegment;

// which transform to use
typedef enum{
    TRANSFORM_NONE,     //camera-centric
    TRANSFORM_2D,       //3D->2D homography
    TRANSFORM_3D,       //3D user-defined - linear combination of four translation/rotation transforms
    TRANSFORM_4D,       //3D user-defined - full 4x3 matrix
    TRANSFORM_INV,      //for testing purposes
    TRANSFORM_NUMBER
} ETransformType;

typedef struct{
    float x,y,z,d;          // position and distance from the camera
    float pitch,roll,yaw;   // angles - precision is unknown

    float segX1,segY1;
    float x1,y1,z1,d1;
    float pitch1,roll1,yaw1;

    float segX2,segY2;
    float x2,y2,z2,d2;
    float pitch2,roll2,yaw2;
    
    float roundness;        // segment roundness as calculated by 5 of [1]
    float bwratio;          // black/white area ratio
    int ID;                 // ID of marker
} STrackedObject;

// rotation/translation model of the 3D transformation                                                                                                                  
typedef struct{
    STrackedObject orig;    // translation vector
    float simlar[3][3];     // rotation description
} S3DTransform;

typedef struct{
    bool valid;
    SSegment seg;
    STrackedObject obj;
} SMarker;

#endif

