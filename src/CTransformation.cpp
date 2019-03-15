#include "CTransformation.h"
#include <stdio.h>
#include "sysmat.h" 

/*
 * File name: CTransformation.h
 * Date:      2014
 * Author:   Tom Krajnik, Matias Nitsche
 * Description: Transforms detected ellipse position and dimensions to arbitrary 3D or 2D coordinate frame. The method is described in Chapter 4 of the article [1]. 
 * Licence: if you use this class for your research, please cite [1]. 
 * References: [1] Krajnik, Nitsche et al.: A practical multirobot localization system. Journal of Intelligent and Robotic Systems, 2014.
 */


int sortByDistance(const void* m1,const void* m2)
{
    if (((STrackedObject*)m1)->d > ((STrackedObject*)m2)->d) return -1;
    if (((STrackedObject*)m1)->d < ((STrackedObject*)m2)->d) return 1;
    return 0;
}

CTransformation::CTransformation(int widthi,int heighti,float diam, const char* calibDefPath)
{
    transformType = TRANSFORM_NONE;
    width = widthi;
    height = heighti;
    trackedObjectDiameter = diam;
    loadCalibration(calibDefPath);

    distCoeffs = Mat(1,5, CV_32FC1);	
    intrinsic = Mat(3,3, CV_32FC1);

    /*for (int i=0;i<9;i++)intrinsic.at<float>(i/3,i%3) = 0;
      intrinsic.at<float>(0,0) = 650; 
      intrinsic.at<float>(1,1) = 650;
      intrinsic.at<float>(0,2) =  320 ; 
      intrinsic.at<float>(1,2) = 240;
      intrinsic.at<float>(2,2) = 1;*/
}

CTransformation::~CTransformation()
{

}

void CTransformation::reconfigure(float circleDiam)
{
    trackedObjectDiameter = circleDiam / 100.0;
}

void CTransformation::updateParams(Mat intri, Mat dist)
{
    intrinsic = intri;
    distCoeffs = dist;
}

void CTransformation::reTransformXY(float *x, float *y,float *z)
{
    Mat in = Mat::ones(1, 1, CV_32FC3);
    Mat out = Mat::ones(1, 1, CV_32FC2);

    //in.at<float>(0) = (*x - intrinsic.at<float>(0,2))/intrinsic.at<float>(0,0);
    //in.at<float>(1) = (*y - intrinsic.at<float>(1,2))/intrinsic.at<float>(1,1);
    in.at<float>(0) = *x;
    in.at<float>(1) = *y;
    in.at<float>(2) = *z;

    projectPoints(in, Mat::zeros(3, 1, CV_32FC1), Mat::zeros(3, 1, CV_32FC1), intrinsic, distCoeffs, out);

    *x = out.at<float>(0);
    *y = out.at<float>(1);
}

void CTransformation::transformXY(float *ax,float *ay)
{
    Mat coords = Mat::ones(1, 1, CV_32FC2);
    Mat metric = Mat::ones(1, 1, CV_32FC2);
    coords.at<float>(0) = *ax;
    coords.at<float>(1) = *ay;
    //	cout << intrinsic << endl;
    undistortPoints(coords,metric,intrinsic,distCoeffs);
    *ax = metric.at<float>(0);
    *ay = metric.at<float>(1);
}

STrackedObject CTransformation::transform4D(STrackedObject o)
{
    STrackedObject r;
    r.x = trf4D[0]*o.x+trf4D[1]*o.y+trf4D[2]*o.z+trf4D[3]; 
    r.y = trf4D[4]*o.x+trf4D[5]*o.y+trf4D[6]*o.z+trf4D[7]; 
    r.z = trf4D[8]*o.x+trf4D[9]*o.y+trf4D[10]*o.z+trf4D[11]; 
    float s = trf4D[12]*o.x+trf4D[13]*o.y+trf4D[14]*o.z+trf4D[15]; 
    r.x = r.x/s;
    r.y = r.y/s;
    r.z = r.z/s;
    return r; 	
}

STrackedObject CTransformation::transform2D(STrackedObject o)
{
    STrackedObject r;
    r.x = hom[0]*o.x+hom[1]*o.y+hom[2]; 
    r.y = hom[3]*o.x+hom[4]*o.y+hom[5];
    r.z = hom[6]*o.x+hom[7]*o.y+hom[8];
    r.x = r.x/r.z;
    r.y = r.y/r.z;
    r.z = 0;
    //printf("%.3f %.3f\n",r.x,r.y);
    return r; 	
}

float CTransformation::distance(STrackedObject o1,STrackedObject o2)
{
    return sqrt((o1.x-o2.x)*(o1.x-o2.x)+(o1.y-o2.y)*(o1.y-o2.y)+(o1.z-o2.z)*(o1.z-o2.z));
}

STrackedObject CTransformation::transform3D(STrackedObject o,int num)
{
    STrackedObject result[4];
    STrackedObject final;
    STrackedObject a;
    final.x = final.y = final.z = 0;
    float str = 0;
    float strAll = 0;
    for (int k = 0;k<num;k++){
        a.x = o.x-D3transform[k].orig.x;
        a.y = o.y-D3transform[k].orig.y;
        a.z = o.z-D3transform[k].orig.z;
        result[k].x = D3transform[k].simlar[0][0]*a.x+D3transform[k].simlar[0][1]*a.y+D3transform[k].simlar[0][2]*a.z;
        result[k].y = D3transform[k].simlar[1][0]*a.x+D3transform[k].simlar[1][1]*a.y+D3transform[k].simlar[1][2]*a.z;
        result[k].z = D3transform[k].simlar[2][0]*a.x+D3transform[k].simlar[2][1]*a.y+D3transform[k].simlar[2][2]*a.z;
        result[k].x = (k%2)*gDimX+(1-(k%2)*2)*result[k].x;
        result[k].y = (k/2)*gDimY+(1-(k/2)*2)*result[k].y;
        if (k ==0 || k == 3) result[k].z = -result[k].z;
        //result.y = +result.y+(D3transform[k].orig.y-D3transform[0].orig.y);
        //result.z = -result.z+(D3transform[k].orig.z-D3transform[0].orig.z);
        str=1.0/(a.x*a.x + a.y*a.y + a.z*a.z+0.01);

        final.x += str*result[k].x;
        final.y += str*result[k].y;
        final.z += str*result[k].z;
        strAll +=str;
        //printf("UUU: %f %f %f %f %f\n",result[k].x,result[k].y,result[k].z,str,establishError(result[k]));
    }
    final.x=final.x/strAll;
    final.y=final.y/strAll;	
    final.z=final.z/strAll;	

    return final;
}

void CTransformation::loadCalibration(const char *str)
{
    FILE* file = fopen(str,"r+");
    int k = 0;
    if (file == NULL){
        for (int i = 0;i<3;i++){
            for (int j = 0;j<3;j++)D3transform[k].simlar[i][j]=0;
        }
        D3transform[k].orig.x=D3transform[k].orig.y=D3transform[k].orig.z=0;
        for (int i = 0;i<9;i++)hom[i] = 0;
        hom[8] = 1;
    }else{
        char errStr[1000];
        char dumStr[1000];
        sprintf(errStr,"Transformation: error reading coordinate system transformation file %s\n",str);
        if (fscanf(file,"Dimensions %f %f\n",&gDimX,&gDimY)!=2) fprintf(stderr,"%s",errStr);
        int dum = 0;
        for (int k = 0;k<4;k++){
            if (fscanf(file,"3D_calibration %i\n",&dum)!=1) fprintf(stderr,"%s",errStr);
            for (int i = 0;i<3;i++){
                for (int j = 0;j<3;j++){
                    if (fscanf(file,"%f ",&D3transform[k].simlar[i][j])!=1) fprintf(stderr,"%s",errStr);
                }
                if (fscanf(file,"\n")!=0) fprintf(stderr,"%s",errStr);
            }
            if (fscanf(file,"Offset %f %f %f\n",&D3transform[k].orig.x,&D3transform[k].orig.y,&D3transform[k].orig.z)!=3)fprintf(stderr,"%s",errStr);
        }
        if (fscanf(file,"%s\n",dumStr)!=1) fprintf(stderr,"%s",errStr);
        for (int i = 0;i<9;i++){
            if (fscanf(file,"%f ",&hom[i])!=1)fprintf(stderr,"%s",errStr);
            if (i%3 == 2){
                if (fscanf(file,"\n")!=0) fprintf(stderr,"%s",errStr);
            }
        }
        fclose(file);
    }
}

void CTransformation::saveCalibration(const char *str)
{
    FILE* file = fopen(str,"w+");
    fprintf(file,"Dimensions %f %f\n",gDimX,gDimY);
    for (int k = 0;k<4;k++){
        fprintf(file,"3D_calibration %i\n",k);
        for (int i = 0;i<3;i++){
            for (int j = 0;j<3;j++){
                fprintf(file,"%f ",D3transform[k].simlar[i][j]);
            }
            fprintf(file,"\n");
        }
        fprintf(file,"Offset %f %f %f\n",D3transform[k].orig.x,D3transform[k].orig.y,D3transform[k].orig.z);
    }
    fprintf(file,"2D_calibration\n");
    for (int i = 0;i<9;i++){
        fprintf(file,"%f ",hom[i]);
        if (i%3 == 2) fprintf(file,"\n");
    }
    fclose(file);
}

STrackedObject CTransformation::normalize(STrackedObject o)
{
    float scale = sqrt(o.x*o.x+o.y*o.y+o.z*o.z);
    STrackedObject r;
    r.x = o.x/scale;
    r.y = o.y/scale;
    r.z = o.z/scale;
    return r;
}

int CTransformation::calibrate2D(STrackedObject *inp,float dimX,float dimY,float robotRadius,float robotHeight,float cameraHeight)
{
    STrackedObject r[4];
    STrackedObject o[4];
    /*specific to the pheromone system - compensates the fact, that the calibration patterns are displayed in a lower position than the robots
      assumes that the camera above the field centre*/
    float iX = dimX/cameraHeight*robotHeight/2;
    float iY = dimY/cameraHeight*robotHeight/2;

    //float iX = dimX/(inp[0].x+inp[1].x+inp[2].x+inp[3].x)*4*off;
    //float iY = dimY/(inp[0].x+inp[1].x+inp[2].x+inp[3].x)*4*off;

    r[0].x = robotRadius+iX;
    r[0].y = robotRadius+iY;
    r[1].x = dimX-robotRadius-iX;
    r[1].y = robotRadius+iY;
    r[2].x = robotRadius+iX;
    r[2].y = dimY-robotRadius-iY;
    r[3].x = dimX-robotRadius-iX;
    r[3].y = dimY-robotRadius-iY;
    for (int i = 0;i<4;i++){
        o[i].x = -inp[i].y/inp[i].x;
        o[i].y = -inp[i].z/inp[i].x;
    }

    MAT est;
    MAT1 vec;
    REAL det;
    for (int i = 0;i<4;i++){
        est[2*i][0]=-o[i].x;
        est[2*i][1]=-o[i].y;
        est[2*i][2]=-1;
        est[2*i][3]=0;
        est[2*i][4]=0;
        est[2*i][5]=0;
        est[2*i][6]=r[i].x*o[i].x;
        est[2*i][7]=r[i].x*o[i].y;
        est[2*i+1][0]=0;
        est[2*i+1][1]=0;
        est[2*i+1][2]=0;
        est[2*i+1][3]=-o[i].x;
        est[2*i+1][4]=-o[i].y;
        est[2*i+1][5]=-1;
        est[2*i+1][6]=r[i].y*o[i].x;
        est[2*i+1][7]=r[i].y*o[i].y;
        vec[2*i][0]=-r[i].x;
        vec[2*i+1][0]=-r[i].y;
    }
    MATINV(8,1,est,vec,&det); 
    for (int i = 0;i<8;i++)  hom[i] = vec[i][0];
    hom[8] = 1;
    transformType = TRANSFORM_2D;
    return 0;
}

S3DTransform CTransformation::calibrate3D(STrackedObject o0,STrackedObject o1,STrackedObject o2,float gridDimX,float gridDimY)
{
    S3DTransform result;
    STrackedObject v[3];
    result.orig = o0;

    MAT m23D;
    MAT1 vec;
    REAL det;

    v[0].x = o1.x-o0.x;
    v[0].y = o1.y-o0.y;
    v[0].z = o1.z-o0.z;
    //v[0] = normalize(v[0]);

    v[1].x = o2.x-o0.x;
    v[1].y = o2.y-o0.y;
    v[1].z = o2.z-o0.z;
    //v[1] = normalize(v[1]);

    v[2].x = +v[0].y*v[1].z-v[1].y*v[0].z;
    v[2].y = +v[0].z*v[1].x-v[1].z*v[0].x;
    v[2].z = +v[0].x*v[1].y-v[1].x*v[0].y;
    //v[2] = normalize(v[2]);

    for (int i = 0;i<3;i++){
        m23D[0][i]=v[i].x;
        m23D[1][i]=v[i].y;
        m23D[2][i]=v[i].z;
    }
    MATINV(3,3,m23D,vec,&det); 
    for (int i = 0;i<3;i++){
        result.simlar[0][i] = m23D[0][i]*gridDimX;
        result.simlar[1][i] = m23D[1][i]*gridDimY;
        result.simlar[2][i] = m23D[2][i]*gridDimX*gridDimY;
    }
    transformType = TRANSFORM_3D;
    return result;
}

int CTransformation::calibrate3D(STrackedObject *o,float gridDimX,float gridDimY)
{
    D3transform[0] = calibrate3D(o[0],o[1],o[2],gridDimX,gridDimY);
    D3transform[1] = calibrate3D(o[1],o[0],o[3],gridDimX,gridDimY);
    D3transform[2] = calibrate3D(o[2],o[3],o[0],gridDimX,gridDimY);
    D3transform[3] = calibrate3D(o[3],o[2],o[1],gridDimX,gridDimY);
    gDimX = gridDimX;
    gDimY = gridDimY;
    transformType = TRANSFORM_3D;
    return 0;
}

//implemented according to
STrackedObject CTransformation::calcEigen(double data[])
{
    STrackedObject result;
    //	double d[3];
    double V[3][3];
    //	double Vi[3][3];
    //	double dat[3][3];
    Mat val = Mat(3,1,CV_32FC1);
    Mat vec = Mat(3,3,CV_32FC1);
    //	for (int i = 0;i<9;i++)dat[i/3][i%3] = data[i];
    //	eigen_decomposition(dat,Vi,d);

    Mat in = Mat(3,3,CV_32FC1);
    for(int i=0;i<3;i++){
        for(int j=0;j<3;j++){
            in.at<float>(i,j) = data[3*i+j];
        }
    }

    eigen(in,val,vec);

    for(int i = 0;i<3;i++){
        V[i][1] = vec.at<float>(1,i);
        V[i][2] = vec.at<float>(0,i);
        V[i][0] = vec.at<float>(2,i);
    }

    //eigenvalues
    float L1 = val.at<float>(1);//d[1];
    float L2 = val.at<float>(0);//d[2];
    float L3 = val.at<float>(2);//d[0];
    //eigenvectors
    int V2=2;
    int V3=0;

    //detected pattern position
    float z = trackedObjectDiameter/sqrt(-L2*L3)/2.0;
    float c0 =  sqrt((L2-L1)/(L2-L3));
    float c0x = c0*V[0][V2];
    float c0y = c0*V[1][V2];
    float c0z = c0*V[2][V2];
    float c1 =  sqrt((L1-L3)/(L2-L3));
    float c1x = c1*V[0][V3];
    float c1y = c1*V[1][V3];
    float c1z = c1*V[2][V3];

    float z0 = -L3*c0x+L2*c1x;
    float z1 = -L3*c0y+L2*c1y;
    float z2 = -L3*c0z+L2*c1z;
    float s1,s2,s3;
    s1 = 1;
    s2 = 1;
    s3 = 1;
    float n0 = +s1*c0x+s2*c1x;
    float n1 = +s1*c0y+s2*c1y;
    float n2 = +s1*c0z+s2*c1z;

    if (z2*z < 0){
        z2 = -z2;
        z1 = -z1;
        z0 = -z0;
//        n0 = -n0;
//        n1 = -n1;
//        n2 = -n2;
    }

    result.x = z2*z;	
    result.y = -z0*z;	
    result.z = -z1*z;
    result.pitch = n0;//cos(segment.m1/segment.m0)/M_PI*180.0;
    result.roll = n1;//atan2(segment.v1,segment.v0)/M_PI*180.0;
    result.yaw = n2;//segment.v1/segment.v0;

    float newX, newY, newZ;
    float nX[2];
    float nY[2];
    float Z[2][3];
    float N[2][3];
    int tmp_idx = 0;
    for(int i=1;i<9;i++){
        s1 = (i%2 == 0) ? -1 : 1;
        s2 = (i ==3 || i ==4 || i==7 || i==8) ? -1 : 1;
        s3 = (i > 4) ? -1 : 1;
        z0 = s3*z*(s1*L3*c0x+s2*L2*c1x);
        z1 = s3*z*(s1*L3*c0y+s2*L2*c1y);
        z2 = s3*z*(s1*L3*c0z+s2*L2*c1z);
        n0 = s1*c0x+s2*c1x;
        n1 = s1*c0y+s2*c1y;
        n2 = s1*c0z+s2*c1z;
        if(z2 > 0 && n2 > 0){
//            printf("s1 %.0f s2 %.0f s3 %.0f\n", s1,s2,s3);
//            printf("z0 %f z1 %f z2 %f n0 %f n1 %f n2 %f\n", z0,z1,z2,n0,n1,n2);
            Z[tmp_idx][0] = z0;
            Z[tmp_idx][1] = z1;
            Z[tmp_idx][2] = z2;
            N[tmp_idx][0] = n0;
            N[tmp_idx][1] = n1;
            N[tmp_idx][2] = n2;
            newX = z0;
            newY = z1;
            newZ = z2;
            reTransformXY(&newX, &newY, &newZ);
            nX[tmp_idx] = newX;
            nY[tmp_idx] = newY;
            tmp_idx++;
        }
    }

    result.segX1 = nX[0];
    result.segY1 = nY[0];
    result.x1 = Z[0][2];
    result.y1 = -Z[0][0];
    result.z1 = -Z[0][1];
    result.pitch1 = N[0][0];
    result.roll1 = N[0][1];
    result.yaw1 = N[0][2];
    
    result.segX2 = nX[1];
    result.segY2 = nY[1];
    result.x2 = Z[1][2];
    result.y2 = -Z[1][0];
    result.z2 = -Z[1][1];
    result.pitch2 = N[1][0];
    result.roll2 = N[1][1];
    result.yaw2 = N[1][2];
    
    result.d1 = sqrt(result.x1*result.x1+result.y1*result.y1+result.z1*result.z1);
    result.d2 = sqrt(result.x2*result.x2+result.y2*result.y2+result.z2*result.z2);

    return result;
}

STrackedObject CTransformation::transform(SSegment segment)
{
    float x,y,x1,x2,y1,y2,major,minor,v0,v1;
    STrackedObject result;

    /* transformation to the canonical camera coordinates, see 4.1 of [1]*/
    x = segment.x;
    y = segment.y;
    transformXY(&x,&y);
    //major axis
    //vertices in image coords
    x1 = segment.x+segment.v0*segment.m0*2;
    x2 = segment.x-segment.v0*segment.m0*2;
    y1 = segment.y+segment.v1*segment.m0*2;
    y2 = segment.y-segment.v1*segment.m0*2;
    //vertices in canonical camera coords 
    transformXY(&x1,&y1);
    transformXY(&x2,&y2);
    //semiaxes length 
    major = sqrt((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2))/2.0;
    v0 = (x2-x1)/major/2.0;
    v1 = (y2-y1)/major/2.0;
    //printf("AAA: %f %f\n",sqrt((x-x1)*(x-x1)+(y-y1)*(y-y1))-sqrt((x-x2)*(x-x2)+(y-y2)*(y-y2)),major);

    //the minor axis 
    //vertices in image coords
    x1 = segment.x+segment.v1*segment.m1*2;
    x2 = segment.x-segment.v1*segment.m1*2;
    y1 = segment.y-segment.v0*segment.m1*2;
    y2 = segment.y+segment.v0*segment.m1*2;
    //vertices in canonical camera coords 
    transformXY(&x1,&y1);
    transformXY(&x2,&y2);
    //minor axis length 
    minor = sqrt((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2))/2.0;
    //printf("BBB: %f %f\n",sqrt((x-x1)*(x-x1)+(y-y1)*(y-y1))-sqrt((x-x2)*(x-x2)+(y-y2)*(y-y2)),minor);

    /*construct the ellipse characteristic equation, see 4.2 of [1], prepare coefs for Eq */
    float a,b,c,d,e,f;
    a = v0*v0/(major*major)+v1*v1/(minor*minor);
    b = v0*v1*(1.0/(major*major)-1.0/(minor*minor));
    c = v0*v0/(minor*minor)+v1*v1/(major*major);
    d = (-x*a-b*y);
    e = (-y*c-b*x);
    f = (a*x*x+c*y*y+2*b*x*y-1.0);
    double data[] ={a,b,d,b,c,e,d,e,f};	//matrix conic coefficients, see 4.2 of [1]

    /*transformation to camera-centric or user-defined coordinate frames*/
    //3D->2D homography, see 4.4.2 of [1]
    if (transformType == TRANSFORM_2D){
        // TODO is calcEigen() really necessary
        //for debug only  // which part is for debug only???
        result = calcEigen(data);
        result.x = x;
        result.y = y;
        result = transform2D(result);

        result.yaw = atan2(segment.v0,segment.v1);  // why calculating twice
        result.yaw = segment.angle;
        result.ID = segment.ID;
    }
    //camera-centric coordinate frame, see 4.3 and 4.4 of [1]
    if (transformType == TRANSFORM_NONE){
        result = calcEigen(data);
        result.yaw = segment.angle;
    }
    //user-defined 3D coordinate system, see 4.4.1 of [1]
    if (transformType == TRANSFORM_3D){
        result = calcEigen(data);
        result = transform3D(result);
    }
    //alternative calculation of 3D->3D transform
    if (transformType == TRANSFORM_4D){
        result = calcEigen(data);
        result = transform4D(result);
    }
    //camera centric + error estimate
    if (transformType == TRANSFORM_INV){
        result = calcEigen(data);
    }

    return result;
}

