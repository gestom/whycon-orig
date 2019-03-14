#include "CNecklace.h"

CNecklace::CNecklace(int bits, int samples, int minimalHamming)
{
    length = bits;
    idLength = pow(2,length); 
    idArray = (SNecklace*)malloc(sizeof(SNecklace)*idLength);
    idSamples = samples;
    int currentID = 0;
    maxID = 0;
    int tempID,bit,rotations;
    int minHam = 1000;
    int hamindex = 1000;
    int ham = 1000;
    debug = false;
    debugSegment = false;

    /*for every possible id*/
    for (int id = 0;id<idLength;id++){
        /*check if there is a lower number that could be created by bitshifting it*/
        tempID  = id;
        rotations = 0;
        int cached [length - 1];
        bool isSymmetrical = false;
        minHam = 1000;
        if (debug) printf("Testing %i\n",tempID);
        do{
            hamindex = getMinimalHamming(tempID,id);
            ham = getHamming(tempID,hamindex);
            if (minHam > ham){
                minHam = ham;
                if (minHam == 0){
                    idArray[id].id = idArray[hamindex].id; 
                    idArray[id].rotation = idArray[hamindex].rotation + rotations; 
                    if (debug) printf("AAA %i %i %i \n",id,idArray[id].id,idArray[id].rotation);
                }
            }
            bit = tempID%2;
            tempID=tempID/2+bit*pow(2,length-1);

            if(bit || id == 0){
                for (int i = 0; i < rotations && !isSymmetrical; i++){
                    // check for symmetry
                    if (cached[i] == tempID){
                        isSymmetrical = true;
                    }
                }
            }
            cached[rotations] = tempID; 
            //ham = getMinimalHamming(tempID,id);
            //if (minHam > ham) minHam = ham;
        }while (rotations++<length-1 && !isSymmetrical);

        if (minHam >= minimalHamming && !isSymmetrical){
            if (debug) printf("Adding %i %i\n",currentID,id);
            idArray[id].id = currentID++;
            idArray[id].rotation = 0;
            idArray[id].hamming = minHam;
        }
        else if (minHam > 0){
            idArray[id].id = -1; 
            idArray[id].rotation = -1;
            idArray[id].hamming = minHam;
        }
        if(isSymmetrical){
            idArray[id].id = -1;
            idArray[id].rotation = -1;
        } 
    }

    //idArray[idLength-1].id = 0;
    //idArray[idLength-1].rotation = 0;
    unknown.id = -1;
    unknown.rotation = -1;

    for (int i = 0; i < idLength; i++) if(maxID < idArray[i].id) maxID = idArray[i].id;
    probArray = (float*)malloc(sizeof(float)*maxID);
    for (int id = 0; id < maxID; id++) probArray[id] = 1./(float)maxID;
}

CNecklace::~CNecklace()
{
    free(idArray);
    free(probArray);
}

int CNecklace::getHamming(int a, int b)
{
    int aa = a;
    int bb = b;
    int ham = 0;
    do {
        if (a%2 != b%2) ham++;
        a = a/2;
        b = b/2;
    }while (a > 0 || b > 0);
    if (debug) printf("Hamming %i %i is %i\n",aa,bb,ham);
    return ham;
}

int CNecklace::getMinimalHamming(int a,int len)
{
    int minDist = 10000;
    int mindex = 10000;
    for (int i = 1;i<len;i++){
        if (get(i,false).rotation == 0){
            int m = getHamming(a,i);
            if (minDist > m){
                minDist = m;
                mindex = i;
                //if (minDist < 3) printf("%i is same as %i\n",a,i);
            }
        }
    }
    if (debug) printf("Minimal hamming of %i is %i\n",a,minDist);
    return mindex;
    //return minDist;
}

int CNecklace::verifyHamming(int a[],int bits,int len)
{
    int overAll = 10000;
    for (int i = 0;i<len;i++){
        for (int j = 0;j<len;j++){
            int minimal = 10000;
            if (i!=j){
                int bit;
                int tempID = a[j];
                int distance;
                if (debug) printf("Testing %i vs %i\n",a[i],a[j]);
                for (int r = 0;r<bits;r++){
                    distance = getHamming(a[i],tempID);
                    if (debug) printf("Test %i %i %i\n",a[i],tempID,distance);
                    if (minimal > distance) minimal = distance;
                    bit = tempID%2;
                    tempID=tempID/2+bit*pow(2,length-1);
                }
            }
            if (debug) printf("%i vs %i has %i\n",a[i],a[j],minimal);
            if (overAll > minimal) overAll = minimal;
        }
    }
    return overAll;
}

SNecklace CNecklace::get(int sequence, bool probabilistic, float confidence)
{
    if (sequence <= 0 || sequence >= idLength) return unknown;
    if (!probabilistic) return idArray[sequence];

    float oe = observationEstimation(confidence);
    float o=.0;

    for (int i = 0; i < maxID; i++){
        if (idArray[sequence].id == i) o += (oe*probArray[i]);
        else o += ((1.0-oe)/(float)(maxID-1)*probArray[i]);
    }

    for (int i = 0; i < maxID; i++){
        if (idArray[sequence].id == i ) probArray[i] = (oe/o)*probArray[i];
        else probArray[i] = (1.0-oe)/(float)(maxID-1)/o*probArray[i];

        if(probArray[i] <= 1./maxID){
            if (debug) printf("Confidence value too small, changing %.9f to %.9f\n", probArray[i], 1./maxID);
            probArray[i] = 1./maxID;
        }

        if(probArray[i] > 1.-(1./maxID)){
            if(debug) printf("Confidence value too big, changing %.9f to %.9f\n", probArray[i], 1.-(1./maxID));
            probArray[i] = 1.-(1./maxID);
        }
    }
    SNecklace toReturn = idArray[sequence];
    toReturn.id = getEstimatedID();
    return toReturn;
}

float CNecklace::observationEstimation(float confidence)
{
    float a = 400.;
    float s = 80.;
    return atan2((confidence-a),s)*((1.-(1./(float)maxID))/M_PI)+(((float)maxID+1.)/(2.*(float)maxID));
}

int CNecklace::getEstimatedID()
{
    int hp = 0;
    for (int id = 0; id < maxID; id++){
        if (debug) printf("%i %f\n", id, probArray[id]);
        if(probArray[id] > probArray[hp]) hp = id;
    }
    // printf("%i\n", hp+1);
    return hp;
}

int CNecklace::identifySegment(SSegment *segment, STrackedObject *object, CRawImage* image)
{
    int segIdx = 0;
    SSegment tmp[2];
    tmp[0].x = object->segX1;
    tmp[0].y = object->segY1;
    tmp[0].m0 = 0.33/0.70*segment->m0; //why those nums?
    tmp[0].m1 = 0.33/0.70*segment->m1;
    tmp[0].v0 = segment->v0;
    tmp[0].v1 = segment->v1;

    tmp[1] = tmp[0];
    tmp[1].x = object->segX2;
    tmp[1].y = object->segY2;

    float sum[2];
    sum[0]=sum[1]=0;
    float variance[2];
    variance[0]=variance[1]=0;

    int step = image->bpp;

    int pos;
    float x[2][idSamples];
    float y[2][idSamples];
    float signal[2][idSamples];
    float smooth[2][idSamples];
    int segmentWidth = idSamples/length/2;

    int maxIdx[2];
    float numPoints[2];
    int maxIndex = 0;

    char code[2][length*4];
    int edgeIndex[2];
    char realCode[2][length*4];
    int ID[2];
    SNecklace result[2];

    for(int i=0;i<2;i++){

        //calculate appropriate positions
        float topY = 0;
        int topIndex = 0;
        for (int a = 0;a<idSamples;a++){
            x[i][a] = tmp[i].x+(tmp[i].m0*cos((float)a/idSamples*2*M_PI)*tmp[i].v0+tmp[i].m1*sin((float)a/idSamples*2*M_PI)*tmp[i].v1)*2.0;
            y[i][a] = tmp[i].y+(tmp[i].m0*cos((float)a/idSamples*2*M_PI)*tmp[i].v1-tmp[i].m1*sin((float)a/idSamples*2*M_PI)*tmp[i].v0)*2.0;
        }
        //retrieve the image brightness on these using bilinear transformation
        float gx,gy;
        int px,py;
        unsigned char* ptr = image->data;
        for (int a = 0;a<idSamples;a++){
            px = x[i][a];
            py = y[i][a];
            gx = x[i][a]-px;
            gy = y[i][a]-py;
            pos = (px+py*image->width);

            /*detection from the image*/
            signal[i][a]  = ptr[(pos+0)*step+0]*(1-gx)*(1-gy)+ptr[(pos+1)*step+0]*gx*(1-gy)+ptr[(pos+image->width)*step+0]*(1-gx)*gy+ptr[step*(pos+image->width+1)+0]*gx*gy;
            signal[i][a] += ptr[(pos+0)*step+1]*(1-gx)*(1-gy)+ptr[(pos+1)*step+1]*gx*(1-gy)+ptr[(pos+image->width)*step+1]*(1-gx)*gy+ptr[step*(pos+image->width+1)+1]*gx*gy;
            signal[i][a] += ptr[(pos+0)*step+2]*(1-gx)*(1-gy)+ptr[(pos+1)*step+2]*gx*(1-gy)+ptr[(pos+image->width)*step+2]*(1-gx)*gy+ptr[step*(pos+image->width+1)+2]*gx*gy;
        }

        //binarize the signal
        float avg = 0;
        for (int a = 0;a<idSamples;a++) avg += signal[i][a];
        avg = avg/idSamples;
        for (int a = 0;a<idSamples;a++) if (signal[i][a] > avg) smooth[i][a] = 1; else smooth[i][a] = 0;

        //find the edge's locations
        int numEdges = 0;  // variable not used at all
        float sx,sy;

        sx = sy = 0;
        numPoints[i]=0;
        if (smooth[i][idSamples-1] != smooth[i][0])sx = 1;
        for (int a = 1;a<idSamples;a++){
            if (smooth[i][a] != smooth[i][a-1]){
                sx += cos(2*M_PI*a/segmentWidth);
                sy += sin(2*M_PI*a/segmentWidth);
                numPoints[i]++;
                if (debugSegment) printf("%i ",a);
            }
        }
        if (debugSegment) printf("\n");
        maxIdx[i] = atan2(sy,sx)/2/M_PI*segmentWidth+segmentWidth/2;

        float meanX = sx / numPoints[i];
        float meanY = sy / numPoints[i];
        float errX, errY;
        sx=sy=0;
        if (smooth[i][idSamples-1] != smooth[i][0])sx = 1;
        for (int a = 1;a<idSamples;a++){
            if (smooth[i][a] != smooth[i][a-1]){
                sx = cos(2*M_PI*a/segmentWidth);
                sy = sin(2*M_PI*a/segmentWidth);
                errX = sx - meanX;
                errY = sy - meanY;
                sum[i] += errX*errX + errY*errY;
            }
        }
        variance[i] = sum[i] / numPoints[i];

        //determine raw code
        for (int a = 0;a<length*2;a++) code[i][a] = smooth[i][(maxIdx[i]+a*segmentWidth)%idSamples]+'0';

        code[i][length*2] = 0;

        //determine the control edges' positions
        edgeIndex[i] = 0;
        for (unsigned int a=0;a<length*2;a++){
            int p = (a+1)%(length*2);
            if (code[i][a]=='0' && code[i][p]=='0') edgeIndex[i] = a;
        }
        edgeIndex[i] = 1-(edgeIndex[i]%2);
        ID[i] = 0;
        for (unsigned int a=0;a<length;a++){
            realCode[i][a] = code[i][edgeIndex[i]+2*a];
//            if (realCode[i][a] == 'X') ID[i] = -1;
            if (ID[i] > -1){
                ID[i] = ID[i]*2;
                if (realCode[i][a]=='1') ID[i]++;
            }
        }
        realCode[i][length] = 0;
        result[i] = get(ID[i]);
    }

    if(debugSegment){
        printf("%d numPoints %f id %d variance %f ID %d\n",0,numPoints[0],result[0].id,variance[0],ID[0]);
        printf("%d numPoints %f id %d variance %f ID %d\n",1,numPoints[1],result[1].id,variance[1],ID[1]);
    }

    if(numPoints[0]==numPoints[1]){
        if(result[0].id==result[1].id){
            if(variance[0] < variance[1]){
                segIdx = 0;
            }else{
                segIdx = 1;
            }
        }else if(result[0].id > -100 && result[1].id > -100){
            if(variance[0] < variance[1]){
                segIdx = 0;
            }else{
                segIdx = 1;
            }
        }else if(result[0].id > -100){
            segIdx = 0;
        }else{
            segIdx = 1;
        }
    }else if(numPoints[0] > length && numPoints[0] < 2*length && numPoints[1] > length && numPoints[1] < 2*length){
        if(result[0].id==result[1].id){
            if(variance[0] < variance[1]){
                segIdx = 0;
            }else{
                segIdx = 1;
            }
        }else if(result[0].id > -100 && result[1].id > -100){
            if(variance[0] < variance[1]){
                segIdx = 0;
            }else{
                segIdx = 1;
            }
        }else if(result[0].id > -100){
            segIdx = 0;
        }else{
            segIdx = 1;
        }
    }else if(numPoints[0] > length && numPoints[0] < 2*length){
        segIdx = 0;
    }else{
        segIdx = 1;
    }

    if(segIdx == 0){
        segment->x = object->segX1;
        segment->y = object->segY1;
        object->x = object->x1;
        object->y = object->y1;
        object->z = object->z1;
        object->pitch = object->pitch1;
        object->roll = object->roll1;
        object->yaw = object->yaw1;
    }else{
        segment->x = object->segX2;
        segment->y = object->segY2;
        object->x = object->x2;
        object->y = object->y2;
        object->z = object->z2;
        object->pitch = object->pitch2;
        object->roll = object->roll2;
        object->yaw = object->yaw2;
    }

    if(debugSegment){
        printf("segIdx %d ",segIdx);
        printf("numPoints %f id %d variance %f ID %d\n\n",numPoints[segIdx],result[segIdx].id,variance[segIdx],ID[segIdx]);
    }

    maxIndex = maxIdx[segIdx];
    
    segment->angle = 2*M_PI*(-(float)maxIndex/idSamples+(float)result[segIdx].rotation/length)+atan2(segment->v1,segment->v0)+1.5*M_PI/length;
    segment->angle = 2*M_PI*(-(float)maxIndex/idSamples-(float)edgeIndex[segIdx]/length/2.0+(float)result[segIdx].rotation/length)+atan2(segment->v1,segment->v0);//+1.5*M_PI/length;
    while (segment->angle > +M_PI)  segment->angle-=2*M_PI;
    while (segment->angle < -M_PI)  segment->angle+=2*M_PI;
    if (debugSegment){
        printf("CODE %i %i %.3f\n",result[segIdx].id,maxIndex,segment->angle);
        printf("Realcode %s %i %s\n",code[segIdx],edgeIndex[segIdx],realCode[segIdx]);
        printf("ORIG: ");
        for (int a = 0;a<idSamples;a++)printf("%.2f ",signal[segIdx][a]);
        printf("\n");
        for (int a = 0;a<idSamples;a++)printf("%.2f ",smooth[segIdx][a]);
        printf("\n");
    }

    for (int a = 0;a<idSamples;a++){
        pos = ((int)x[segIdx][a]+((int)y[segIdx][a])*image->width);
        if (pos > 0 && pos < image->width*image->height){
            image->data[step*pos+0] = 0;
            image->data[step*pos+1] = (unsigned char)(255.0*a/idSamples);
            image->data[step*pos+2] = 0;
        }
    }

    return result[segIdx].id++;
}
