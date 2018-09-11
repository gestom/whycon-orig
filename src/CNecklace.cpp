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
    if(! probabilistic) return idArray[sequence];

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
    tmp[0].m0 = 0.33/0.70*segment->m0;
    tmp[0].m1 = 0.33/0.70*segment->m1;
    tmp[0].v0 = segment->v0;
    tmp[0].v1 = segment->v1;

    tmp[1] = tmp[0];
    tmp[1].x = object->segX2;
    tmp[1].y = object->segY2;

    float sum[2];
    float variance[2];

    int step = image->bpp;

    int pos;
    float x[2][idSamples];
    float y[2][idSamples];
    float signal[2][idSamples];
    float differ[2][idSamples];
    float smooth[2][idSamples];
    int segmentWidth = idSamples/length/2;
    //calculate appropriate positions
    for(int i=0;i<2;i++){

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

        int toothLen = 0;
        int err;
        int teeth = 0;
        for(int j=0;j<idSamples;j++){
            if(smooth[i][j] == 0){
                toothLen++;
                if((j+1 < idSamples && smooth[i][j+1] != 0) || j+1 == idSamples){
                    err = (toothLen-segmentWidth > segmentWidth) ? toothLen-2*segmentWidth : toothLen-segmentWidth;
                    sum[i] = err*err;
                    toothLen = 0;
                    teeth++;
                }
            }else{
                toothLen++;
                if((j+1 < idSamples && smooth[i][j+1] != 1) || j+1 == idSamples){
                    err = (toothLen-segmentWidth > segmentWidth) ? toothLen-2*segmentWidth : toothLen-segmentWidth;
                    sum[i] = err*err;
                    toothLen = 0;
                    teeth++;
                }
            }
        }

        variance[i] = sum[i] / teeth;
    }

    if(variance[0] < variance[1]){
        segIdx = 0;
        segment->x = object->segX1;
        segment->y = object->segY1;
        object->x = object->x1;
        object->y = object->y1;
        object->z = object->z1;
        object->pitch = object->pitch1;
        object->roll = object->roll1;
        object->yaw = object->yaw1;
    }else{
        segIdx = 1;
        segment->x = object->segX2;
        segment->y = object->segY2;
        object->x = object->x2;
        object->y = object->y2;
        object->z = object->z2;
        object->pitch = object->pitch2;
        object->roll = object->roll2;
        object->yaw = object->yaw2;
    }

    //find the edge's locations
    int maxIndex = 0;
    int numEdges = 0;
    float sx,sy;
    sx = sy = 0;
    if (smooth[segIdx][idSamples-1] != smooth[segIdx][0])sx = 1;
    for (int a = 1;a<idSamples;a++){
        if (smooth[segIdx][a] != smooth[segIdx][a-1]){
            sx += cos(2*M_PI*a/segmentWidth);
            sy += sin(2*M_PI*a/segmentWidth);
            if (debug) printf("%i ",a);
        }
    }
    if (debug) printf("\n");
    maxIndex = atan2(sy,sx)/2/M_PI*segmentWidth+segmentWidth/2;

    //determine raw code
    char code[length*4];
    for (int i = 0;i<length*2;i++) code[i] = smooth[segIdx][(maxIndex+i*segmentWidth)%idSamples]+'0';

    code[length*2] = 0;

    //determine the control edges' positions
    int edgeIndex = 0;
    for (unsigned int a=0;a<length*2;a++){
        int p = (a+1)%(length*2);
        if (code[a]=='0' && code[p]=='0') edgeIndex = a;
    }
    char realCode[length*4];
    edgeIndex = 1-(edgeIndex%2);
    int ID = 0;
    for (unsigned int a=0;a<length;a++){
        realCode[a] = code[edgeIndex+2*a];
        if (realCode[a] == 'X') ID = -1;
        if (ID > -1){
            ID = ID*2;
            if (realCode[a]=='1') ID++;
        }
    }
    realCode[length] = 0;
    SNecklace result = get(ID);
    segment->angle = 2*M_PI*(-(float)maxIndex/idSamples+(float)result.rotation/length)+atan2(segment->v1,segment->v0)+1.5*M_PI/length;
    segment->angle = 2*M_PI*(-(float)maxIndex/idSamples-(float)edgeIndex/length/2.0+(float)result.rotation/length)+atan2(segment->v1,segment->v0);//+1.5*M_PI/length;
    while (segment->angle > +M_PI)  segment->angle-=2*M_PI;
    while (segment->angle < -M_PI)  segment->angle+=2*M_PI;
    //printf("CODE %i %i %i %i %s %s %.3f %.3f\n",result.id,result.rotation,maxIndex,ID,realCode,code,segment->angle,atan2(segment->v1,segment->v0));
    if (debug){
        printf("CODE %i %i %.3f\n",result.id,maxIndex,segment->angle);
        printf("Realcode %s %i %s\n",code,edgeIndex,realCode);
        printf("ORIG: ");
        for (int a = 0;a<idSamples;a++)printf("%.2f ",signal[segIdx][a]);
        printf("\n");
        //for (int a = 0;a<idSamples;a++)printf("%.2f ",differ[a]);
        //printf("\n");
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

    return result.id++;
}
