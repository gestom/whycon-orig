#include "CCircleDetect.h"

#define min(a,b) ((a) < (b) ? (a) : (b))
#define max(a,b) ((a) > (b) ? (a) : (b))

int* CCircleDetect::buffer = NULL;
int* CCircleDetect::queue = NULL;

//Variable initialization
CCircleDetect::CCircleDetect(int wi, int he, bool id, int bits, int samples, int dist) {
    idBits = bits;
    idSamples = samples;
    hammingDist = dist;
    identify = id;
    step = -1;
    ID = -1;
    enableCorrections = false;
    lastTrackOK = false;
    debug = false;
    draw = true;
    maxFailed = 0;
    minSize = 100;
    maxThreshold = 256;
    centerDistanceToleranceRatio = 0.01;
    centerDistanceToleranceAbs = 5;
    circularTolerance = 0.8; // 1.4
    ratioTolerance = 0.8; // 1.4
    threshold = maxThreshold / 2;
    numFailed = maxFailed;
    track = true;
    circularityTolerance = 0.02;

    //initialization - fixed params
    width = wi; 
    height = he;
    len = width*height;
    ownBuffer = false;
    if (buffer == NULL) {
        ownBuffer = true;
        buffer = (int*) malloc(len * sizeof (int));
        queue = (int*) malloc(len * sizeof (int));
        SSegment dummy;
        bufferCleanup(dummy);
    }
    diameterRatio = 33.0 / 70.0; //inner vs. outer circle diameter
    float areaRatioInner_Outer = diameterRatio*diameterRatio;
    outerAreaRatio = M_PI * (1.0 - areaRatioInner_Outer) / 4;
    innerAreaRatio = M_PI / 4.0;
    areasRatio = (1.0 - areaRatioInner_Outer) / areaRatioInner_Outer;
    sizer = sizerAll = 0;
}

void CCircleDetect::reconfigure(float ict,float fct,float art,float cdtr,float cdta, bool id, int minS)
{
    circularTolerance = ict/100.0;
    circularityTolerance = fct/100.0;
    ratioTolerance = 1+art/100.0;
    centerDistanceToleranceRatio = cdtr/100.0;
    centerDistanceToleranceAbs = cdta;
    minSize = minS;
    identify = id;
}

int CCircleDetect::adjustDimensions(int wi, int he) {
    width = wi;
    height = he;
    len = width*height;
    free(buffer);
    free(queue);
    buffer = (int*) malloc(len * sizeof (int));
    queue = (int*) malloc(len * sizeof (int));
    SSegment dummy;
    bufferCleanup(dummy);
}

CCircleDetect::~CCircleDetect() {
    if (ownBuffer) {
        free(buffer);
        free(queue);
    }
}

bool CCircleDetect::changeThreshold() {
    int div = 1;
    int dum = numFailed;
    while (dum > 1) {
        dum = dum / 2;
        div *= 2;
    }
    int t_step = 256 / div;
    threshold = (t_step * (numFailed - div) + t_step / 2);
    if (debug) fprintf(stdout, "Threshold: %i %i %i\n", div, numFailed, threshold);
    return t_step > 16;
}

bool CCircleDetect::examineSegment(CRawImage *image, SSegment *segmen, int ii, float areaRatio) {
    int vx, vy;
    queueOldStart = queueStart;
    int position = 0;
    int pos;
    bool result = false;
    int type = buffer[ii];
    int maxx, maxy, minx, miny;

    buffer[ii] = ++numSegments;
    segmen->x = ii % width;
    segmen->y = ii / width;
    minx = maxx = segmen->x;
    miny = maxy = segmen->y;
    segmen->valid = false;
    segmen->round = false;
    //push segment coords to the queue
    queue[queueEnd++] = ii;
    //and until queue is empty
    while (queueEnd > queueStart) {
        //pull the coord from the queue
        position = queue[queueStart++];
        //search neighbours
        pos = position + 1;
        if (buffer[pos] == 0) {
            ptr = &image->data[pos * step];
            buffer[pos] = (ptr[0] > threshold) - 2;
        }
        if (buffer[pos] == type) {
            queue[queueEnd++] = pos;
            maxx = max(maxx, pos % width);
            buffer[pos] = numSegments;
        }
        pos = position - 1;
        if (buffer[pos] == 0) {
            ptr = &image->data[pos * step];
            buffer[pos] = (ptr[0] > threshold) - 2;
        }
        if (buffer[pos] == type) {
            queue[queueEnd++] = pos;
            minx = min(minx, pos % width);
            buffer[pos] = numSegments;
        }
        pos = position - width;
        if (buffer[pos] == 0) {
            ptr = &image->data[pos * step];
            buffer[pos] = (ptr[0] > threshold) - 2;
        }
        if (buffer[pos] == type) {
            queue[queueEnd++] = pos;
            miny = min(miny, pos / width);
            buffer[pos] = numSegments;
        }
        pos = position + width;
        if (buffer[pos] == 0) {
            ptr = &image->data[pos * step];
            buffer[pos] = (ptr[0] > threshold) - 2;
        }
        if (buffer[pos] == type) {
            queue[queueEnd++] = pos;
            maxy = max(maxy, pos / width);
            buffer[pos] = numSegments;
        }
    }

    //once the queue is empty, i.e. segment is complete, we compute its size 
    segmen->size = queueEnd - queueOldStart;
    if (segmen->size > minSize) {
        //and if its large enough, we compute its other properties 
        segmen->maxx = maxx;
        segmen->maxy = maxy;
        segmen->minx = minx;
        segmen->miny = miny;
        segmen->type = -type;
        vx = (segmen->maxx - segmen->minx + 1);
        vy = (segmen->maxy - segmen->miny + 1);
        segmen->x = (segmen->maxx + segmen->minx) / 2;
        segmen->y = (segmen->maxy + segmen->miny) / 2;
        segmen->roundness = vx * vy * areaRatio / segmen->size;
        //we check if the segment is likely to be a ring
        if (segmen->roundness - circularTolerance < 1.0 && segmen->roundness + circularTolerance > 1.0 || true) {//TODO
            //if its round, we compute yet another properties 
            segmen->round = true;
            segmen->mean = 0;
            for (int p = queueOldStart; p < queueEnd; p++) {
                pos = queue[p];
                segmen->mean += image->data[pos * step];
            }
            segmen->mean = segmen->mean / segmen->size;
            result = true;
        }
    }
    return result;
}

void CCircleDetect::bufferCleanup(SSegment init) {
    int pos = (height - 1) * width;
    if (init.valid == false || track == false || lastTrackOK == false) {
        memset(buffer, 0, sizeof (int)*len);
        for (int i = 0; i < width; i++) {
            buffer[i] = -1000;
            buffer[pos + i] = -1000;
        }
        for (int i = 0; i < height; i++) {
            buffer[width * i] = -1000;
            buffer[width * i + width - 1] = -1000;
        }
    } else {
        int pos, ix, ax, iy, ay;
        ix = max(init.minx - 2, 1);
        ax = min(init.maxx + 2, width - 2);
        iy = max(init.miny - 2, 1);
        ay = min(init.maxy + 2, height - 2);
        for (int y = iy; y < ay; y++) {
            pos = y*width;
            for (int x = ix; x < ax; x++) buffer[pos + x] = 0;
        }
    }
}

SSegment CCircleDetect::calcSegment(SSegment segment,int size,long int x,long int y,long int cm0,long int cm1,long int cm2)
{
    float cm0f,cm1f,cm2f,fm0,fm1,fm2,f0,f1;
    SSegment result = segment;
    float sx = (float)x/size;
    float sy = (float)y/size;
    cm0f = (cm0 - sx*sx*size);
    cm1f = (cm1 - sx*sy*size);
    cm2f = (cm2 - sy*sy*size);
    fm0 = cm0f/size;
    fm1 = cm1f/size;
    fm2 = cm2f/size;
    float det = (fm0+fm2)*(fm0+fm2)-4*(fm0*fm2-fm1*fm1);
    if (det > 0) det = sqrt(det); else det = 0;
    f0 = ((fm0+fm2)+det)/2;
    f1 = ((fm0+fm2)-det)/2;
    result.x = sx;
    result.y = sy;
    result.m0 = sqrt(f0);
    result.m1 = sqrt(f1);
    if (fm1 != 0){
        result.v0 = -fm1/sqrt(fm1*fm1+(fm0-f0)*(fm0-f0));
        result.v1 = (fm0-f0)/sqrt(fm1*fm1+(fm0-f0)*(fm0-f0));
    }else{
        result.v0 = result.v1 = 0;
        if (fm0 > fm2) result.v0 = 1.0; else result.v1 = 1.0;
    }
    //printf("An: %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f\n",det,result.v0,result.v1,fm0,fm1,fm2,f0,f1);
    return result;
}

SSegment CCircleDetect::findSegment(CRawImage* image, SSegment init) {
    numSegments = 0;
    int pos = 0;
    int ii = 0;
    int start = 0;
    bool cont = true;

    if (image->width != width || image->height != height) {
        adjustDimensions(image->width, image->height);
        init.valid = false;
    }
    step = image->bpp;

    //bufferCleanup(init);
    if (init.valid && track){
        ii = ((int)init.y)*image->width+init.x;
        start = ii;
    }
    while (cont) 
    {
        if (buffer[ii] == 0){
            ptr = &image->data[ii*step];
            if (ptr[0] < threshold) buffer[ii] = -2;
        }
        if (buffer[ii] == -2){
            //new segment found
            queueEnd = 0;
            queueStart = 0;
            //if the segment looks like a ring, we check its inside area
            if (examineSegment(image,&outer,ii,outerAreaRatio)){
                pos = outer.y*image->width+outer.x;
                if (buffer[pos] == 0){
                    ptr = &image->data[pos*step];
                    buffer[pos]= (ptr[0] >= threshold)-2;
                }   
                if (buffer[pos] == -1){
                    if (examineSegment(image,&inner,pos,innerAreaRatio)){
                        //the inside area is a circle. now what is the area ratio of the black and white ? also, are the circles concentric ?

                        if (debug) printf("Area ratio should be %.3f, but is %.3f, that is %.0f%% off. ",areasRatio,(float)outer.size/inner.size,(1-outer.size/areasRatio/inner.size)*100);
                        if ((float)outer.size/areasRatio/(float)inner.size - ratioTolerance < 1.0 && (float)outer.size/areasRatio/(float)inner.size + ratioTolerance > 1.0){ 
                            if (debug) fprintf(stdout,"Segment BW ratio OK.\n");
                            if (debug) fprintf(stdout,"Concentricity %.0f %.0f %.0f %.0f.",inner.x,inner.y, outer.x,outer.y);
                            if((abs(inner.x-outer.x) <= centerDistanceToleranceAbs+centerDistanceToleranceRatio*((float)(outer.maxx-outer.minx))) &&
                                    (abs(inner.y-outer.y) <= centerDistanceToleranceAbs+centerDistanceToleranceRatio*((float)(outer.maxy-outer.miny))))

                            {
                                if (debug) fprintf(stdout,"Concentricity OK.\n");
                                long int six,siy,tx,ty,cm0,cm1,cm2;
                                six=siy=cm0=cm1=cm2=0;

                                for (int p = queueOldStart;p<queueEnd;p++){
                                    pos = queue[p];
                                    tx = pos%image->width;
                                    ty = pos/image->width;
                                    six += tx;
                                    siy += ty;
                                    cm0+=tx*tx; 
                                    cm1+=tx*ty;
                                    cm2+=ty*ty; 
                                }
                                inner=calcSegment(inner,queueEnd-queueOldStart,six,siy,cm0,cm1,cm2);
                                //inner.x = (float)six/(queueEnd-queueOldStart);
                                //inner.y = (float)siy/(queueEnd-queueOldStart);

                                for (int p = 0;p<queueOldStart;p++){
                                    pos = queue[p];
                                    tx = pos%image->width;
                                    ty = pos/image->width;
                                    six += tx;
                                    siy += ty;
                                    cm0+=tx*tx; 
                                    cm1+=tx*ty;
                                    cm2+=ty*ty; 
                                }
                                outer=calcSegment(outer,queueEnd,six,siy,cm0,cm1,cm2);
                                outer.bwRatio = (float)inner.size/outer.size;

                                sizer+=outer.size + inner.size; //for debugging
                                sizerAll+=len;                                  //for debugging
                                float circularity = M_PI*4*(outer.m0)*(outer.m1)/queueEnd;
                                if (debug) fprintf(stdout,"Segment circularity: %i %03f %03f \n",queueEnd,M_PI*4*(outer.m0)*(outer.m1)/queueEnd,M_PI*4*(outer.m0)*(outer.m1));
                                if (circularity-1.0 < circularityTolerance && circularity-1.0 > -circularityTolerance){

                                    //chromatic aberation correction
                                    if (enableCorrections){
                                        float r = diameterRatio*diameterRatio;
                                        float m0o = outer.m0;
                                        float m1o = outer.m1;
                                        float ratio = (float)inner.size/(outer.size + inner.size);
                                        float m0i = sqrt(ratio)*m0o;
                                        float m1i = sqrt(ratio)*m1o;
                                        float a = (1-r);
                                        float b = -(m0i+m1i)-(m0o+m1o)*r;
                                        float c = (m0i*m1i)-(m0o*m1o)*r;
                                        float t = (-b-sqrt(b*b-4*a*c))/(2*a);
                                        //plc second version
                                        //float t0 = (-b-sqrt(b*b-4*a*c))/(2*a);    
                                        //float t1 = (-b+sqrt(b*b-4*a*c))/(2*a);
                                        //if (m1i - t0 > 0 && m1i - t1 >0) printf("AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA\n"); 
                                        //float t0 = (m0i-diameterRatio*m0o)/(1+diameterRatio);
                                        //float t1 = (m1i-diameterRatio*m1o)/(1+diameterRatio);
                                        m0i-=t;m1i-=t;m0o+=t;m1o+=t;
                                        //fprintf(stdout,"UUU: %f R: %f %f R: %f %f\n",t,m1i/m1o*0.14,m0i/m0o*0.14,(m0o*m1o-m0i*m1i)/(m0i*m1i),(0.14*0.14-0.05*0.05)/(0.05*0.05));
                                        inner.m0 = m0o;
                                        inner.m1 = m1o;
                                    }
                                    outer.size = outer.size+inner.size;
                                    outer.horizontal = outer.x-inner.x;
                                    if (fabs(inner.v0*outer.v0+inner.v1*outer.v1) > 0.5){
                                        outer.r0 = inner.m0/outer.m0;
                                        outer.r1 = inner.m1/outer.m1;
                                    }else{
                                        outer.r0 = inner.m1/outer.m0;
                                        outer.r1 = inner.m0/outer.m1;
                                    }

                                    float orient = atan2(outer.y-inner.y,outer.x-inner.x);
                                    outer.angle = atan2(outer.v1,outer.v0);
                                    if (debug) printf("Angle: %.3f %.3f \n",outer.angle,orient);
                                    if (fabs(normalizeAngle(outer.angle-orient)) > M_PI/2) outer.angle = normalizeAngle(outer.angle+M_PI);

                                    outer.valid = inner.valid = true;
                                    threshold = (outer.mean+inner.mean)/2;
                                    if (track) ii = start -1;
                                }else{
                                    if (track && init.valid){
                                        ii = start -1;
                                        if (debug) fprintf(stdout,"Segment failed circularity test.\n");
                                    }
                                }
                            }else{
                                if (track && init.valid){
                                    ii = start -1;
                                    if (debug) fprintf(stdout,"Segment failed concentricity test.\n");
                                }
                            }
                        }else{
                            //tracking failed
                            if (track && init.valid){
                                ii = start -1;
                                if (debug) fprintf(stdout,"Segment failed BW test.\n");
                            }
                        }
                    }else{
                        //tracking failed
                        if (track && init.valid){
                            ii = start -1;
                            if (debug) printf("Inner segment not a circle\n");
                        }
                    }
                }else{
                    if (track && init.valid){
                        ii = start -1;
                        if (debug) printf("Inner segment not white %i %i %i\n",threshold,ptr[0],outer.size);
                    }
                }
            }else{
                //tracking failed
                if (track && init.valid){
                    ii = start -1;
                    if (debug) printf("Outer segment %.0f %.0f %i not a circle\n",outer.x,outer.y,outer.size);
                }
            }
        }
        ii++;
        if (ii >= len) ii = 0;
        cont = (ii != start);
    }
    if (debug) printf("II: %i %i\n",ii,start);
    if (debug)fprintf(stdout,"Inner %.2f %.2f Area: %i Vx: %i Vy: %i Mean: %i Thr: %i Eigen: %03f %03f %03f %03f Axes: %03f \n",inner.x,inner.y,inner.size,inner.maxx-inner.minx,inner.maxy-inner.miny,inner.mean,threshold,inner.m0,inner.m1,inner.v0,inner.v1,inner.v0*outer.v0+inner.v1*outer.v1);
    if (debug && identify) fprintf(stdout,"Outer %.2f %.2f Area: %i Vx: %i Vy: %i Mean: %i Thr: %i Eigen: %03f %03f %03f %03f Ratios: %.3f %.3f %i\n",outer.x,outer.y,outer.size,outer.maxx-outer.minx,outer.maxy-outer.miny,outer.mean,threshold,outer.m0,outer.m1,outer.v0,outer.v1,outer.r0*150,outer.r1*150,outer.ID);
    else if (debug)fprintf(stdout,"Outer %.2f %.2f Area: %i Vx: %i Vy: %i Mean: %i Thr: %i Eigen: %03f %03f %03f %03f Ratios: %.3f %.3f\n",outer.x,outer.y,outer.size,outer.maxx-outer.minx,outer.maxy-outer.miny,outer.mean,threshold,outer.m0,outer.m1,outer.v0,outer.v1,outer.r0*150,outer.r1*150);

    if (outer.valid){
        if (numSegments == 2 ){
            lastTrackOK = true;
        } else {
            lastTrackOK = false;
        }
    }

    //threshold management
    if (outer.valid){
        lastThreshold = threshold;
        numFailed = 0;  
    }else if (numFailed < maxFailed){
        if (numFailed++%2 == 0) changeThreshold(); else threshold = lastThreshold;
    }else{
        numFailed++;
        if (changeThreshold()==false) numFailed = 0;
    }

    //Drawing results 
    if (outer.valid){
        for (int p =  queueOldStart;p< queueEnd;p++)
        {
            pos = queue[p]; 
            image->data[step*pos+0] =  image->data[step*pos+1] =  image->data[step*pos+2] = 0;
        }
    }
    if (draw){
        if (init.valid || track || lastTrackOK){
            for (int p = 0;p<queueOldStart;p++){
                pos = queue[p];
                image->data[step*pos+0] = 255;
                image->data[step*pos+1] = 255;
                image->data[step*pos+2] = 200;
            }
        }
    }
    bufferCleanup(outer);

    return outer;
}

SSegment CCircleDetect::getInnerSegment()
{
    return inner;
}

float CCircleDetect::normalizeAngle(float a) {
    while (a > +M_PI) a+=-2*M_PI;
    while (a < -M_PI) a+=+2*M_PI;
    return a;
}

