#include "CCircleDetect.h"

/*
 * File name: CCircleDetect.h
 * Date:      2014
 * Author:   Tom Krajnik, Matias Nitsche
 * Description: implements computationally efficient and precist circular pattern detection. Algorithm is described in Chapter 3 of the article [1]. 
 * Licence: if you use this class for your research, please cite [1]. 
 * References: [1] Krajnik, Nitsche et al.: A practical multirobot localization system. Journal of Intelligent and Robotic Systems, 2014.
 */

#define min(a,b) ((a) < (b) ? (a) : (b))
#define max(a,b) ((a) > (b) ? (a) : (b))

int* CCircleDetect::buffer = NULL;
int* CCircleDetect::queue = NULL;
int* CCircleDetect::mask = NULL;
int CCircleDetect::maskNum = 0;

//Variable initialization
CCircleDetect::CCircleDetect(int wi,int he,int idi)
{
	localSearch = false;				//by default, search for the pattern eveywhere, true is used when position of the pattern is indicated by a click
	ID = idi;					//pattern ID - not used in this case	
	numberIDs =0;					//pattern ID - not used in this case	
	enableCorrections = true;			//enables brightness compensation, see 3.5 of [1]
	lastTrackOK = false;				//was the pattern detected in the previous step ? used to initiate the search position
	debug = 0;					//debug level 
	draw = false; 					//draw the detected segments in bright colors to indicate segmentation results
	drawAll = true;					//draw all segmentation results - used for debugging
	maxFailed = 0;					//used to decide when to start changing the threshold 
	minSize = 15;					//minimal pattern size in pixels
	thresholdStep = 256;				//related to thresholding in case of unsuccessful detections, see 3.2 of [1]
	maxThreshold = 3*256;				//related to thresholding in case of unsuccessful detections, see 3.2 of [1]
	centerDistanceToleranceRatio = 0.1;		//max allowd distance of the inner and outer circle centers (relative to pattern dimensions)
	centerDistanceToleranceAbs = 15;		//max allowed distance of the inner and outer circle centers (in pixels)
	circularTolerance = 1.5;			//maximal tolerance of bounding box dimensions vs expected pixel area - see equation 2 of the paper [1] 
	ratioTolerance = 1.4;				//maximal tolerance of black to white pixel ratios - see Algorithm 2 of [1]
	threshold = maxThreshold/2;			//default tresholt

	numFailed = maxFailed;				//used to decide when to start changing the threshold 
	track = true;					//initiate the search from the last position ?
	circularityTolerance = 0.02;			//final circularity test, see Equation 5 of [1]

	/*initialization of supporting structures according to the image size provided*/ 
	width = wi;
	height = he;
	len = width*height;
	siz = len*3;
	ownBuffer = false;
	if (buffer == NULL){
		ownBuffer = true;
		buffer = (int*)malloc(len*sizeof(int));
		queue = (int*)malloc(len*sizeof(int));
		mask = (int*)malloc(len*sizeof(int));
		SSegment dummy;
		bufferCleanup(dummy);
	}

	/*inner vs. outer circle diameter, used to calculate expected pixel ratios, see Alg 2 and Eq. 2 of [1]   */
	diameterRatio = 50.0/122.0; 			
	float areaRatioInner_Outer = diameterRatio*diameterRatio;
	outerAreaRatio = M_PI*(1.0-areaRatioInner_Outer)/4;
	innerAreaRatio = M_PI/4.0;
	areasRatio = (1.0-areaRatioInner_Outer)/areaRatioInner_Outer;

	//timers for benchmarking
	tima = timb = timc =timd = sizer = sizerAll = 0;
//	loadCircleID("../etc/ID.txt");
}

CCircleDetect::~CCircleDetect()
{
//	if (debug > 5) printf("Timi %i %i %i %i\n",tima,timb,sizer,sizerAll);
	if (ownBuffer){
		free(buffer);
		free(queue);
	}
}

bool CCircleDetect::changeThreshold()
{
	int div = 1;
	int dum = numFailed;
	while (dum > 1){
		dum = dum/2;
		div*=2;
	}
	int step = 256/div;
	threshold = 3*(step*(numFailed-div)+step/2);
	if (debug > 5) fprintf(stdout,"Threshold: %i %i %i\n",div,numFailed,threshold/3);
	return step > 8;
}

bool CCircleDetect::examineSegment(CRawImage *image,SSegment *segmen,int ii,float areaRatio)
{
	timer.reset();
	timer.start();
	int vx,vy;
	queueOldStart = queueStart;
	int position = 0;
	int pos;	
	bool result = false;
	int type = buffer[ii];
	int maxx,maxy,minx,miny;

	buffer[ii] = ++numSegments;
	segmen->x = ii%width; 
	segmen->y = ii/width;
	minx = maxx = segmen->x;
	miny = maxy = segmen->y;
	segmen->valid = false;
	segmen->round = false;
	//push segment coords to the queue
	queue[queueEnd++] = ii;
	//and until queue is empty
	while (queueEnd > queueStart){
		//pull the coord from the queue
		position = queue[queueStart++];
		//search neighbours
		pos = position+1;
		if (buffer[pos] == 0){
			 ptr = &image->data[pos*3];
			 buffer[pos]=((ptr[0]+ptr[1]+ptr[2]) > threshold)-2;
		}
		if (buffer[pos] == type){
			queue[queueEnd++] = pos;
			maxx = max(maxx,pos%width);
			buffer[pos] = numSegments;
		}
		pos = position-1;
		if (buffer[pos] == 0){
			 ptr = &image->data[pos*3];
			 buffer[pos]=((ptr[0]+ptr[1]+ptr[2]) > threshold)-2;
		}
		if (buffer[pos] == type){
			queue[queueEnd++] = pos;
			minx = min(minx,pos%width);
			buffer[pos] = numSegments;
		}
		pos = position-width;
		if (buffer[pos] == 0){
			 ptr = &image->data[pos*3];
			 buffer[pos]=((ptr[0]+ptr[1]+ptr[2]) > threshold)-2;
		}
		if (buffer[pos] == type){
			queue[queueEnd++] = pos;
			miny = min(miny,pos/width);
			buffer[pos] = numSegments;
		}
		pos = position+width;
		if (buffer[pos] == 0){
			 ptr = &image->data[pos*3];
			 buffer[pos]=((ptr[0]+ptr[1]+ptr[2]) > threshold)-2;
		}
		if (buffer[pos] == type){
			queue[queueEnd++] = pos;
			maxy = max(maxy,pos/width);
			buffer[pos] = numSegments;
		}
	}

	//once the queue is empty, i.e. segment is complete, we compute its size 
	segmen->size = queueEnd-queueOldStart;
	if (segmen->size > minSize){
		//and if its large enough, we compute its other properties 
		segmen->maxx = maxx;
		segmen->maxy = maxy;
		segmen->minx = minx;
		segmen->miny = miny;
		segmen->type = -type;
		vx = (segmen->maxx-segmen->minx+1);
		vy = (segmen->maxy-segmen->miny+1);
		segmen->x = (segmen->maxx+segmen->minx)/2;
		segmen->y = (segmen->maxy+segmen->miny)/2;
		segmen->roundness = vx*vy*areaRatio/segmen->size;
		//we check if the segment is likely to be a ring 
		if (segmen->roundness - circularTolerance < 1.0 && segmen->roundness + circularTolerance > 1.0)
		{
			//if its round, we compute yet another properties 
			segmen->round = true;
			segmen->mean = 0;
			for (int p = queueOldStart;p<queueEnd;p++){
				pos = queue[p];
				segmen->mean += image->data[pos*3]+image->data[pos*3+1]+image->data[pos*3+2];
			}
			segmen->mean = segmen->mean/segmen->size;
			result = true;	
		}
	}
	timb +=timer.getTime();
	return result;
}

void CCircleDetect::bufferCleanup(SSegment init)
{
	int pos = (height-1)*width;
	if (init.valid ==false || track == false || lastTrackOK==false){
		memset(buffer,0,sizeof(int)*len);
		for (int i = 0;i<width;i++){
			buffer[i] = -1000;	
			buffer[pos+i] = -1000;
		}
		for (int i = 0;i<height;i++){
			buffer[width*i] = -1000;	
			buffer[width*i+width-1] = -1000;
		}
	}else{
		int pos,ix,ax,iy,ay;
		ix = max(init.minx - 2,1);
		ax = min(init.maxx + 2,width-2);
		iy = max(init.miny - 2,1);
		ay = min(init.maxy + 2,height-2);
		for (int y=iy;y<ay;y++){
			pos = y*width; 
			for (int x=ix;x<ax;x++) buffer[pos+x] = 0;
		}
	}
}

//not for use in this version
int CCircleDetect::loadCircleID(const char* id)
{
	FILE* idFile = fopen(id,"r");
	int dummy = 0;
	numberIDs=0;
	if (idFile == NULL)
	{
		fprintf(stderr,"ID file not found\n");
		return -2;
	}
	while(feof(idFile)==0){
		fscanf(idFile,"%i %f %f\n",&dummy,&idx[numberIDs],&idy[numberIDs]);
		if (dummy != numberIDs){
			fprintf(stderr,"ID file corrupted\n");
			fclose(idFile);
			return -1;
		}
		numberIDs++;
	}
	fclose(idFile);
	return 0;
}

void CCircleDetect::identifySegment(SSegment* segment)
{
	segment->ID = 0;
	return;
	//not for use in this version
	float maxDistance = 1000;
	int index = -1;
	float dx,dy;
	for (int i=0;i<numberIDs;i++)
	{
		dx = segment->r0-idx[i];
		dy = segment->r1-idy[i];
		if (dx*dx+dy*dy < maxDistance)
		{
			maxDistance = dx*dx+dy*dy;
			index = i;
		}
	}
	segment->ID = index;
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

SSegment CCircleDetect::findSegment(CRawImage* image, SSegment init)
{
	numSegments = 0;

	timer.reset();
	timer.start();
	tima += timer.getTime();
	int pos = 0;
	int ii = 0;
	int start = 0;
	bool cont = true;

	//bufferCleanup(init);
	if (init.valid && track){
		ii = ((int)init.y)*image->width+init.x;
		start = ii;
	}
	while (cont) 
	{
		if (buffer[ii] == 0){
			ptr = &image->data[ii*3];
			//buffer[ii]=((ptr[0]+ptr[1]+ptr[2]) > threshold)-2;
			if ((ptr[0]+ptr[1]+ptr[2]) < threshold) buffer[ii] = -2;
		}
		if (buffer[ii] == -2){
			//new segment found
			queueEnd = 0;
			queueStart = 0;
			//if the segment looks like a ring, we check its inside area
			if (examineSegment(image,&outer,ii,outerAreaRatio)){
				pos = outer.y*image->width+outer.x;
				if (buffer[pos] == 0){
					ptr = &image->data[pos*3];
					buffer[pos]=((ptr[0]+ptr[1]+ptr[2]) >= threshold)-2;
				}	
				if (buffer[pos] == -1){
					if (examineSegment(image,&inner,pos,innerAreaRatio)){
						//the inside area is a circle. now what is the area ratio of the black and white ? also, are the circles concentric ?

						if (debug > 5) printf("Area ratio should be %.3f, but is %.3f, that is %.0f%% off. ",areasRatio,(float)outer.size/inner.size,(1-outer.size/areasRatio/inner.size)*100);
						if ((float)outer.size/areasRatio/(float)inner.size - ratioTolerance < 1.0 && (float)outer.size/areasRatio/(float)inner.size + ratioTolerance > 1.0){ 
							if (debug > 5) fprintf(stdout,"Segment BW ratio OK.\n");
							if (debug > 5) fprintf(stdout,"Concentricity %.0f %.0f %.0f %.0f.",inner.x,inner.y, outer.x,outer.y);
							if((abs(inner.x-outer.x) <= centerDistanceToleranceAbs+centerDistanceToleranceRatio*((float)(outer.maxx-outer.minx))) &&
									(abs(inner.y-outer.y) <= centerDistanceToleranceAbs+centerDistanceToleranceRatio*((float)(outer.maxy-outer.miny))))

							{
								if (debug > 5) fprintf(stdout,"Concentricity OK.\n");
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
								sizerAll+=len; 								    //for debugging
								float circularity = M_PI*4*(outer.m0)*(outer.m1)/queueEnd;
								if (debug > 5) fprintf(stdout,"Segment circularity: %i %03f %03f \n",queueEnd,M_PI*4*(outer.m0)*(outer.m1)/queueEnd,M_PI*4*(outer.m0)*(outer.m1));
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
									if (debug > 5) printf("Angle: %.3f %.3f \n",outer.angle,orient);
									if (fabs(normalizeAngle(outer.angle-orient)) > M_PI/2) outer.angle = normalizeAngle(outer.angle+M_PI);
									
									//fiducial identification - experimental only
									identifySegment(&outer);
									//if (lastTrackOK == false) identifySegment(&outer);
									//outer.ID =ID;
									outer.valid = inner.valid = true;
									threshold = (outer.mean+inner.mean)/2;
									if (track) ii = start -1;
								}else{
									if (track && init.valid){
										ii = start -1;
										if (debug > 0) fprintf(stdout,"Segment failed circularity test.\n");
									}
								}
							}else{
								if (track && init.valid){
									ii = start -1;
									if (debug > 0) fprintf(stdout,"Segment failed concentricity test.\n");
								}
							}
						}else{
							//tracking failed
							if (track && init.valid){
								ii = start -1;
								if (debug >0) fprintf(stdout,"Segment failed BW test.\n");
							}
						}
					}else{
						//tracking failed
						if (track && init.valid){
							 ii = start -1;
							 if (debug >0) printf("Inner segment not a circle\n");
						}
					}
				}else{
					if (track && init.valid){
						ii = start -1;
						if (debug>0) printf("Inner segment not white %i %i %i\n",threshold,ptr[0]+ptr[1]+ptr[2],outer.size);
					}
				}
			}else{
				//tracking failed
				if (track && init.valid){
					ii = start -1;
					if (debug>0) printf("Outer segment %.0f %.0f %i not a circle\n",outer.x,outer.y,outer.size);
				}
			}
		}
		ii++;
		if (ii >= len) ii = 0;
		cont = (ii != start);
	}
	if (debug > 5) printf("II: %i %i\n",ii,start);
	if (debug > 1)fprintf(stdout,"Inner %.2f %.2f Area: %i Vx: %i Vy: %i Mean: %i Thr: %i Eigen: %03f %03f %03f %03f Axes: %03f \n",inner.x,inner.y,inner.size,inner.maxx-inner.minx,inner.maxy-inner.miny,inner.mean,threshold,inner.m0,inner.m1,inner.v0,inner.v1,inner.v0*outer.v0+inner.v1*outer.v1);
	if (debug > 1)fprintf(stdout,"Outer %.2f %.2f Area: %i Vx: %i Vy: %i Mean: %i Thr: %i Eigen: %03f %03f %03f %03f Ratios: %.3f %.3f %i\n",outer.x,outer.y,outer.size,outer.maxx-outer.minx,outer.maxy-outer.miny,outer.mean,threshold,outer.m0,outer.m1,outer.v0,outer.v1,outer.r0*150,outer.r1*150,outer.ID);
	if (outer.valid){
		if (numSegments == 2 ){
			lastTrackOK = true;
			localSearch = false;
		} else {
			lastTrackOK = false;
			if (localSearch) outer.valid = false;
		}
	}
	//threshold management
	if (outer.valid){
		lastThreshold = threshold;
		drawAll = false;
		numFailed = 0;	
	}else if (numFailed < maxFailed){
		if (numFailed++%2 == 0) changeThreshold(); else threshold = lastThreshold;
		if (debug > 5) drawAll = true;
	}else{
		numFailed++;
		if (changeThreshold()==false) numFailed = 0;
		if (debug > 5) drawAll = true;
	}

	//Drawing results 
	if (outer.valid){
		for (int p =  queueOldStart;p< queueEnd;p++)
		{
			pos = queue[p];	
			image->data[3*pos+0] = 	image->data[3*pos+1] = 	image->data[3*pos+2] = 0;
		}
	}
	if (draw){
		if (init.valid || track || lastTrackOK){
			for (int p = queueOldStart;p<queueEnd;p++){
				pos = queue[p];	
				image->data[3*pos+0] = 0;
				image->data[3*pos+1] = 255;
				image->data[3*pos+2] = 255;
			}
			for (int p = 0;p<queueOldStart;p++){
				pos = queue[p];	
				image->data[3*pos+0] = 255;
				image->data[3*pos+1] = 0;
				image->data[3*pos+2] = 255;
			}
		}
	}
	bufferCleanup(outer);
	return outer;
}
