#include "CTransformation.h"
#include <stdio.h>
#include "sysmat.h" 

int sortByDistance(const void* m1,const void* m2)
{
        if (((STrackedObject*)m1)->d > ((STrackedObject*)m2)->d) return -1;
        if (((STrackedObject*)m1)->d < ((STrackedObject*)m2)->d) return 1;
        return 0;
}

CTransformation::CTransformation(int widthi,int heighti,float diam,bool fullUnbarreli)
{
	transformType = TRANSFORM_NONE;
	fullUnbarrel = fullUnbarreli;
	width = widthi;
	height = heighti;
	char dummy[1000];
	FILE* file = fopen("../etc/Calib_Results.m","r");
	trackedObjectDiameter = diam;
	while (feof(file)== false){
		int err = fscanf(file,"%s",dummy);
		if (strcmp(dummy,"fc") == 0){
			for (int j = 0;j<10;j++){
				if (fscanf(file,"%s\n",dummy)!=1) fprintf(stderr,"Transformation: error reading camera calibration file, %i.\n",err);
				if (j==2) fc[0] = atof(dummy);
				if (j==4) fc[1] = atof(dummy);
			}
		}
		if (strcmp(dummy,"cc") == 0){
			for (int j = 0;j<10;j++){
				if(fscanf(file,"%s\n",dummy)!=1) fprintf(stderr,"Transformation: error reading camera calibration file.\n");
				if (j==2) cc[0] = atof(dummy);
				if (j==4) cc[1] = atof(dummy);
			}
		}
		if (strcmp(dummy,"kc") == 0){
			for (int j = 0;j<12;j++){
				if(fscanf(file,"%s\n",dummy)!=1) fprintf(stderr,"Transformation: error reading camera calibration file.\n");
				if (j>1 && j%2 == 0) kc[j/2] = atof(dummy);
			}
		}
		if (strcmp(dummy,"kc_error") == 0){
			for (int j = 0;j<12;j++){
				if(fscanf(file,"%s\n",dummy)!=1) fprintf(stderr,"Transformation: error reading camera calibration file.\n");
				if (j>1 && j%2 == 0) kcerr[j/2] = atof(dummy);
			}
		}
		if (strcmp(dummy,"fc_error") == 0){
			for (int j = 0;j<10;j++){
				if (fscanf(file,"%s\n",dummy)!=1) fprintf(stderr,"Transformation: error reading camera calibration file, %i.\n",err);
				if (j==2) fcerr[0] = atof(dummy);
				if (j==4) fcerr[1] = atof(dummy);
			}
		}
	}
	kc[0] = 1.0;
	for (int i = 0;i<6;i++) printf("%05f,",kc[i]);
	printf("\n");
	for (int i = 0;i<2;i++) printf("%05f,",fc[i]);
	printf("\n");
	for (int i = 0;i<2;i++) printf("%05f,",cc[i]);
	printf("\n");
	unbarrelInitialized = false;

	if (fullUnbarrel){
		unbarrelInitialized = true;
		float ix,iy; 
		float gx,gy,ux,uy;
		xArray = (float*)malloc(width*height*sizeof(float));
		yArray = (float*)malloc(width*height*sizeof(float));
		gArrayX = (float*)malloc(width*height*sizeof(float));
		gArrayY = (float*)malloc(width*height*sizeof(float));
		pArray = (int*)malloc(width*height*sizeof(int)*4);

		for (int x = 0;x<width;x++){
			for (int y = 0;y<height;y++){
				xArray[y*width+x] = barrelX(x,y);
				yArray[y*width+x] = barrelY(x,y);
				if (xArray[y*width+x] < 0 || xArray[y*width+x] > (width-1) || yArray[y*width+x] < 0 || yArray[y*width+x] > (height-1)){
					xArray[y*width+x] = 0; 
					yArray[y*width+x] = 0; 
				}	
				ux = trunc(xArray[y*width+x]);
				uy = trunc(yArray[y*width+x]);
				gx = xArray[y*width+x]-ux;
				gy = yArray[y*width+x]-uy;
				ix = (int)ux;
				iy = (int)uy;
				pArray[y*width+x] = width*iy+ix;
				gArrayX[y*width+x] = gx; 
				gArrayY[y*width+x] = gy; 
			}
		}
	}
	loadCalibration("../etc/default.cal");
}

CTransformation::~CTransformation()
{
	if (unbarrelInitialized){
		free(xArray);
		free(yArray);
		free(gArrayX);
		free(gArrayY);
		free(pArray);
	}
}

float CTransformation::barrelX(float x,float y)
{
	x = (x-cc[0])/fc[0];
	y = (y-cc[1])/fc[1];
	float cx,dx;
	float r = x*x+y*y;
	dx = 2*kc[3]*x*y + kc[4]*(r + 2*x*x);
	cx = (1+kc[1]*r+kc[2]*r*r+kc[5]*r*r*r)*x+dx;
	cx = cx*fc[0]+cc[0];
	return cx;
}

float CTransformation::barrelY(float x,float y)
{
	x = (x-cc[0])/fc[0];
	y = (y-cc[1])/fc[1];
	float cy,dy;
	float r = x*x+y*y;
	dy = 2*kc[4]*x*y + kc[3]*(r + 2*y*y);
	cy = (1+kc[1]*r+kc[2]*r*r+kc[5]*r*r*r)*y+dy;
	cy = cy*fc[1]+cc[1];
	return cy;
}

float CTransformation::unbarrelX(float x,float y)
{
	if (fullUnbarrel)return x;
	float ix,iy,dx,dy,r,rad;
	ix = x = (x-cc[0])/fc[0];
	iy = y = (y-cc[1])/fc[1];
	for (int i= 0;i<5;i++){
		r = x*x+y*y;
		dx = 2*kc[3]*x*y + kc[4]*(r + 2*x*x);
		dy = 2*kc[4]*x*y + kc[3]*(r + 2*y*y);
		rad = 1+kc[1]*r+kc[2]*r*r+kc[5]*r*r*r;
		x = (ix-dx)/rad;
		y = (iy-dy)/rad;
	}
	return (x*fc[0]+cc[0]);
}

float CTransformation::unbarrelY(float x,float y)
{
	if (fullUnbarrel) return y;
	float ix,iy,dx,dy,r,rad;
	ix = x = (x-cc[0])/fc[0];
	iy = y = (y-cc[1])/fc[1];
	for (int i= 0;i<5;i++){
		r = x*x+y*y;
		dx = 2*kc[3]*x*y + kc[4]*(r + 2*x*x);
		dy = 2*kc[4]*x*y + kc[3]*(r + 2*y*y);
		rad = 1+kc[1]*r+kc[2]*r*r+kc[5]*r*r*r;
		x = (ix-dx)/rad;
		y = (iy-dy)/rad;
	}
	return (y*fc[1]+cc[1]);
}

void CTransformation::transformXYerr(float *ax,float *ay)
{
	float x,y,dx,dy,r,rad;
	//*ax = x = (*ax-cc[0])/fc[0];
	//*ay = y = (*ay-cc[1])/fc[1];
	x = *ax;
	y = *ay;
	if (fullUnbarrel)return;
	r = x*x+y*y;
	dx = 2*kcerr[3]*x*y + kcerr[4]*(r + 2*x*x);
	dy = 2*kcerr[4]*x*y + kcerr[3]*(r + 2*y*y);
	rad = kcerr[1]*r+kcerr[2]*r*r+kcerr[5]*r*r*r;
	*ax=rad*x+dx;
	*ay=rad*y+dy;
}

void CTransformation::transformXY(float *ax,float *ay)
{
	float x,y,ix,iy,dx,dy,r,rad;
	*ax = ix = x = (*ax-cc[0])/fc[0];
	*ay = iy = y = (*ay-cc[1])/fc[1];
	if (fullUnbarrel)return;
	for (int i= 0;i<5;i++){
		r = x*x+y*y;
		dx = 2*kc[3]*x*y + kc[4]*(r + 2*x*x);
		dy = 2*kc[4]*x*y + kc[3]*(r + 2*y*y);
		rad = 1+kc[1]*r+kc[2]*r*r+kc[5]*r*r*r;
		x = (ix-dx)/rad;
		y = (iy-dy)/rad;
	}
	*ax=x;
	*ay=y;
}

float CTransformation::transformX(float xc,float yc)
{
	return (unbarrelX(xc,yc)-cc[0])/fc[0];
}

float CTransformation::transformY(float xc,float yc)
{
	return (unbarrelY(xc,yc)-cc[1])/fc[1];
}

void CTransformation::unbarrel(unsigned char *dst,unsigned char *src)
{
	src[0] = src[1] = src[2] = 255;
	if (fullUnbarrel){
		float gx,gy;
		for (int p = 0;p<width*(height-1);p++){
			gx = gArrayX[p];
			gy = gArrayY[p];
			dst[3*p] = src[3*pArray[p]]*(1-gx)*(1-gy)+src[3*pArray[p]+3]*gx*(1-gy)+src[3*pArray[p]+width*3]*(1-gx)*gy+src[3*pArray[p]+(width+1)*3]*gx*gy; 
			dst[3*p+1] = src[3*pArray[p]+1]*(1-gx)*(1-gy)+src[3*pArray[p]+3+1]*gx*(1-gy)+src[3*pArray[p]+width*3+1]*(1-gx)*gy+src[3*pArray[p]+(width+1)*3+1]*gx*gy; 
			dst[3*p+2] = src[3*pArray[p]+2]*(1-gx)*(1-gy)+src[3*pArray[p]+3+2]*gx*(1-gy)+src[3*pArray[p]+width*3+2]*(1-gx)*gy+src[3*pArray[p]+(width+1)*3+2]*gx*gy; 
		}
	}else{
		fprintf(stdout,"Image unbarrel was not enabled\n");
	}
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
	r.error = establishError(r);
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
	r.error = establishError(r);
	return r; 	
}

float CTransformation::distance(STrackedObject o1,STrackedObject o2)
{
	return sqrt((o1.x-o2.x)*(o1.x-o2.x)+(o1.y-o2.y)*(o1.y-o2.y)+(o1.z-o2.z)*(o1.z-o2.z));
}

STrackedObject CTransformation::transformInv(STrackedObject o[])
{
	STrackedObject trk[4];
	trk[3].x = trk[3].y = trk[3].z = 0;
	for (int i=0;i<3;i++) trk[i] = o[i];
	for (int i=0;i<3;i++) trk[i].d = distance(trk[(i+1)%3],trk[(i+2)%3]);
	qsort(trk,3,sizeof(STrackedObject),sortByDistance);
	float an = atan2(trk[2].x-trk[0].x,trk[2].y-trk[0].y);
	fprintf(stdout,"Dock position: %.3f %.3f %.3f %.3f\n",trk[0].x,trk[0].y,trk[0].z,180*an/M_PI);
	/*for (int i=0;i<3;i++) fprintf(stdout,"%.3f ",trk[i].d);
	fprintf(stdout,"%.3f \n",sqrt(trk[1].d*trk[1].d+trk[2].d*trk[2].d));
	D3transform[0] =  calibrate3D(trk[0],trk[2],trk[1],trk[2].d,trk[1].d);
	transformType = TRANSFORM_INV;
	trk[3] = transform3D(trk[3],1);
	fprintf(stdout,"Robot: %.3f %.3f %.3f\n",trk[3].x,trk[3].y,trk[3].z);*/
	return o[0];
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

 	
	float x,y,z;
	final.esterror = 0;
	for (int k = 0;k<num;k++){
		x = final.x-result[k].x;
		y = final.y-result[k].y;
		z = final.z-result[k].z,
		final.esterror+=sqrt(x*x+y*y+z*z);
	}
	float xerr0 = -o.z/o.x;
	float yerr0 = -o.y/o.x;
	transformXYerr(&xerr0,&yerr0);
	final.esterror= sqrt(xerr0*xerr0+yerr0*yerr0)*30+fcerr[0]/fc[0]*30;
	
	//final.esterror = final.esterror/num;
	final.error = establishError(final);
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

//this function if meant for debugging
float CTransformation::establishError(STrackedObject o)
{
	STrackedObject result;
	float scale = 0.625;
	result.x = (o.x/scale-rintf(o.x/scale))*scale;
	result.y = (o.y/scale-rintf(o.y/scale))*scale;
	result.z = (o.z/scale-rintf(o.z/scale))*scale;
	return sqrt(result.x*result.x+result.y*result.y+result.z*result.z);
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

STrackedObject CTransformation::crossPrd(STrackedObject o0,STrackedObject o1,STrackedObject o2,float gridDimX,float gridDimY)
{
	STrackedObject v[3];
	v[0].x = o1.x-o0.x;
	v[0].y = o1.y-o0.y;
	v[0].z = o1.z-o0.z;
	v[0] = normalize(v[0]);

	v[1].x = o2.x-o0.x;
	v[1].y = o2.y-o0.y;
	v[1].z = o2.z-o0.z;
	v[1] = normalize(v[1]);

	v[2].x = +v[0].y*v[1].z-v[1].y*v[0].z+o0.x;
	v[2].y = +v[0].z*v[1].x-v[1].z*v[0].x+o0.y;
	v[2].z = +v[0].x*v[1].y-v[1].x*v[0].y+o0.z;
	return v[2];
}

int CTransformation::calibrate4D(STrackedObject o[],float gridDimX,float gridDimY)
{
	//OBSOLETE
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
STrackedObject CTransformation::eigen(double data[])
{
	STrackedObject result;
	result.error = 0;
	double d[3];
	double V[3][3];
	double dat[3][3];
	for (int i = 0;i<9;i++)dat[i/3][i%3] = data[i];
	eigen_decomposition(dat,V,d);

	//eigenvalues
	float L1 = d[1]; 
	float L2 = d[2];
	float L3 = d[0];
	//eigenvectors
	int V2=2;
	int V3=0;

	//detected pattern position
	float z = trackedObjectDiameter/sqrt(-L2*L3)/2.0;
	float c0 =  sqrt((L2-L1)/(L2-L3));
	float c0x = c0*V[2][V2];
	float c0y = c0*V[1][V2];
	float c0z = c0*V[2][V2];
	float c1 =  sqrt((L1-L3)/(L2-L3));
	float c1x = c1*V[0][V3];
	float c1y = c1*V[1][V3];
	float c1z = c1*V[2][V3];

	float z0 = -L3*c0x+L2*c1x;
	float z1 = -L3*c0y+L2*c1y;
	float z2 = -L3*c0z+L2*c1z;
	float s1,s2;
	s1=s2=1;
	float n0 = +s1*c0x+s2*c1x;
	float n1 = +s1*c0y+s2*c1y;
	float n2 = +s1*c0z+s2*c1z;

	//n0 = -L3*c0x-L2*c1x;
	//n1 = -L3*c0y-L2*c1y;
	//n2 = -L3*c0z-L2*c1z;
	
	//rotate the vector accordingly
	if (z2*z < 0){
		 z2 = -z2;
		 z1 = -z1;
		 z0 = -z0;
	//	 n0 = -n0;
	//	 n1 = -n1;
	//	 n2 = -n2;
	}
	result.x = z2*z;	
	result.y = -z0*z;	
	result.z = -z1*z;
	result.pitch = n0;//cos(segment.m1/segment.m0)/M_PI*180.0;
	result.roll = n1;//atan2(segment.v1,segment.v0)/M_PI*180.0;
	result.yaw = n2;//segment.v1/segment.v0;
	//result.roll = n2*z;	
	//result.pitch = -n0*z;	
	//result.yaw = -n1*z;


	return result;
}

STrackedObject CTransformation::transform(SSegment segment,bool unbarreli)
{
	float x,y,x1,x2,y1,y2,major,minor,v0,v1;
	STrackedObject result;
	fullUnbarrel = unbarreli;
	//Transform to the Canonical camera coordinates
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

	//Construct the ellipse characteristic equation
	float a,b,c,d,e,f;
	a = v0*v0/(major*major)+v1*v1/(minor*minor);
	b = v0*v1*(1.0/(major*major)-1.0/(minor*minor));
	c = v0*v0/(minor*minor)+v1*v1/(major*major);
	d = (-x*a-b*y);
	e = (-y*c-b*x);
	f = (a*x*x+c*y*y+2*b*x*y-1.0);
	
	//transformation to global coordinates
	double data[] ={a,b,d,b,c,e,d,e,f};
	//printf("%lf %lf %lf\n%lf %lf %lf\n%lf %lf %lf\n",data[0],data[1],data[2],data[3],data[4],data[5],data[6],data[7],data[8]);
	//homographic transform
	if (transformType == TRANSFORM_2D){
		//for debug only
		result = eigen(data);
		float d = sqrt(result.x*result.x+result.y*result.y+result.z*result.z);

		result.x = x;
		result.y = y;
		result = transform2D(result);
		result.d = d;

		float xerr = x;
		float yerr = y;
		transformXYerr(&xerr,&yerr);
		result.esterror = fabs(sqrt(xerr*xerr+yerr*yerr))*30; 
		result.yaw = atan2(segment.v0,segment.v1);
		result.yaw = segment.angle;
		result.ID = segment.ID;
	}
	//transform
	if (transformType == TRANSFORM_NONE){
		result = eigen(data);
	}
	if (transformType == TRANSFORM_3D){
		result = eigen(data);
		float d = sqrt(result.x*result.x+result.y*result.y+result.z*result.z);
		result = transform3D(result);
//		result.esterror += 20.0/(segment.m0*4)+0.20/(segment.m1*4);
		result.esterror += 15.0/(segment.m1*4);
		result.d = d;
	}
	if (transformType == TRANSFORM_4D){
		result = eigen(data);
		float d = sqrt(result.x*result.x+result.y*result.y+result.z*result.z);
		result = transform4D(result);
		result.d = d;
	}
	if (transformType == TRANSFORM_INV){
		result = eigen(data);
		float d = sqrt(result.x*result.x+result.y*result.y+result.z*result.z);
		result.d = d;
	}
	/*result.pitch = acos(fmin(minor/major,1.0))/M_PI*180.0; //TODO
	result.roll = segment.horizontal; //TODO*/
	return result;
}

