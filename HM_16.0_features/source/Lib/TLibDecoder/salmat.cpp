#include "salmat.h"
#include<opencv2/core/core.hpp>  //re
#include<opencv2/highgui/highgui.hpp>  
#include "cv.hpp"
#include "TDecCu.h" 
#include <Math.h>
#include<fstream>  
using namespace cv;  
using namespace std; 
salmat::salmat(float Delta1,float Delta2,int Size1,int Size2,int row,int col)
{
	DeltaSpatial=Delta1;
	DeltaTemporal=Delta2;
	SpatialSize=Size1;
	MinSize=Size2;
	rows=row;
	cols=col;

	PreFrameSmooth=cvRound(FRAMERATE*0.3);
	PreFrameTemp=cvRound(FRAMERATE*0.7);
	SmoothFrame=new Mat[PreFrameSmooth];
	TemporalFrame=new Mat[PreFrameTemp];

}
salmat::salmat(void)
{
}	


salmat::~salmat(void)
{
}

void salmat::Initial()
{

}

void salmat::ForwardFilter(UInt FrameIndex)
{
	Mat tempmat=Mat::zeros(ThisFrameValue.size(), ThisFrameValue.type());

	ThisFrameValue.copyTo(SmoothFrame[FrameIndex%PreFrameSmooth]);
	if (FrameIndex<PreFrameSmooth)
	{	
		for(int i=0;i<=FrameIndex;i++)
			tempmat=tempmat+SmoothFrame[i]/(FrameIndex+1);
	}
	else
	{
		for(int i=0;i<PreFrameSmooth;i++)
			tempmat=tempmat+SmoothFrame[i]/PreFrameSmooth;
	}
	tempmat.copyTo(ThisFrameValue);
}

void salmat::MatrixDuplication(Mat src, Mat& dst, int Multi) //dst.rows could not be src.rows*Multi
{	
	
	CV_Assert(src.depth() == CV_32FC1&&dst.depth() == CV_32FC1);
	CV_Assert(dst.rows>(src.rows-1)*Multi&&dst.cols>(src.cols-1)*Multi&&dst.rows<=src.rows*Multi&&dst.cols<=src.cols*Multi); 
	if (Multi==1)
		src.copyTo(dst);
	else
	{
	int lengthrow,lengthcol;
	const int rows = src.rows;  
    const int cols = src.cols; 
	CV_Assert(src.isContinuous()&&dst.isContinuous());
	float *addr=(float *)src.data;	
	float *dstaddr=(float *)dst.data;
	int j,i,li,lj;
	
	for (int k=0;k<rows*cols;k++)
	{
	j=k%cols;
	i=k/cols;
	lengthrow=(i==rows-1)?(dst.rows-(src.rows-1)*Multi):(Multi);
	lengthcol=(j==cols-1)?(dst.cols-(src.cols-1)*Multi):(Multi);
	for (int l=0;l<lengthrow*lengthcol;l++)
	{
		li=l/lengthcol;
		lj=l%lengthcol;
		*(dstaddr+(i*Multi+li)*dst.cols+j*Multi+lj)=*(addr+k);
		
	}
	}

	}
}


void salmat::MapDownSample(Mat& src,Mat& dst,int Multi)
{
	CV_Assert(src.depth() == CV_32FC1&&dst.depth() == CV_32FC1);
	CV_Assert(src.rows>(dst.rows-1)*Multi&&src.cols>(dst.cols-1)*Multi&&src.rows<=dst.rows*Multi&&src.cols<=dst.cols*Multi); 
	if (Multi==1)
		src.copyTo(dst);
	else
	{
	const int rows = dst.rows;  
    const int cols = dst.cols; 
	
	CV_Assert(dst.isContinuous()&&src.isContinuous());
	float *srcaddr=(float *)src.data;	
	float *dstaddr=(float *)dst.data;
	int j,i;
	
	for (int k=0;k<rows*cols;k++)
	{
	j=k%cols;
	i=k/cols;
	*(dstaddr+k)=*(srcaddr+i*Multi*src.cols+j*Multi);
	}
	}
}


void salmat::MatrixNorm(Mat src, Mat& dst)
{	
	double min,max;
	minMaxIdx(src,&min,&max);
	if (max>0)
		dst=src/max;
	else
		src.copyTo(dst);
}


void salmat::MatrixNorm2(Mat src, Mat& dst)
{	
	double min,max;
	minMaxIdx(src,&min,&max);
	dst=(src-min);
	minMaxIdx(dst,&min,&max);	
	if (max>0)
		dst=dst/max;
	
}

void salmat::MapThreshold(Mat& src,Mat& dst)
{
double MeanValue,temptresh,temptresh2,tempmin,tempmax; 
float* addr=(float*)src.data;
float* dstaddr=(float*)dst.data;
Mat temp;
Mat mask(src.size(),CV_8UC1);
MeanValue=mean(src).val[0];
minMaxIdx(src,&tempmin,&tempmax);
temptresh=tempmax-MeanValue;
if(temptresh==0||temptresh>=tempmax)
	src.copyTo(dst);
else
{
	float sum=0;
	int count=0;
	for(int k=0;k<src.rows*src.cols;k++)
	{
	if(*(addr+k)>temptresh)
		{sum=sum+*(addr+k);
		count++;
		}
	}
	temptresh=sum/count;
	if(temptresh==0)
		src.copyTo(dst);
	else
	{
	for(int k=0;k<src.rows*src.cols;k++)
	{
	if(*(addr+k)>=temptresh)
			*(dstaddr+k)=1;
	else
		*(dstaddr+k)=*(addr+k)/temptresh;
	}
	}
}
}


void salmat::GenerateOriMap()
{	
	int flitersize;
	Mat TempmatS(ThisFrameValue.size(),ThisFrameValue.type());
	Mat TempmatL(ThisFrameValue.size()*4,ThisFrameValue.type());	
	MapThreshold(ThisFrameValue,TempmatS);
	MatrixDuplication(TempmatS,TempmatL,4);
	MatrixNorm(TempmatL,OriMap);	 
}



void salmat::GenerateTemporalMap(UInt FrameIndex,int *mvx,int* mvy)
{
	Mat tempmat;
	UInt LCU_HEIGHT=ceilf((float)VIDEO_HIGHT/MinSize);
	UInt LCU_WIDTH=ceilf((float)VIDEO_WIDTH/MinSize);
	Mat TempmatL(ThisFrameValue.size()*4,ThisFrameValue.type());
	Mat TempmatS(LCU_HEIGHT,LCU_WIDTH,ThisFrameValue.type());
	Mat summat=Mat::zeros(TempmatS.size(), TempmatS.type());
	int summvx=0;
	int summvy=0;
	int Index=FrameIndex%PreFrameTemp;
	int RealIndex,flitersize;
	MapDownSample(ThisFrameValue,TempmatS,MinSize/4);
	TempmatS.copyTo(TemporalFrame[FrameIndex%PreFrameTemp]);

	if (FrameIndex<PreFrameTemp)
	{		
		for(int i=1;i<FrameIndex+1;i++)
		{
			summvx=summvx+mvx[FrameIndex-i];
			summvy=summvy+mvy[FrameIndex-i];
			translateTransform(TemporalFrame[FrameIndex-i],tempmat,cvRound(summvx/MinSize),cvRound(summvy/MinSize));
			summat=summat+abs(tempmat-TempmatS)*exp(-i/DeltaTemporal);
		}
	}
	else
	{
		for(int i=1;i<PreFrameTemp;i++)
		{
			if(Index-i<0)
				RealIndex=Index-i+PreFrameTemp;
			else
				RealIndex=Index-i;
			summvx=summvx+mvx[FrameIndex-i];
			summvy=summvy+mvy[FrameIndex-i];
			translateTransform(TemporalFrame[RealIndex],tempmat,cvRound(summvx/MinSize),cvRound(summvy/MinSize));
			summat=summat+abs(tempmat-TempmatS)*exp(-i/DeltaTemporal);
		}
	}

	
	 
	MatrixDuplication(summat,TempmatL,MinSize);
	MatrixNorm(TempmatL,TemporalMap);
	MapThreshold(TemporalMap,TemporalMap);

}


void salmat::GenerateTemporalMap_mv(UInt FrameIndex,int *mvx,int* mvy,salmat& Mvx,salmat& Mvy,Mat& dst)
{
	Mat tempmatx,tempmaty,dtempx,dtempy,tempxy;
	UInt LCU_HEIGHT=ceilf((float)VIDEO_HIGHT/Mvx.MinSize);
	UInt LCU_WIDTH=ceilf((float)VIDEO_WIDTH/Mvx.MinSize);
	Mat TempmatL(Mvx.ThisFrameValue.size()*4,Mvx.ThisFrameValue.type());
	Mat TempmatSx(LCU_HEIGHT,LCU_WIDTH,Mvx.ThisFrameValue.type());
	Mat TempmatSy(LCU_HEIGHT,LCU_WIDTH,Mvx.ThisFrameValue.type());

	Mat summat=Mat::zeros(TempmatSx.size(),TempmatSx.type());
	int summvx=0;
	int summvy=0;
	int Index=FrameIndex%Mvx.PreFrameTemp;
	int RealIndex,flitersize;
	MapDownSample(Mvx.ThisFrameValue,TempmatSx,Mvx.MinSize/4);
	MapDownSample(Mvy.ThisFrameValue,TempmatSy,Mvx.MinSize/4);
	TempmatSx.copyTo(Mvx.TemporalFrame[FrameIndex%Mvx.PreFrameTemp]);
	TempmatSy.copyTo(Mvy.TemporalFrame[FrameIndex%Mvx.PreFrameTemp]);
	if (FrameIndex<Mvx.PreFrameTemp)
	{		
		for(int i=1;i<FrameIndex;i++)
		{
			summvx=summvx+mvx[FrameIndex-i];
			summvy=summvy+mvy[FrameIndex-i];
			translateTransform(Mvx.TemporalFrame[FrameIndex-i],tempmatx,cvRound(summvx/Mvx.MinSize),cvRound(summvy/Mvx.MinSize));
			translateTransform(Mvy.TemporalFrame[FrameIndex-i],tempmaty,cvRound(summvx/Mvx.MinSize),cvRound(summvy/Mvx.MinSize));
			dtempx=tempmatx-TempmatSx;
			dtempy=tempmaty-TempmatSy;
			CalDist(dtempx,dtempy,tempxy);
			summat=summat+tempxy*exp(-i/Mvx.DeltaTemporal);
		}
	}
	else
	{
		for(int i=1;i<Mvx.PreFrameTemp;i++)
		{
			if(Index-i<0)
				RealIndex=Index-i+Mvx.PreFrameTemp;
			else
			RealIndex=Index-i;
			summvx=summvx+mvx[FrameIndex-i];
			summvy=summvy+mvy[FrameIndex-i];
			translateTransform(Mvx.TemporalFrame[RealIndex],tempmatx,cvRound(summvx/Mvx.MinSize),cvRound(summvy/Mvx.MinSize));
			translateTransform(Mvy.TemporalFrame[RealIndex],tempmaty,cvRound(summvx/Mvx.MinSize),cvRound(summvy/Mvx.MinSize));
			dtempx=tempmatx-TempmatSx;
			dtempy=tempmaty-TempmatSy;
			CalDist(dtempx,dtempy,tempxy);
			summat=summat+tempxy*exp(-i/Mvx.DeltaTemporal);
		}
	}

	MatrixDuplication(summat,TempmatL,Mvx.MinSize);
	MatrixNorm(TempmatL,dst);
	MapThreshold(dst,dst);
}


void salmat::GenerateSpatialMap()
{	
	int flitersize,colnum,rownum,len,i,j;
	float rownumold,colnumold,thisvalue=0;

	double timec1=static_cast<double>(getTickCount());
	colnumold = (float)ThisFrameValue.cols;
	rownumold = (float)ThisFrameValue.rows;
	colnum=ceilf(colnumold/(MinSize/4));
	rownum=ceilf(rownumold/(MinSize/4));
	Mat TempmatS(rownum,colnum,ThisFrameValue.type());
	Mat TempmatM(ThisFrameValue.size(),ThisFrameValue.type());
	
	Mat TempMat(SpatialSize,SpatialSize,ThisFrameValue.type());
	Mat TempmatL(ThisFrameValue.size()*4,ThisFrameValue.type());
	Mat TempOut(rownum,colnum,ThisFrameValue.type());
	MapThreshold(ThisFrameValue,TempmatM);
	MapDownSample(TempmatM,TempmatS,MinSize/4);
	len=floorf((float)SpatialSize/2);
	Mat TempmatSEx=Mat::zeros(rownum+2*len,colnum+2*len,ThisFrameValue.type());
	Mat WeightMap=GenerateWeightMap(SpatialSize,SpatialSize);
	TempmatS.copyTo(TempmatSEx(Rect(len,len,colnum,rownum)));
	MatrixNorm2(TempmatSEx,TempmatSEx);

	CV_Assert(TempmatSEx.isContinuous()&&TempOut.isContinuous()); 
	float* outaddr=(float*)TempOut.data;
	float* seaddr=(float*)TempmatSEx.data;
	for (int k=0;k<rownum*colnum;k++)
	{	
		i=len+k/colnum;
		j=len+k%colnum;
		thisvalue=*(seaddr+i*TempmatSEx.cols+j);
		TempmatSEx(Rect(j-len,i-len,SpatialSize,SpatialSize)).copyTo(TempMat);
		TempMat=abs(TempMat-thisvalue);
		TempMat=TempMat.mul(WeightMap);
		*(outaddr+k)=sum(TempMat).val[0];
	}
	MatrixNorm(TempOut,TempOut);
	MatrixDuplication(TempOut,TempmatL,MinSize);
	MatrixNorm(TempmatL,SpatialMap);
}


Mat salmat::GenerateWeightMap(int row,int col)
{	
	int lenrow,lencol,i,j;
	float tempdis;
	Mat weightmap(row,col,CV_32FC1);
	float* addr=(float*)weightmap.data;
	lenrow=floorf((float)row/2);
	lencol=floorf((float)col/2);
	for (int k=0;k<row*col;k++)
	{
		i=k/col;
		j=k%col;
		tempdis=pow(float(i-lenrow),2)+pow(float(j-lencol),2);
		*(addr+k)=exp(-tempdis/(2*pow(DeltaSpatial,2)));
	}
	return weightmap;
}


void salmat::GenerateSpatialMap_mv()
{	
	int flitersize,colnum,rownum,len,i,j;
	float rownumold,colnumold,thisvalue;
	colnumold = (float)ThisFrameValue.cols;
	rownumold = (float)ThisFrameValue.rows;
	colnum=ceilf(colnumold/(MinSize/4));
	rownum=ceilf(rownumold/(MinSize/4));
	Mat TempmatS(rownum,colnum,ThisFrameValue.type());
	Mat TempmatM(ThisFrameValue.size(),ThisFrameValue.type());
	Mat TempmatSEx;
	Mat TempMat(SpatialSize,SpatialSize,ThisFrameValue.type());
	Mat TempmatL(ThisFrameValue.size()*4,ThisFrameValue.type());
	Mat TempOut(rownum,colnum,ThisFrameValue.type());

	MapThreshold(ThisFrameValue,TempmatM);
	MapDownSample(TempmatM,TempmatS,MinSize/4);
	len=floorf((float)SpatialSize/2);
	Mat WeightMap=GenerateWeightMap(SpatialSize,SpatialSize);
	TempmatSEx=Mat::zeros(rownum+2*len,colnum+2*len,ThisFrameValue.type());
	TempmatS.copyTo(TempmatSEx(Rect(len,len,colnum,rownum)));
	MatrixNorm2(TempmatSEx,TempmatSEx);
	
	CV_Assert(TempmatSEx.isContinuous()&&TempOut.isContinuous()); 
	float* outaddr=(float*)TempOut.data;
	float* seaddr=(float*)TempmatSEx.data;
	for (int k=0;k<rownum*colnum;k++)
	{	
		i=len+k/colnum;
		j=len+k%colnum;
		thisvalue=*(seaddr+i*TempmatSEx.cols+j);
		TempmatSEx(Rect(j-len,i-len,SpatialSize,SpatialSize)).copyTo(TempMat);
		TempMat=abs(TempMat-thisvalue);
		TempMat=TempMat.mul(WeightMap);
		*(outaddr+k)=sum(TempMat).val[0];
	}
	MatrixNorm(TempOut,TempOut);
	MatrixDuplication(TempOut,TempmatL,MinSize);
	TempmatL.copyTo(SpatialMap);
}	

 void salmat::GenerateSpatialMap_mv2(Mat& srcx,Mat& srcy,Mat& dst)
 {
	
	 int flitersize;
	 Mat tempsum,temp;
	tempsum=srcx.mul(srcx)+srcy.mul(srcy);
	sqrt(tempsum,temp);
	MatrixNorm(temp,dst);
 }

 void salmat::translateTransform(cv::Mat const& src, cv::Mat& dst, int dx, int dy)  //re
{  
    CV_Assert(src.depth() == CV_32FC1);  
    const int rows = src.rows;  
    const int cols = src.cols;  
	int i,j;
	if(dx==0&&dy==0)
		src.copyTo(dst);
	else
	{
	Mat dst2=Mat::zeros(rows, cols, src.type());
	dst2.copyTo(dst);
	if (abs(dx)<cols&&abs(dy)<rows)
	{
		int StartXsrc=dx>0?0:abs(dx);
		int StartYsrc=dy>0?0:abs(dy);
		int StartXdst=dx<=0?0:abs(dx);
		int StartYdst=dy<=0?0:abs(dy);
		Mat tempsrc=src(Rect(StartXsrc,StartYsrc,cols-abs(dx),rows-abs(dy)));
		Mat tempdst=dst(Rect(StartXdst,StartYdst,cols-abs(dx),rows-abs(dy)));
		tempsrc.copyTo(tempdst);
	}
	}

}  

 void salmat::CalDist(Mat& src1,Mat& src2, Mat& dst)
{
	Mat tempsum;
	tempsum=src1.mul(src1)+src2.mul(src2);
	sqrt(tempsum,dst);
}