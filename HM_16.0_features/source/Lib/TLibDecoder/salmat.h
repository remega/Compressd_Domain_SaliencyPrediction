#include "TLibCommon/CommonDef.h"
#include<opencv2/core/core.hpp>  //re
#include<opencv2/highgui/highgui.hpp>  
#include "cv.hpp"
#include "TLibDecoder/TDecCu.h"
#include<queue> 
#include <opencv2/opencv_lib.h>
//#include <Math.h>





using namespace cv;  
using namespace std; 



#pragma once

class salmat
{
public:
	//UInt test; 

	Mat* SmoothFrame;
	Mat* TemporalFrame;
	//Mat* AbsStartAdress_SmoothFrame;
	//Mat* AbsEndAdress_SmoothFrame;
	//Mat* CurrentAdress_SmoothFrame;
	Mat ThisFrameValue;
	Mat OriMap;
	Mat TemporalMap;
	Mat SpatialMap;
	float DeltaSpatial;
	float DeltaTemporal;
	int SpatialSize;
	int MinSize;
	int PreFrameSmooth;
	int PreFrameTemp;
	int rows;
	int cols;
public:
	salmat(float Delta1,float Delta2,int Size1,int Size2,int row,int col);
	salmat(void);
	~salmat(void);
	void Initial(); 
	void ForwardFilter(UInt FrameIndex);
	static void MatrixDuplication(Mat src, Mat& dst, int Multi);
    static void MapDownSample(Mat& src,Mat& dst,int Multi);
	static void MatrixNorm(Mat src, Mat& dst);
	static void MatrixNorm2(Mat src, Mat& dst);
	void GenerateOriMap();
	static void MapThreshold(Mat& src,Mat& dst);
	void GenerateSpatialMap();
	Mat GenerateWeightMap(int row,int col);
	void GenerateSpatialMap_mv();
	static void GenerateSpatialMap_mv2(Mat& srcx,Mat& srcy,Mat& dst);
	void GenerateTemporalMap(UInt FrameIndex,int *mvx,int* mvy);
	static void salmat::GenerateTemporalMap_mv(UInt FrameIndex,int *mvx,int* mvy,salmat& Mvx,salmat& Mvy,Mat& dst);
	static void translateTransform(cv::Mat const& src, cv::Mat& dst, int dx, int dy);  //re
	static void CalDist(Mat& src1,Mat& src2, Mat& dst);
};

