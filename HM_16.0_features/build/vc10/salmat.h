#include "TLibCommon/CommonDef.h"
#include<opencv2/core/core.hpp>  //re
#include<opencv2/highgui/highgui.hpp>  
#include "cv.hpp"
#include "TLibDecoder/TDecCu.h"
#include<queue> 
//#include <Math.h>


#define DeltaTempMv 26;
#define DeltaTempBit 46;
#define DeltaTempDepth 46;
#define DeltaSpatialMv 11;
#define DeltaSpatialBit 3;
#define DeltaSpatialDepth 13;


using namespace cv;  
using namespace std; 



#pragma once

class salmat
{
public:
	UInt test; 
	Mat a[PreFrameSmooth];

	salmat(void);
	~salmat(void);


};

