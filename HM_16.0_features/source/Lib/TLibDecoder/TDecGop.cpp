/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2010-2014, ITU/ISO/IEC
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *  * Neither the name of the ITU/ISO/IEC nor the names of its contributors may
 *    be used to endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 */

/** \file     TDecGop.cpp
    \brief    GOP decoder class
*/

#include "TDecGop.h"
#include "TDecCAVLC.h"
#include "TDecSbac.h"
#include "TDecBinCoder.h"
#include "TDecBinCoderCABAC.h"
#include "libmd5/MD5.h"
#include "TLibCommon/SEI.h"
#include "TDecCu.h"         //yangren
#include <time.h>
#include <Math.h>
//==========================================
//re
#include<fstream>  
#include<opencv2/core/core.hpp>  
#include<opencv2/highgui/highgui.hpp>  
#include "cv.hpp"
#include "salmat.h"
#include "code_mat_h.h"
#include <opencv2/opencv_lib.h>
//#include <Math.h>
//===========================================
#define PI 3.1415926

using namespace cv;  
using namespace std; 
#define DeltaTempMv 26
#define DeltaTempBit 46
#define DeltaTempDepth 46
#define DeltaSpatialMv 11
#define DeltaSpatialBit 3
#define DeltaSpatialDepth 13
#define PatchSizeMv 12
#define PatchSizeBit 5
#define PatchSizeDepth 12
string dir= "../HMdata/";//the output path


float TDecCu::bits[LCUNUM] = {0};//yangren
float TDecCu::MV[LCUNUM][1000] = {0};//yangren
float TDecCu::CUPU[LCUNUM][1000] = {0};//yangren
float TDecCu::Pred[LCUNUM][1000] = {0};//yangren
salmat BitAllocation(DeltaSpatialBit,DeltaTempBit,PatchSizeBit,64,VIDEO_HIGHT,VIDEO_WIDTH);
salmat LCUDepth(DeltaSpatialDepth,DeltaTempDepth,PatchSizeDepth,32,VIDEO_HIGHT,VIDEO_WIDTH);
salmat MotinVectorX(DeltaSpatialMv,DeltaTempMv,PatchSizeMv,32,VIDEO_HIGHT,VIDEO_WIDTH);
salmat MotinVectorY(DeltaSpatialMv,DeltaTempMv,PatchSizeMv,32,VIDEO_HIGHT,VIDEO_WIDTH);
salmat MotinVectorNorm(DeltaSpatialMv,DeltaTempMv,PatchSizeMv,32,VIDEO_HIGHT,VIDEO_WIDTH);

float mv_x_extend2[VIDEO_HIGHT/4][VIDEO_WIDTH/4]={0};
float mv_y_extend2[VIDEO_HIGHT/4][VIDEO_WIDTH/4]={0};
float depth_extend2[VIDEO_HIGHT/4][VIDEO_WIDTH/4]={0};


double alltime=0;
int CameraMotion_x[FRAMENUM];
int CameraMotion_y[FRAMENUM];

float FeatureWeight[9]={0.058,0.171,-0.095,0.068,-0.01,0.509,-0.051,0.154,0.196};
//int count=0;

extern Bool g_md5_mismatch; ///< top level flag to signal when there is a decode problem

//! \ingroup TLibDecoder
//! \{
static Void calcAndPrintHashStatus(TComPicYuv& pic, const SEIDecodedPictureHash* pictureHashSEI);

//==========================================
//re

void MatArrayLeftShift(Mat* Matarray,UInt lenth, Mat const& NewMat);
void CameraDect(Mat& bitmap,Mat& mvmap_x, Mat& mvmap_y,Mat& m_norm,int& mov_x, int& mov_y);
void CalDist(Mat& src1,Mat& src2, Mat& dst);
int max(int*arr, int len);
void vote_alg(Mat& ref,Mat& srcx,Mat& srcy, float& mov_x,float& mov_y,int BackNum);
//void QuickSort(float e[], int first, int end);
float Umean(float*arr, int len);
void MergeSort(float arr[], int start, int end);
void Merge(float arr[], int start, int mid, int end);
float MeanArry(float* arry,int len);
void matoutyput(Mat& m);

//==========================================

// ====================================================================================================================
// Constructor / destructor / initialization / destroy
// ====================================================================================================================

TDecGop::TDecGop()
{
  m_dDecTime = 0;
  m_pcSbacDecoders = NULL;
  m_pcBinCABACs = NULL;
}

TDecGop::~TDecGop()
{

}

Void TDecGop::create()
{

}


Void TDecGop::destroy()
{
}

Void TDecGop::init( TDecEntropy*            pcEntropyDecoder,
                   TDecSbac*               pcSbacDecoder,
                   TDecBinCABAC*           pcBinCABAC,
                   TDecCavlc*              pcCavlcDecoder,
                   TDecSlice*              pcSliceDecoder,
                   TComLoopFilter*         pcLoopFilter,
                   TComSampleAdaptiveOffset* pcSAO
                   )
{
  m_pcEntropyDecoder      = pcEntropyDecoder;
  m_pcSbacDecoder         = pcSbacDecoder;
  m_pcBinCABAC            = pcBinCABAC;
  m_pcCavlcDecoder        = pcCavlcDecoder;
  m_pcSliceDecoder        = pcSliceDecoder;
  m_pcLoopFilter          = pcLoopFilter;
  m_pcSAO  = pcSAO;
}


// ====================================================================================================================
// Private member functions
// ====================================================================================================================
// ====================================================================================================================
// Public member functions
// ====================================================================================================================

Void TDecGop::decompressSlice(TComInputBitstream* pcBitstream, TComPic*& rpcPic)
{
  TComSlice*  pcSlice = rpcPic->getSlice(rpcPic->getCurrSliceIdx());
  // Table of extracted substreams.
  // These must be deallocated AND their internal fifos, too.
  TComInputBitstream **ppcSubstreams = NULL;

  //-- For time output for each slice
//  clock_t iBeforeTime = clock();
  m_pcSbacDecoder->init( (TDecBinIf*)m_pcBinCABAC );
  m_pcEntropyDecoder->setEntropyDecoder (m_pcSbacDecoder);

  UInt uiNumSubstreams = pcSlice->getPPS()->getEntropyCodingSyncEnabledFlag() ? pcSlice->getNumEntryPointOffsets()+1 : pcSlice->getPPS()->getNumSubstreams();

  // init each couple {EntropyDecoder, Substream}
  UInt *puiSubstreamSizes = pcSlice->getSubstreamSizes();
  ppcSubstreams    = new TComInputBitstream*[uiNumSubstreams];
  m_pcSbacDecoders = new TDecSbac[uiNumSubstreams];
  m_pcBinCABACs    = new TDecBinCABAC[uiNumSubstreams];
  for ( UInt ui = 0 ; ui < uiNumSubstreams ; ui++ )
  {
    m_pcSbacDecoders[ui].init(&m_pcBinCABACs[ui]);
    ppcSubstreams[ui] = pcBitstream->extractSubstream(ui+1 < uiNumSubstreams ? puiSubstreamSizes[ui] : pcBitstream->getNumBitsLeft());
  }

  for ( UInt ui = 0 ; ui+1 < uiNumSubstreams; ui++ )
  {
    m_pcEntropyDecoder->setEntropyDecoder ( &m_pcSbacDecoders[uiNumSubstreams - 1 - ui] );
    m_pcEntropyDecoder->setBitstream      (  ppcSubstreams   [uiNumSubstreams - 1 - ui] );
    m_pcEntropyDecoder->resetEntropy      (pcSlice);
  }

  m_pcEntropyDecoder->setEntropyDecoder ( m_pcSbacDecoder  );
  m_pcEntropyDecoder->setBitstream      ( ppcSubstreams[0] );
  m_pcEntropyDecoder->resetEntropy      (pcSlice);

  m_pcSbacDecoders[0].load(m_pcSbacDecoder);
  m_pcSliceDecoder->decompressSlice( ppcSubstreams, rpcPic, m_pcSbacDecoder, m_pcSbacDecoders);//Decode
  m_pcEntropyDecoder->setBitstream(  ppcSubstreams[uiNumSubstreams-1] );
  // deallocate all created substreams, including internal buffers.
  for (UInt ui = 0; ui < uiNumSubstreams; ui++)
  {
    ppcSubstreams[ui]->deleteFifo();
    delete ppcSubstreams[ui];
  }
  delete[] ppcSubstreams;
  delete[] m_pcSbacDecoders; m_pcSbacDecoders = NULL;
  delete[] m_pcBinCABACs; m_pcBinCABACs = NULL;

//  m_dDecTime += (Double)(clock()-iBeforeTime) / CLOCKS_PER_SEC;



UInt LCU_HEIGHT=ceilf((float)VIDEO_HIGHT/64);
UInt LCU_WIDTH=ceilf((float)VIDEO_WIDTH/64);
UInt Q_HEIGHT=ceilf((float)VIDEO_HIGHT/4);
UInt Q_WIDTH=ceilf((float)VIDEO_WIDTH/4);

UInt FrameIndex;
FrameIndex=pcSlice->getPOC();

cout<<FrameIndex<<endl;



//ofstream SaveFile("D:/restest2.txt");
//   for (int aa=0;aa<LCUNUM;aa++)
//   {
//	   for(UInt bb = 0; bb < 1000; bb ++)
//	  {
//	     if(TDecCu::MV[aa][bb]==999) break;
//		 SaveFile<<TDecCu::MV[aa][bb]<<" ";
//	  }
//	   SaveFile<<endl;
//	}
//
//    SaveFile<<endl<<endl<<endl;
//   for (int aa=0;aa<LCUNUM;aa++)
//   {
//	   for(UInt bb = 0; bb < 1000; bb ++)
//	  {
//	     if(TDecCu::CUPU[aa][bb]==999) break;
//		 SaveFile<<TDecCu::CUPU[aa][bb]<<" ";
//	  }
//	   SaveFile<<endl;
//	}
//      SaveFile<<endl<<endl<<endl;
//   for (int aa=0;aa<LCUNUM;aa++)
//   {
//	   for(UInt bb = 0; bb < 1000; bb ++)
//	  {
//	     if(TDecCu::Pred[aa][bb]==999) break;
//		 SaveFile<<TDecCu::Pred[aa][bb]<<" ";
//	  }
//	   SaveFile<<endl;
//	}
//
//SaveFile.close();
getFrame(TDecCu::MV,TDecCu::Pred,TDecCu::CUPU,mv_x_extend2,mv_y_extend2,depth_extend2);
Mat TempMvY(Q_HEIGHT,Q_WIDTH,CV_32FC1, *mv_y_extend2,Q_WIDTH*sizeof(float));
Mat TempMvX(Q_HEIGHT,Q_WIDTH,CV_32FC1, *mv_x_extend2,Q_WIDTH*sizeof(float));
Mat TempDepth(Q_HEIGHT,Q_WIDTH,CV_32FC1, *depth_extend2,Q_WIDTH*sizeof(float));
//

Mat TempBit(LCU_HEIGHT,LCU_WIDTH,CV_32FC1, TDecCu::bits,LCU_WIDTH*sizeof(float));
Mat BitMap(VIDEO_HIGHT/4,VIDEO_WIDTH/4,CV_32FC1); 
salmat::MatrixDuplication(TempBit, BitMap,BitAllocation.MinSize/4);
Mat mv_norm;

CameraDect(BitMap,TempMvX,TempMvY,mv_norm,CameraMotion_x[FrameIndex],CameraMotion_y[FrameIndex]);
salmat::MatrixNorm(TempBit,TempBit);
BitAllocation.ThisFrameValue.create(VIDEO_HIGHT/4,VIDEO_WIDTH/4,CV_32FC1); 
salmat::MatrixDuplication(TempBit,BitAllocation.ThisFrameValue,BitAllocation.MinSize/4);


BitAllocation.ForwardFilter(FrameIndex);
salmat::MatrixNorm(TempDepth,LCUDepth.ThisFrameValue);
LCUDepth.ForwardFilter(FrameIndex);
TempMvX.copyTo(MotinVectorX.ThisFrameValue);
MotinVectorX.ForwardFilter(FrameIndex);
TempMvY.copyTo(MotinVectorY.ThisFrameValue);
MotinVectorY.ForwardFilter(FrameIndex);


BitAllocation.GenerateOriMap();
LCUDepth.GenerateOriMap();
salmat::MatrixNorm(mv_norm,MotinVectorNorm.ThisFrameValue);
MotinVectorNorm.ForwardFilter(FrameIndex);
MotinVectorNorm.GenerateOriMap();




BitAllocation.GenerateTemporalMap(FrameIndex,CameraMotion_x,CameraMotion_y);
LCUDepth.GenerateTemporalMap(FrameIndex,CameraMotion_x,CameraMotion_y);
Mat MvTemporal;
salmat::GenerateTemporalMap_mv(FrameIndex,CameraMotion_x,CameraMotion_y,MotinVectorX,MotinVectorY,MvTemporal);


BitAllocation.GenerateSpatialMap();
LCUDepth.GenerateSpatialMap();
MotinVectorX.GenerateSpatialMap_mv();
MotinVectorY.GenerateSpatialMap_mv();
Mat MvSpatial;
salmat::GenerateSpatialMap_mv2(MotinVectorX.SpatialMap,MotinVectorY.SpatialMap,MvSpatial);


Mat FinalMap,FinalMap2;
FinalMap=MotinVectorNorm.OriMap*FeatureWeight[0]+MvTemporal*FeatureWeight[1]+MvSpatial*FeatureWeight[2]+BitAllocation.OriMap*FeatureWeight[3]+BitAllocation.TemporalMap*FeatureWeight[4]+BitAllocation.SpatialMap*FeatureWeight[5]+LCUDepth.OriMap*FeatureWeight[6]+LCUDepth.TemporalMap*FeatureWeight[7]+LCUDepth.SpatialMap*FeatureWeight[8];
int flitersize=cvRound(VIDEO_HIGHT/15);
if (flitersize%2==0)
		flitersize++;	
GaussianBlur(FinalMap,FinalMap,Size(flitersize,flitersize),cvRound(VIDEO_HIGHT/30),0,BORDER_CONSTANT);
salmat::MatrixNorm2(FinalMap,FinalMap);

FinalMap=FinalMap*255;
FinalMap.convertTo(FinalMap2,CV_8UC1);

char str[4];
sprintf(str,"%d",FrameIndex);
imwrite(dir+str+".png",FinalMap2);



}

Void TDecGop::filterPicture(TComPic*& rpcPic)
{
  TComSlice*  pcSlice = rpcPic->getSlice(rpcPic->getCurrSliceIdx());

  //-- For time output for each slice
//  clock_t iBeforeTime = clock();

  // deblocking filter
  Bool bLFCrossTileBoundary = pcSlice->getPPS()->getLoopFilterAcrossTilesEnabledFlag();
  m_pcLoopFilter->setCfg(bLFCrossTileBoundary);
  m_pcLoopFilter->loopFilterPic( rpcPic );

  if( pcSlice->getSPS()->getUseSAO() )
  {
    m_pcSAO->reconstructBlkSAOParams(rpcPic, rpcPic->getPicSym()->getSAOBlkParam());
    m_pcSAO->SAOProcess(rpcPic);
    m_pcSAO->PCMLFDisableProcess(rpcPic);
  }

  rpcPic->compressMotion();
  /*
  Char c = (pcSlice->isIntra() ? 'I' : pcSlice->isInterP() ? 'P' : 'B');
  if (!pcSlice->isReferenced()) c += 32;

  //-- For time output for each slice
  printf("POC %4d TId: %1d ( %c-SLICE, QP%3d ) ", pcSlice->getPOC(),
                                                  pcSlice->getTLayer(),
                                                  c,
                                                  pcSlice->getSliceQp() );

  m_dDecTime += (Double)(clock()-iBeforeTime) / CLOCKS_PER_SEC;
  printf ("[DT %6.3f] ", m_dDecTime );
  m_dDecTime  = 0;

  for (Int iRefList = 0; iRefList < 2; iRefList++)
  {
    printf ("[L%d ", iRefList);
    for (Int iRefIndex = 0; iRefIndex < pcSlice->getNumRefIdx(RefPicList(iRefList)); iRefIndex++)
    {
      printf ("%d ", pcSlice->getRefPOC(RefPicList(iRefList), iRefIndex));
    }
    printf ("] ");
  }
  if (m_decodedPictureHashSEIEnabled)
  {
    SEIMessages pictureHashes = getSeisByType(rpcPic->getSEIs(), SEI::DECODED_PICTURE_HASH );
    const SEIDecodedPictureHash *hash = ( pictureHashes.size() > 0 ) ? (SEIDecodedPictureHash*) *(pictureHashes.begin()) : NULL;
    if (pictureHashes.size() > 1)
    {
      printf ("Warning: Got multiple decoded picture hash SEI messages. Using first.");
    }
    calcAndPrintHashStatus(*rpcPic->getPicYuvRec(), hash);
  }

  printf("\n");*/

#if SETTING_PIC_OUTPUT_MARK
  rpcPic->setOutputMark(rpcPic->getSlice(0)->getPicOutputFlag() ? true : false);
#else
  rpcPic->setOutputMark(true);
#endif
  rpcPic->setReconMark(true);
}

/**
 * Calculate and print hash for pic, compare to picture_digest SEI if
 * present in seis.  seis may be NULL.  Hash is printed to stdout, in
 * a manner suitable for the status line. Theformat is:
 *  [Hash_type:xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx,(yyy)]
 * Where, x..x is the hash
 *        yyy has the following meanings:
 *            OK          - calculated hash matches the SEI message
 *            ***ERROR*** - calculated hash does not match the SEI message
 *            unk         - no SEI message was available for comparison
 */
static Void calcAndPrintHashStatus(TComPicYuv& pic, const SEIDecodedPictureHash* pictureHashSEI)
{
  /* calculate MD5sum for entire reconstructed picture */
  TComDigest recon_digest;
  Int numChar=0;
  const Char* hashType = "\0";

  if (pictureHashSEI)
  {
    switch (pictureHashSEI->method)
    {
      case SEIDecodedPictureHash::MD5:
        {
          hashType = "MD5";
          numChar = calcMD5(pic, recon_digest);
          break;
        }
      case SEIDecodedPictureHash::CRC:
        {
          hashType = "CRC";
          numChar = calcCRC(pic, recon_digest);
          break;
        }
      case SEIDecodedPictureHash::CHECKSUM:
        {
          hashType = "Checksum";
          numChar = calcChecksum(pic, recon_digest);
          break;
        }
      default:
        {
          assert (!"unknown hash type");
          break;
        }
    }
  }

  /* compare digest against received version */
  const Char* ok = "(unk)";
  Bool mismatch = false;

  if (pictureHashSEI)
  {
    ok = "(OK)";
    if (recon_digest != pictureHashSEI->m_digest)
    {
      ok = "(***ERROR***)";
      mismatch = true;
    }
  }

  printf("[%s:%s,%s] ", hashType, digestToString(recon_digest, numChar).c_str(), ok);

  if (mismatch)
  {
    g_md5_mismatch = true;
    printf("[rx%s:%s] ", hashType, digestToString(pictureHashSEI->m_digest, numChar).c_str());
  }
}
//! \}

//===========================================================================================================
//re




void MatArrayLeftShift(Mat* Matarray,UInt lenth, Mat const& NewMat)
{
	for (int i=0;i<lenth-1;i++)
	{
		Matarray[i+1].copyTo(Matarray[i]);
	}
	NewMat.copyTo(Matarray[lenth-1]);
}



void CameraDect(Mat& bitmap,Mat& mvmap_x, Mat& mvmap_y,Mat& m_norm,int& mov_x, int& mov_y)
{
	double min,max;	
	int BackNum=0,StaticNum=0;
	float mvx,mvy,temp;
//	Mat MaskedMvx,MaskedMvy,mask2,tempsum;
	//Mat tempx(bitmap.size(),bitmap.type());
	//Mat tempy(bitmap.size(),bitmap.type());
	Mat mask=Mat::zeros(bitmap.size(),bitmap.type());
	float* bitaddr=(float*)bitmap.data;
	float* maskaddr=(float*)mask.data;
	float* mvxaddr=(float*)mvmap_x.data;
	float* mvyaddr=(float*)mvmap_y.data;
	mvmap_x=mvmap_x/4;
	mvmap_y=mvmap_y/4;
	//medianBlur(mvmap_x ,mvmap_x, 3);//
	//medianBlur(mvmap_y ,mvmap_y, 3); 
	CV_Assert(mask.isContinuous()&&bitmap.isContinuous()&&mvmap_x.isContinuous()&&mvmap_y.isContinuous());
	minMaxIdx(bitmap,&min,&max);	
	for (int k=0;k<bitmap.rows*bitmap.cols;k++)
	{
		if(*(bitaddr+k)<min+20)
		{
			BackNum++;
			*(maskaddr+k)=1;
			temp=pow(*(mvxaddr+k),2)+pow(*(mvyaddr+k),2);
			if(temp<=2)
				StaticNum++;
		}
	}
	/*threshold(bitmap, mask, min+20, 1, THRESH_BINARY_INV);	
	BackNum=(int)sum(mask).val[0];
	MaskedMvx=mvmap_x.mul(mask);
	MaskedMvy=mvmap_y.mul(mask);
	tempsum=MaskedMvx.mul(MaskedMvx)+MaskedMvy.mul(MaskedMvy);	
	threshold(tempsum, mask2, 2, 1, THRESH_BINARY_INV);
	float a=sum(mask2).val[0];*/
	//StaticNum=(int)sum(mask2).val[0]-(bitmap.rows*bitmap.cols-BackNum);
	if(StaticNum<=BackNum/2)
	{
   
	vote_alg(mask,mvmap_x,mvmap_y,mvx,mvy,BackNum);
	mvmap_x=mvmap_x-mvx;
	mvmap_y=mvmap_y-mvy;
	
	CalDist(mvmap_x,mvmap_y,m_norm);
	mov_x=cvRound(mvx);
	mov_y=cvRound(mvy);

	//SaveFile2 <<" "<<mov_x<<" "<<mov_y<<endl;//<<endl<<LCUDepth.TemporalMap;
	}
	else
	{
	CalDist(mvmap_x,mvmap_y,m_norm);
	mov_x=0;
	mov_y=0;
	//SaveFile2 <<" "<<mov_x<<" "<<mov_y<<endl;
	}

}

void CalDist(Mat& src1,Mat& src2, Mat& dst)
{
	Mat tempsum;
	tempsum=src1.mul(src1)+src2.mul(src2);
	sqrt(tempsum,dst);
}

void vote_alg(Mat& ref,Mat& srcx,Mat& srcy, float& mov_x,float& mov_y,int BackNum)
{
	int row=srcx.rows;
	int col=srcx.cols;
	
	float *srcx_data=(float*)srcx.data;
	float *srcy_data=(float*)srcy.data;
	float *ref_data=(float*)ref.data;
	float **StatisX= new float*[16];
	float **StatisY= new float*[16];

	for (int i = 0; i < 16; i++)
	{	StatisX[i] = new float[BackNum];
		StatisY[i] = new float[BackNum];
	}
	
	float temp;
	int count[16]={0};
	int index,MaxIndex;
	CV_Assert(srcx.isContinuous()&&srcy.isContinuous()&&ref.isContinuous());
	
	for (int i=0;i<row*col;i++)
	{
		if((*(ref_data+i))!=0)
		{
		index=((atan2((*(srcy_data+i)),(*(srcx_data+i)))/PI)+1)*8.0;
		//index=(int)((atan2((*(srcx_data+i)),(*(srcy_data+i)))/PI)+1)*8.0;
		if(index==16)
			index--;
		StatisX[index][count[index]]=*(srcx_data+i);
		StatisY[index][count[index]]=*(srcy_data+i);
		count[index]++;
		}
	}
	MaxIndex=max(count,16);

	mov_x=Umean(*(StatisX+MaxIndex),count[MaxIndex]);
	mov_y=Umean(*(StatisY+MaxIndex),count[MaxIndex]);

}

int max(int*arr, int len)
{
 int temp,index;
 temp=*arr;
 index=0;
 for (int i=1;i<len;i++)
 {
  
  if(arr[i]>temp)
  { temp=arr[i]; 
	index=i;
  }
 }
 return index;
}

float Umean(float*arr, int len)
{
 float tempmean,mean;
 float sum=0;
 bool flag;
 int count=0;
	for(int i=0;i<len;i++)
	{
	sum=sum+arr[i];
	}
	mean=sum/len;
	sum=0;
	if(mean>0)
		flag=1;
	else
		flag=0;

	for(int i=0;i<len;i++)
	{
	if((flag&&arr[i]<mean)||(flag==0&&arr[i]>mean))
		sum=sum+arr[i];
	else
		sum=sum+mean;
	}
	mean=sum/len;
 //MergeSort(arr,0,len-1);
 //if (abs(arr[0])<abs(arr[len-1]))
	//{
	//   for (int i=0;i<len/2;i++)
	//	  sum=sum+arr[i];
	//   mean=sum/(len/2);
	// }
 //else
	// {
	//   for (int i=len/2+1;i<len;i++)
	//		sum=sum+arr[i];
	//   mean=sum/(len-(len/2));
	//}
 //return mean;
	return mean;
}


void Merge(float arr[], int start, int mid, int end)
{

int n1, n2;
n1 = mid - start + 1;
n2 = end - mid;
float *temp1=new float[n1];
float *temp2=new float[n2];

for (int i = 0; i < n1; i++)
{
temp1[i] = arr[start + i];
}

for (int i = 0; i < n2; i++)
{
temp2[i] = arr[mid + i + 1];
}

temp1[n1] = temp2[n2] = 1000;

for (int k = start, i = 0, j = 0; k <= end; k++)
{
if (temp1[i] <= temp2[j])
{
arr[k] = temp1[i];
i++;
}
else
{
arr[k] = temp2[j];
j++;
}
}
}


void MergeSort(float arr[], int start, int end)
{	
int i;
if (start < end)
{
	i = (end + start) / 2;
	MergeSort(arr, start, i);
	MergeSort(arr, i + 1, end);
	Merge(arr, start, i, end);
}
}

float MeanArry(float* arry,int len)
{	double sum=0,mean;
	for(int i=0;i<len;i++)
	{
	sum=sum+arry[i];
	}
	mean=sum/len;
	return mean;
}


void matoutyput(Mat& m)
{
salmat::MatrixNorm2(m,m);
m=m*255;
m.convertTo(m,CV_8UC1);
}