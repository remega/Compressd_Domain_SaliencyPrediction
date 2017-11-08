// #include <windows.h>
// #include <stdlib.h>
// #include <stdio.h>
#include <string.h>
// #include "engine.h"
#include <mex.h>
#include<malloc.h>
#include<math.h>
using namespace std;

void img_extract(double *img_patch,const mxArray *ori_image,int startrow,int startcol, int length)
{
//mxArray *img_patch=NULL;
int orirow,oricol;
int j;
double *imgpoint;
//img_patch = mxCreateDoubleMatrix(length, length, mxREAL);
orirow = (int)mxGetM(ori_image);
oricol = (int)mxGetN(ori_image);
//patpoint = mxGetPr(img_patch);
imgpoint = mxGetPr(ori_image);
for (j=1;j<=length;j++)
	{
	memcpy((img_patch+(j-1)*length), (imgpoint+(startcol+j-2)*orirow+startrow-1), length*sizeof(double));
	}	

// return img_patch;
}

void outer_pro(double *out,double *a,double *b,int length)
{
	int i;
	// double *out;
	// out=(double *)malloc(length*sizeof(double));
	for(i=1;i<=length;i++)
	{
		*(out+i-1)=(*(a+i-1))*(*(b+i-1));
	}
}

double sumup(double *a,int length)
{
double out=0;
int i;
	for(i=1;i<=length;i++)
	{
		out=out+*(a+i-1);
	}
return out;
}

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
	double *temp_patch,*temp1,*my_maskPoint,*finPoint,*salmap;
	mxArray *my_image=NULL;
	int m,n,len,winsize,m1,n1;
	int row,col,i;
	double patch_mean,temp2;
	
	// my_image=prhs[0];
	my_maskPoint=mxGetPr(prhs[1]);
	m=(int)*(mxGetPr(prhs[4]));
	n=(int)*(mxGetPr(prhs[5]));
	m1 = mxGetM(prhs[0]);
	n1 = mxGetN(prhs[0]);
	len=(int)*(mxGetPr(prhs[2]));
	winsize=(int)*(mxGetPr(prhs[3]));
	salmap=mxGetPr(prhs[0]);
	//invalid_ind=mxGetPr(prhs[5]);
	
	temp_patch=(double *)malloc(winsize*winsize*sizeof(double));
	temp1=(double *)malloc(winsize*winsize*sizeof(double));
	plhs[0] = mxCreateDoubleMatrix(m, n, mxREAL);
	finPoint = mxGetPr(plhs[0]);
	
	
	for(row=1+len;row<=m+len;row++)
		{
		for(col=1+len;col<=n+len;col++)
			{
			img_extract(temp_patch,prhs[0],row-len,col-len,winsize);	
			// outer_pro(temp1,temp_patch,my_maskPoint,winsize*winsize);
			patch_mean=*(salmap+(col-1)*m1+row-1);	
			for (i=1;i<=winsize*winsize;i++)
			{
			if(*(temp_patch+i-1)!=0)
				*(temp_patch+i-1)=abs(*(temp_patch+i-1)-patch_mean);
			}
			outer_pro(temp1,temp_patch,my_maskPoint,winsize*winsize);
			temp2=sumup(temp1,winsize*winsize);
			*(finPoint+(col-len-1)*m+row-len-1)=temp2;	
			
			}		
		}

}