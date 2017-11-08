#include<iostream>
#include<string.h>
using namespace std;
int srem(int x, int y)
{
	int k = x%y;
	if (k)
		return k;
	else return y;
}
int depth_cal(int radii)
{
	int depth;
	if(radii == 64) 
		depth = 0;
	else if (radii == 32)
		depth = 1;
	else if(radii == 16) 
		depth = 2;
	else if(radii == 8) 
		depth = 3;
	else depth = 0;
	return depth;
}

int len_pu(int num_pu)
{
	int len;
	if(num_pu == 3)
	    len = 3;
	else if (num_pu == 15)
	    len = 0;
	else
	    len = 1;
	return len;
}
void printarr(int *arr,int arr_l)
{
	for(int i=0;i<arr_l;i++)
		cout<<arr[i]<<' ';
	cout<<endl;
}
int seq_pu(int num)
{
	int seq;
	if (num == 99)
    	seq = 0;
	else if (num == -1)
        seq = 4;
	else
    	seq = 1;
    return seq;
}

int ctoi(char *k)
{
	int a=0;
	for(int i=0;i<strlen(k);i++)
	{
		a=a*10+k[i]-'0';
	}
	return a;
}
void inf_32batch(int *l_first,int ii,int *l_start,int *l_end)
{
	int jj = 0;
	*l_start = 0;
	while(1)
	{
	    *l_start = *l_start + l_first[jj];
	    jj = jj + 1;
	    if (jj > 2 * ii)
	        break;
	}
	*l_start = *l_start;
	*l_end = *l_start + l_first[2 * ii+1];
 } 


int length_cal(int num)
{
	int len=0;
	if(num == -1)
	    len = 5;
	else if (num == -2)
	        len = 9;
	else if (num == -3)
	        len = 13;
	else if (num == -4)
	        len = 17;
	else if (num == -5)
	        len = 21;
	return len;
}
int seq_cal(int num)
{
	int seq=0;
	if (num == -1)
	    seq = 4;
	else if (num == -2)
	        seq = 7;
	else if (num == -3)
	        seq = 10;
	else if (num == -4)
	        seq = 13;
	else if (num == -5)
	        seq = 16;
	else if (num == 99)
	        seq = 0;
	else
	    seq = 1;
	return seq;
}
void pre_pro(int *add_99_0,int add_99_0_l,int *l,int *l_l)//,int *add,int *add_l
{
	//根据tst1判断需要再次分割的32x32的子块
	int add[1000],add_l=0;
	for(int i=0;i<add_99_0_l;i++)
	{
		if(add_99_0[i] < 0)
		{
			add[add_l]=i;
			add_l++;
		}
	}
	//l获得每个子块对应的分割信息数组
	*l_l=2 * add_l;
	l[0] = add[0];
	int len = length_cal(add_99_0[add[0]]);
	l[1] = len;
	for (int ii = 1;ii<add_l;ii++)
	{
		l[ii * 2] = add[ii] - add[ii-1] - 1;
	    len = length_cal(add_99_0[add[ii]]);
	    l[ii * 2+1] = len;
	}
	    
}
int change99(int a)
{
	if(a==99) a=-1;
	return a;
}
int find99xxxx(int *arr,int la,int *brr)
{
	int num=1;
	brr[0]=99;
	for(int i=1;i<la;i++)
	{
		if(arr[i]==99&&arr[i+1]!=99&&arr[i+2]!=99&&arr[i+3]!=99&&arr[i+4]!=99) {
			brr[num]=change99(arr[i]) + change99(arr[i+1]) + change99(arr[i+2]) + change99(arr[i+3]) + change99(arr[i+4]);;
			i+=4;
		}
		else brr[num]=arr[i];
		num++;
	}
	return num;
}
int find990000(int *arr, int la, int *brr)
{
	int num = 1;
	brr[0] = 99;
	for (int i = 1; i<la; i++)
	{
		if (arr[i] == 99 && arr[i + 1] == 0 && arr[i + 2] == 0 && arr[i + 3] == 0 && arr[i + 4] == 0) {
			brr[num] = -1;
			i += 4;
		}
		else brr[num] = arr[i];
		num++;
	}
	return num;
}
int find990000(float *arr,int la,int *brr)
{
	int num=1;
	brr[0]=99;
	for(int i=1;i<la;i++)
	{
		if(arr[i]==99&&arr[i+1]==0&&arr[i+2]==0&&arr[i+3]==0&&arr[i+4]==0) {
			brr[num]=-1;
			i+=4;
		}
		else brr[num]=arr[i];
		num++;
	}
	return num;
}
void pu(float *arr,int arr_l,int *seq,int *seq_l,int *pu_number) 
{
	int inf_pu[1000],inf_pu_l,inf_pu_temp[1000],addr_pu[1000],addr_pu_l=0;
	inf_pu_l = find990000(arr,arr_l,inf_pu);
	for(int i=0;i<inf_pu_l;i++)
	{
		if(inf_pu[i] != 99) inf_pu_temp[i]=inf_pu[i];
		else inf_pu_temp[i] = -1;
		
		if(inf_pu_temp[i]>0) {
			pu_number[addr_pu_l]=inf_pu_temp[i];
			addr_pu[addr_pu_l]=i;
			seq[addr_pu_l]=1;
			addr_pu_l++;
		}
		
	}
	*seq_l=addr_pu_l;
	for (int ii=0;ii<addr_pu_l;ii++)
	{
	    int jj = 0;
	    while(1)
	    {
	        int num_current = seq_pu(inf_pu[jj]);
	        seq[ii] = seq[ii] + num_current;
	        jj = jj + 1;
	        if (jj >= addr_pu[ii])
	            break;
	    }
	}
} 