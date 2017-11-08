// code_mat_h.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include "code_mat_h.h"

ifstream f_cupu("decoder_cupu.txt");
ifstream f_mv("decoder_mv.txt");
ifstream f_pred("decoder_pred.txt");

ofstream f_mv_x("mv_x.txt");
ofstream f_mv_y("mv_y.txt");
ofstream f_depth("depth.txt");

#define width VIDEO_WIDTH
#define height VIDEO_HIGHT
#define numFrames FRAMENUM
#define stopNum 999

//float **mv_x_extend = new float*[int(height / 64.0 + 0.5) * 64];
//float **mv_y_extend = new float*[int(height / 64.0 + 0.5) * 64];
//float **depth_extend = new float*[int(height / 64.0 + 0.5) * 64];

float **cupu = new float*[int(height / 64.0 + 0.5) * 64];
float **mv = new float*[int(height / 64.0 + 0.5) * 64];
float **pred = new float*[int(height / 64.0 + 0.5) * 64];

void saveres(int x) {
	f_mv_x << x << endl;
	for (int i = 0; i < height / 4; i++)
	{
		for (int j = 0; j < width / 4; j++)
		{
			f_mv_x << mv_x_extend[i * 4][j * 4] << ' ';
		}
		f_mv_x << endl;
	}
	f_mv_y << x << endl;
	for (int i = 0; i < height / 4; i++)
	{
		for (int j = 0; j < width / 4; j++)
		{
			f_mv_y << mv_y_extend[i * 4][j * 4] << ' ';
		}
		f_mv_y << endl;
	}
	f_depth << x << endl;
	for (int i = 0; i < height / 4; i++)
	{
		for (int j = 0; j < width / 4; j++)
		{
			f_depth << depth_extend[i * 4][j * 4] << ' ';
		}
		f_depth << endl;
	}
	printf("%d\n", x);
}
void printres(int x) {
	printf("mv_x_extend %d\n", x);
	for (int i = 0; i < height / 4; i++)
	{
		for (int j = 0; j < width / 4; j++)
		{
			printf("%d ", mv_x_extend[i * 4][j * 4]);
		}
		printf("\n");
	}
	system("pause");
	printf("mv_y_extend %d\n", x);
	for (int i = 0; i < height / 4; i++)
	{
		for (int j = 0; j < width / 4; j++)
		{
			printf("%d ", mv_y_extend[i * 4][j * 4]);
		}
		printf("\n");
	}
	system("pause");
	printf("depth_extend %d\n", x);
	for (int i = 0; i < height / 4; i++)
	{
		for (int j = 0; j < width / 4; j++)
		{
			printf("%d ", depth_extend[i * 4][j * 4]);
		}
		printf("\n");
	}
	system("pause");
}
void read_frame_data()
{
	int num_patch = int(height / 64.0 + 0.5) * int(width / 64.0 + 0.5);
	char tmp[1000];
	std::string line;
	for (int numline = 0; numline < num_patch; numline++)
	{
		int cupu_l = 0;
		if (std::getline(f_cupu, line))
		{
			std::istringstream iss(line);
			cupu_l = 0;
			while (iss >> tmp)
			{
				if (tmp[0] == '<')
					continue;
				else if ((tmp[0] >= '0'&&tmp[0] <= '9')) {
					cupu[numline][cupu_l] = ctoi(tmp);
					cupu_l++;
				}
				else if ((tmp[0] == '-')) {
					cupu[numline][cupu_l] = -1 * ctoi(tmp + 1);
					cupu_l++;
				}
				else break;
			}
		}
		else break;
		cupu[numline][cupu_l] = stopNum;

		int mv_l = 0;
		if (std::getline(f_mv, line))
		{
			std::istringstream iss(line);
			mv_l = 0;
			while (iss >> tmp)
			{
				if (tmp[0] == '<')
					continue;
				else if ((tmp[0] >= '0'&&tmp[0] <= '9')) {
					mv[numline][mv_l] = ctoi(tmp);
					mv_l++;
				}
				else if ((tmp[0] == '-')) {
					mv[numline][mv_l] = -1 * ctoi(tmp + 1);
					mv_l++;
				}
				else break;
			}
		}
		else break;
		mv[numline][mv_l] = stopNum;

		int pred_l = 0;
		if (std::getline(f_pred, line))
		{
			std::istringstream iss(line);
			pred_l = 0;
			while (iss >> tmp)
			{
				if (tmp[0] == '<')
					continue;
				else if ((tmp[0] >= '0'&&tmp[0] <= '9')) {
					pred[numline][pred_l] = ctoi(tmp);
					pred_l++;
				}
				else if ((tmp[0] == '-')) {
					pred[numline][pred_l] = -1 * ctoi(tmp + 1);
					pred_l++;
				}
				else break;
			}
		}
		else break;
		pred[numline][pred_l] = stopNum;
	}
}
//void conarr()
//{
//	for (int i = 0; i < int(height / 64.0 + 0.5) * 64; i++)
//	{
//		mv_x_extend[i] = new float[width];
//		mv_y_extend[i] = new float[width];
//		depth_extend[i] = new float[width];
//
//		cupu[i] = new float[width];
//		mv[i] = new float[width];
//		pred[i] = new float[width];
//	}
//}
void delarr()
{
	for (int i = 0; i < int(height / 64.0 + 0.5) * 64; i++)
	{
		delete[]mv_x_extend[i];
		delete[]mv_y_extend[i];
		delete[]depth_extend[i];

		delete[]cupu[i];
		delete[]mv[i];
		delete[]pred[i];
	}
	delete[]mv_x_extend;
	delete[]mv_y_extend;
	delete[]depth_extend;

	delete[]cupu;
	delete[]mv;
	delete[]pred;
}
//int main()
//{
//	conarr();
//	//construct arrays : mv, pred, cupu, mv_x_extend, mv_y_extend, depth_extend
//
//	for (int numframe = 0; numframe < numFrames; numframe++)
//	{
//		read_frame_data();
//		//read data from files
//
//		getFrame(mv, pred, cupu, mv_x_extend, mv_y_extend, depth_extend);
//		//usage:
//		//      getFrame(float**, float**, float**,
//		//               float**, float**, float**, )
//		
//		//printres(numframe);
//		//uncomment this to print arrays on screen.
//		saveres(numframe);
//		//uncomment this to save arrays as files.
//	}
//	
//	delarr();
//	//delete arrays and release memory : mv, pred, cupu, mv_x_extend, mv_y_extend, depth_extend
//    return 0;
//}

