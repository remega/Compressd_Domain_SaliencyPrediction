// code_mat_c.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include<iostream>
#include<fstream>
#include<string.h>
#include<cmath>
#include <sstream>
#include <string>
#include "func.h"
#include "TDecCu.h"
using namespace std;

#define width VIDEO_WIDTH
#define height VIDEO_HIGHT
#define numFrames FRAMENUM
#define stopNum 999

int num_patch = int(height / 64.0 + 0.5) * int(width / 64.0 + 0.5),
		num_col = int(width / 64.0+0.5),
		imag_num = numFrames;

int m[64][64], m_x[64][64], m_y[64][64];
int m_nonpu[64][64];
float mv_x_extend[(VIDEO_HIGHT/64+1)*64][(VIDEO_WIDTH/64+1)*64];
float mv_y_extend[(VIDEO_HIGHT/64+1)*64][(VIDEO_WIDTH/64+1)*64];
float depth_extend[(VIDEO_HIGHT/64+1)*64][(VIDEO_WIDTH/64+1)*64];


void printm()
{
	cout << "m" << endl;
	for (int x = 0; x<64; x++)
	{
		for (int y = 0; y<64; y++)
		{
			cout << m[x][y] << ' ';
		}
		cout << endl;
	}
}
void checkm()
{
	int key = 0;
	for (int x = 0; x<64; x++)
		for (int y = 0; y<64; y++)
			if (m[x][y] > 10000) { key = 1; break; }
	if (key == 1)
	{
		printm();
		system("pause");
	}
}
void batch_64(float *arr, int num)
{
	int brr[1000], flag = 0, arr_l = num;
	for (int i = 0; i<num; i++)
	{
		if (arr[i]<99) brr[i] = 0;
		else brr[i] = arr[i];
		if (brr[i] == 99) flag++;
	}

	if (flag == 0)
	{
		for (int x = 0; x<64; x++)
			for (int y = 0; y<64; y++)
				m[x][y] = 1;
	}
	else if (flag == 1)
	{
		for (int x = 0; x<32; x++)
			for (int y = 0; y<32; y++)
				m[x][y] = 1;
		for (int x = 0; x<32; x++)
			for (int y = 32; y<64; y++)
				m[x][y] = 2;
		for (int x = 32; x<64; x++)
			for (int y = 0; y<32; y++)
				m[x][y] = 3;
		for (int x = 32; x<64; x++)
			for (int y = 32; y<64; y++)
				m[x][y] = 4;
	}
	else {
		//----------------32x32
		int addr_1[1000], addr_1_l;
		int addr_1_new[1000], addr_1_new_l;
		addr_1_l = find990000(brr, num, addr_1);

		while (addr_1_l>5)
		{
			addr_1_new_l = find99xxxx(addr_1, addr_1_l, addr_1_new);
			for (int i = 0; i<addr_1_new_l; i++)
				addr_1[i] = addr_1_new[i];
			addr_1_l = addr_1_new_l;
		}

		//处理0 
		int addr0[1000], addr0_l = 0;
		for (int i = 0; i<addr_1_l; i++)
		{
			if (addr_1[i] == 0) {
				addr0[addr0_l] = i;
				addr0_l++;
			}
		}
		for (int i = 0; i<addr0_l; i++)
		{
			int n = addr0[i], layer = 1;
			int row_start, row_end, col_start, col_end;
			row_start = 32 / layer * int((n) / 3);//1 + 
			row_end = 32 / layer + 32 / layer * int((n) / 3);
			col_start = 32 / layer * ((n + 1) % 2);//1 +  + 1
			col_end = 32 / layer + 32 / layer * ((n + 1) % 2);// + 1

			int seq = 1, jj = 0;
			while (1)
			{
				int seq_prior = seq_cal(addr_1[jj]);
				seq = seq + seq_prior;
				jj = jj + 1;
				if (jj > addr0[i] - 1)
					break;
			}
			for (int x = row_start; x < row_end; x++)
				for (int y = col_start; y < col_end; y++)
					m[x][y] = seq;
			//printm();
		}

		int L_32[1000], L_32_l = 0;
		pre_pro(addr_1, addr_1_l, L_32, &L_32_l);

		//处理非0 
		int addrn0[1000], addrn0_l = 0;
		for (int i = 0; i<addr_1_l; i++)
		{
			if (addr_1[i] != 0) {
				addrn0[addrn0_l] = i;
				addrn0_l++;
			}
		}
		for (int i = 1; i<addrn0_l; i++)
		{
			int seq_init = 0, s = 0;
			while (1)
			{
				seq_init = seq_init + seq_cal(addr_1[s]);//seq_init记录该子块之前已分块的数目
				s = s + 1;
				if (s > addrn0[i] - 1)
					break;
			}
			int n = addrn0[i], layer = 1;
			int row_start, row_end, col_start, col_end;
			row_start = 32 / layer * int((n) / 3);//1 + 
			row_end = 32 / layer + 32 / layer * int((n) / 3);
			col_start = 32 / layer * ((n + 1) % 2);//1 +  + 1
			col_end = 32 / layer + 32 / layer * ((n + 1) % 2);// + 1

			int l_start, l_end;
			inf_32batch(L_32, i - 1, &l_start, &l_end);
			int inf_32[1000], inf_32_l = 0;
			for (int i = l_start; i<l_end; i++)
			{
				inf_32[inf_32_l] = brr[i];
				inf_32_l++;
			}
			int addr_2[1000], addr_2_l;
			int addr_2_new[1000], addr_2_new_l;
			addr_2_l = find990000(inf_32, inf_32_l, addr_2);
			while (addr_2_l>5)
			{
				addr_2_new_l = find99xxxx(addr_2, addr_2_l, addr_2_new);
				for (int i = 0; i<addr_2_new_l; i++)
					addr_2[i] = addr_2_new[i];
				addr_2_l = addr_2_new_l;
			}

			//----------------16x16
			//处理0 
			int addr_16_0[1000], addr_16_0_l = 0;
			int m_lay_1[32][32];

			for (int x = 0; x<32; x++)
				for (int y = 0; y<32; y++)
					m_lay_1[x][y] = 0;

			for (int i = 0; i<addr_2_l; i++)
			{
				if (addr_2[i] == 0) {
					addr_16_0[addr_16_0_l] = i;
					addr_16_0_l++;
				}
			}
			for (int i = 0; i<addr_16_0_l; i++)
			{
				int n = addr_16_0[i], layer = 2;
				int row_start, row_end, col_start, col_end;
				row_start = 32 / layer * int((n) / 3);//1 + 
				row_end = 32 / layer + 32 / layer * int((n) / 3);
				col_start = 32 / layer * ((n + 1) % 2);//1 +  + 1
				col_end = 32 / layer + 32 / layer * ((n + 1) % 2);// + 1

				int seq = seq_init + 1;
				int jj = 0;
				while (1)
				{
					int seq_prior = seq_cal(addr_2[jj]);
					seq = seq + seq_prior;
					jj = jj + 1;
					if (jj > addr_16_0[i] - 1)
						break;
				}
				for (int x = row_start; x<row_end; x++)
					for (int y = col_start; y<col_end; y++)
						m_lay_1[x][y] = seq;
			}

			//处理非0 
			int addr_16_other[1000], addr_16_other_l = 0;
			int m_lay_2[16][16];
			for (int x = 0; x<16; x++)
				for (int y = 0; y<16; y++)
					m_lay_2[x][y] = 0;
			for (int i = 0; i<addr_2_l; i++)
			{
				if (addr_2[i] == -1) {
					addr_16_other[addr_16_other_l] = i;
					addr_16_other_l++;
				}
			}
			for (int i = 0; i<addr_16_other_l; i++)
			{
				int n = addr_16_other[i], layer = 2;
				int row_start, row_end, col_start, col_end;
				row_start = 32 / layer * int((n) / 3);//1 + 
				row_end = 32 / layer + 32 / layer * int((n) / 3);
				col_start = 32 / layer * ((n + 1) % 2);//1 +  + 1
				col_end = 32 / layer + 32 / layer * ((n + 1) % 2);// + 1

				int seq = seq_init + 1, jj = 0;
				while (1)
				{
					int seq_prior = seq_cal(addr_2[jj]);
					seq = seq + seq_prior;
					jj = jj + 1;
					if (jj > addr_16_other[i] - 1)
						break;
				}
				for (int x = 0; x<8; x++)
					for (int y = 0; y<8; y++)
						m_lay_2[x][y] = seq;
				for (int x = 0; x<8; x++)
					for (int y = 8; y<16; y++)
						m_lay_2[x][y] = seq + 1;
				for (int x = 8; x<16; x++)
					for (int y = 0; y<8; y++)
						m_lay_2[x][y] = seq + 2;
				for (int x = 8; x<16; x++)
					for (int y = 8; y<16; y++)
						m_lay_2[x][y] = seq + 3;

				for (int x = row_start; x<row_end; x++)
					for (int y = col_start; y<col_end; y++)
						m_lay_1[x][y] = m_lay_2[x - row_start][y - col_start];
			}

			for (int x = row_start; x<row_end; x++)
				for (int y = col_start; y<col_end; y++)
					m[x][y] = m_lay_1[x - row_start][y - col_start];
		}
	}
	for (int x = 0; x<64; x++)
		for (int y = 0; y<64; y++)
		{
			m_nonpu[x][y] = m[x][y];
		}
	
	//---------------pu
	int seq[1000], seq_l = 0, pu_number[1000];
	pu(arr, arr_l, seq, &seq_l, pu_number);

	int addr_15[1000], info_15[1000], addr_15_l = 0, seq_l_tmp = 0;
	for (int i = 0; i<seq_l; i++)
	{
		if (pu_number[i] == 15)
		{
			addr_15[addr_15_l] = i;
			info_15[addr_15_l] = seq[i];
			addr_15_l++;
		}
		else {
			seq[seq_l_tmp] = seq[i];
			pu_number[seq_l_tmp] = pu_number[i];
			seq_l_tmp++;
		}
	}
	seq_l = seq_l_tmp;
	int step = 0, number_seq = seq[0];
	for (int ii = 0; ii< seq_l; ii++)
	{
		if (ii == 0)
			number_seq = seq[0];
		else
		{
			int a = 0;
			int jj = 0;
			while (1)
			{
				a = a + len_pu(pu_number[jj]);
				jj = jj + 1;
				if (jj >= ii)
					break;
			}
			number_seq = a + seq[ii];
		}
		int inf_pu[64 * 64], inf_pu_l = 0, in;
		for (int y = 0; y<64; y++)
		{
			for (int x = 0; x<64; x++)
			{
				if (m[x][y] == number_seq) {
					inf_pu[inf_pu_l] = x + y * 64;
					inf_pu_l++;
				}
				else if (m[x][y]>number_seq)
					m[x][y] = m[x][y] + len_pu(pu_number[ii]);
			}
		}
		int radii = sqrt((float)inf_pu_l), inf_pu_re[64][64];//re
		for (int x = 0; x<radii; x++)
		{
			for (int y = 0; y<radii; y++)
			{
				inf_pu_re[x][y] = inf_pu[x + y*radii];
			}
		}
		if (pu_number[ii] == 2)
			for (int i = inf_pu_l / 2; i<inf_pu_l; i++)
				m[inf_pu[i] % 64][inf_pu[i] / 64] = number_seq + 1;
		else if (pu_number[ii] == 6)
			for (int i = inf_pu_l / 4; i<inf_pu_l; i++)
				m[inf_pu[i] % 64][inf_pu[i] / 64] = number_seq + 1;
		else if (pu_number[ii] == 7)
			for (int i = inf_pu_l / 4 * 3; i<inf_pu_l; i++)
				m[inf_pu[i] % 64][inf_pu[i] / 64] = number_seq + 1;
		else if (pu_number[ii] == 3)
		{
			for (int x = 0; x<radii / 2; x++)
				for (int y = radii / 2; y<radii; y++)
				{
					m[inf_pu_re[x][y] % 64][inf_pu_re[x][y] / 64] = number_seq + 1;
				}
			for (int x = radii / 2; x<radii; x++)
				for (int y = 0; y<radii / 2; y++)
				{
					m[inf_pu_re[x][y] % 64][inf_pu_re[x][y] / 64] = number_seq + 2;
				}
			for (int x = radii / 2; x<radii; x++)
				for (int y = radii / 2; y<radii; y++)
				{
					m[inf_pu_re[x][y] % 64][inf_pu_re[x][y] / 64] = number_seq + 3;
				}
		}
		else if (pu_number[ii] == 1)
			for (int x = radii / 2; x<radii; x++)
				for (int y = 0; y<radii; y++)
				{
					m[inf_pu_re[x][y] % 64][inf_pu_re[x][y] / 64] = number_seq + 1;
				}
		else if (pu_number[ii] == 4)
			for (int x = radii / 4; x<radii; x++)
				for (int y = 0; y<radii; y++)
				{
					m[inf_pu_re[x][y] % 64][inf_pu_re[x][y] / 64] = number_seq + 1;
				}
		else if (pu_number[ii] == 5)
			for (int x = radii / 4 * 3; x<radii; x++)
				for (int y = 0; y<radii; y++)
				{
					m[inf_pu_re[x][y] % 64][inf_pu_re[x][y] / 64] = number_seq + 1;
				}
	}
	if (addr_15_l != 0)
	{
		int addr_15_new[1000], info_15_l, m_single[64 * 64], m_single_l = 0;
		for (int x = 0; x<64; x++)
			for (int y = 0; y<64; y++)
				if (m_single_l<m[x][y]) m_single_l = m[x][y];
		for (int i = 0; i<m_single_l; i++)
			m_single[i] = i + 1;
		for (int ii = 0; ii<addr_15_l; ii++)
		{
			int temp, keyt = 0;

			for (int x = 0; x<64; x++)
			{
				for (int y = 0; y<64; y++)
				{
					if (m_nonpu[x][y] == info_15[ii])
					{
						if (keyt == 0)
						{
							keyt = 1;
							temp = m[x][y];
						}
						m[x][y] = m[x][y] * (-1);
						//printm();
					}
				}
			}
			m_single[temp - 1] = -1 * temp;
		}
		int m_temp[64 * 64], m_temp1[64 * 64];
		for (int x = 0; x<m_single_l; x++)
		{
			if (m_single[x]>0) m_temp[x] = m_single[x];
			else m_temp[x] = 0;
			m_temp1[x] = m_temp[x];
		}
		for (int ii = 1; ii<m_single_l; ii++)
		{
			int num_0 = 0;
			for (int x = 0; x < ii; x++)
			{
				if (m_temp1[x] == 0)
					num_0++;
			}
			m_temp[ii] = m_temp[ii] - num_0;
		}
		int addr_dif[1000], addr_dif_l = 0;
		for (int x = 0; x<m_single_l; x++)
		{
			if (m_temp[x] <= 0) m_temp[x] = m_single[x];
			if (m_temp[x] != m_single[x]) {
				addr_dif[addr_dif_l] = x;
				addr_dif_l++;
			}
		}
		if (addr_dif_l>0)
		{
			for (int ii = 0; ii<addr_dif_l; ii++)
				for (int x = 0; x<64; x++)
					for (int y = 0; y<64; y++)
						if (m[x][y] == m_single[addr_dif[ii]])
							m[x][y] = m_temp[addr_dif[ii]];
		}
	}
}
void cupu_f(float *cupu)
{
	int num = 0;
	while (cupu[num] != stopNum) num++;
	batch_64(cupu, num);
	for (int x = 0; x<64; x++)
		for (int y = 0; y<64; y++)
		{
			m_x[x][y] = m[x][y];
			m_y[x][y] = m[x][y];
		}
}
void my_data(float *mv,float *pred,int *mv_final, int *mv_x, int *mv_y, int *x)
{
	int nummv = 0, numpred = 0;
	while (mv[nummv] != stopNum) nummv++;
	while (pred[numpred] != stopNum) numpred++;
	int addr_intra[1000], total_intra = 0;
	for (int i = 0; i<numpred; i++)
	{
		if (pred[i] == 2) {
			addr_intra[total_intra] = i;
			total_intra++;
		}
	}
	int mv_extend_l = nummv + total_intra * 3, mv_extend[1000];
	for (int i = 0; i < 1000; i++)
	{
		mv_extend[i] = 0;
	}
	if (total_intra == 0)
	{
		for (int i = 0; i<mv_extend_l; i++)
			mv_extend[i] = mv[i];
	}
	else if (total_intra == 1)
	{
		for (int i = 0; i<(addr_intra[0]) * 4; i++)
			mv_extend[i] = mv[i];
		int t0 = 3, mvc = 0;
		for (int i = (addr_intra[0]) * 4; i<(addr_intra[0] + 1) * 4; i++)
		{
			if (t0 != 0)
			{
				t0--;
				mv_extend[i] = 0;
			}
			else {
				mv_extend[i] = mv[addr_intra[0]];
				//mvc++;
			}
		}
		mvc = (addr_intra[0]) * 4 + 1;
		for (int i = (addr_intra[0] + 1) * 4; i<mv_extend_l; i++)
		{
			mv_extend[i] = mv[mvc];
			mvc++;
		}
	}
	else {
		int mv_length[1000];
		for (int ii = 0; ii<total_intra; ii++) {
			int num_inter = 0, num_intra = 0;
			for (int i = 0; i<addr_intra[ii]; i++)
			{
				if (pred[i] == 1) num_inter++;
				else if (pred[i] == 2) num_intra++;
			}
			mv_length[ii] = 4 * num_inter + num_intra;
		}
		int start_point, end_point;
		for (int ii = 0; ii<total_intra; ii++) {
			int t0 = 3, mvc = mv_length[ii], len_temp;
			for (int i = mv_length[ii] + ii * 3; i<mv_length[ii] + ii * 3 + 4; i++)
			{
				if (t0 != 0)
				{
					t0--;
					mv_extend[i] = 0;
				}
				else {
					mv_extend[i] = mv[mvc];
					mvc++;
				}
			}
			if (ii == 0) {
				for (int i = 0; i<mv_length[ii]; i++)
					mv_extend[i] = mv[i];
				len_temp = mv_length[ii];
			}
			else {
				start_point = len_temp + 4;
				end_point = start_point + mv_length[ii] - mv_length[ii - 1] - 1;
				len_temp = end_point;
				int mcv = mv_length[ii - 1] + 1;
				for (int i = start_point; i<end_point; i++)
				{
					mv_extend[i] = mv[mcv];
					mcv++;
				}
			}
		}
		if (mv_length[total_intra - 1] + 1 != numpred)
		{
			int mvc = mv_length[total_intra - 1] + 1;
			for (int i = end_point + 4; i<mv_extend_l; i++)
			{
				mv_extend[i] = mv[mvc];
				mvc++;
			}
		}
	}
	*x = mv_extend_l / 4;
	for (int j = 0; j<*x; j++)
	{
		mv_final[j] = mv_extend[2 + j * 4] ^ 2 + mv_extend[3 + j * 4] ^ 2;
		mv_x[j] = mv_extend[2 + j * 4];
		mv_y[j] = mv_extend[3 + j * 4];
	}


}

void getFrame(float (*mv_arr)[1000],float (*pred_arr)[1000],float (*cupu_arr)[1000],float (*mv_x_extend2)[VIDEO_WIDTH/4],float (*mv_y_extend2)[VIDEO_WIDTH/4],float (*depth_extend2)[VIDEO_WIDTH/4])
{	
	for (int line = 0; line < num_patch; line++)
	{
		cupu_f(cupu_arr[line]);
		int mv_inf[1000], mv_x_tmp[1000], mv_y_tmp[1000], mv_inf_l;
		//int width = 832,height = 480,numFrames=500;

		int row_64_start, row_64_end, col_64_start, col_64_end;
		row_64_start = 64 * ((((line) % num_patch) / num_col));
		row_64_end = 64 * (((line) % num_patch) / num_col + 1);
		col_64_start = 64 * (((line) % num_patch) % num_col);
		col_64_end = 64 * (srem(((line+1)%num_patch), num_col));
		for (int i = 0; i < 1000; i++)
		{
			mv_inf[i] = 0;
			mv_x_tmp[i] = 0;
			mv_y_tmp[i] = 0;
		}
		my_data(mv_arr[line], pred_arr[line], mv_inf, mv_x_tmp, mv_y_tmp, &mv_inf_l);
		for (int k = 0; k<mv_inf_l; k++)
		{
			for (int i = 0; i<64; i++)
				for (int j = 0; j<64; j++)
				{
					if (m_x[i][j] == k + 1) m_x[i][j] = mv_x_tmp[k];
					if (m_y[i][j] == k + 1) m_y[i][j] = mv_y_tmp[k];
				}
		}
		for (int i = row_64_start; i<row_64_end; i++)
			for (int j = col_64_start; j<col_64_end; j++)
			{
				mv_x_extend[i][j] = m_x[i - row_64_start][j - col_64_start];
				mv_y_extend[i][j] = m_y[i - row_64_start][j - col_64_start];
			}
		int max_64batch = m_nonpu[0][0], m_nonpu_temp[64][64];
		for (int x = 0; x < 64; x++)
			for (int y = 0; y < 64; y++)
			{
				if (max_64batch < m_nonpu[x][y])
					{
						max_64batch = m_nonpu[x][y];
				}
				
				m_nonpu_temp[x][y] = m_nonpu[x][y];
			}

		for (int ii = 1; ii <= max_64batch; ii++)
		{
			int batch_len = 0, depth_batch;
			for (int x = 0; x < 64; x++)
				for (int y = 0; y < 64; y++)
				{
					if (m_nonpu_temp[x][y] == ii) batch_len++;
				}
			depth_batch = depth_cal(sqrt((float)batch_len));
			for (int x = 0; x < 64; x++)
				for (int y = 0; y < 64; y++)
					if (m_nonpu_temp[x][y] == ii) m_nonpu[x][y] = depth_batch;
		}

		for (int i = row_64_start; i<row_64_end; i++)
			for (int j = col_64_start; j<col_64_end; j++)
				depth_extend[i][j] = m_nonpu[i - row_64_start][j - col_64_start];
	}
	for (int i = 0; i < height / 4; i++)
	{
		for (int j = 0; j < width / 4; j++)
		{
			mv_x_extend2[i][j] = mv_x_extend[i * 4][j * 4];
			mv_y_extend2[i][j] = mv_y_extend[i * 4][j * 4];
			depth_extend2[i][j] = depth_extend[i * 4][j * 4];
		}
	}
	//delete
}