#pragma once
#include <stdio.h>
#include <string>
#include <vector>
#include <windows.h>
#include <chrono>
#include <iostream>
#include <fstream>
using namespace std;

//计算向量的差
template <typename T>
std::vector <T> diff2vector(std::vector <T> vec1, std::vector <T> vec2)
{
	std::vector <T> temp;
	int dim = vec1.size();
	for (int i=0; i<dim; i++)
	{
		temp.push_back(vec1[i]-vec2[i]);
	}
	return temp;
}

//计算数组的差
template <typename T>
void diff2array(T *arr1, T *arr2, T *arr, const int dim)
{
	for (int i = 0; i<dim; i++)
	{
		arr[i] = arr1[i]- arr2[i];
	}
}

//计算向量的二范数之平方
template <typename T>
T normSum2vector(std::vector <T> vec_temp)
{
	T vec_sum = 0;
	int num_vec = vec_temp.size();
	for (int ivec = 0; ivec<num_vec; ivec++)
	{
		vec_sum = vec_sum + vec_temp[ivec] * vec_temp[ivec];
	}
	return vec_sum;
}

//计算数组的二范数之平方的模板函数
template <typename T>
T normSum2array(T *arr_temp, const int dim_array)
{
	T arr_sum = 0;
	for (int iarr = 0; iarr<dim_array; iarr++)
	{
		arr_sum = arr_sum + arr_temp[iarr] * arr_temp[iarr];
	}
	return arr_sum;
}

//向量降序排列
template <typename T>
void descending_sort(std::vector <T> &vec_temp)
{
	T temp = 0; //临时变量
	for (int i = 0; i < vec_temp.size(); i++)
	{
		for (int j = i + 1; j < vec_temp.size(); j++)
		{
			if (vec_temp[i]<vec_temp[j])   //如果前一个元素小于后一个元素
			{
				temp = vec_temp[i];
				vec_temp[i] = vec_temp[j]; //大的元素到前一个位置
				vec_temp[j] = temp;        //小的元素到后一个位置
			}
		}
	}
}

//向量的最大值
template <typename T>
T vector_maxvalue(std::vector <T> &vec_temp)
{
	T max_temp = -100000; //临时变量
	for (int i = 0; i < vec_temp.size(); i++)
	{
		if (vec_temp[i]>max_temp)
		{
			max_temp = vec_temp[i];
		}
	}
	return max_temp;
}

//向量的最小值
template <typename T>
T vector_minvalue(std::vector <T> &vec_temp)
{
	T min_temp = 1000000; //临时变量
	for (int i = 0; i < vec_temp.size(); i++)
	{
		if (vec_temp[i]<min_temp)
		{
			min_temp = vec_temp[i];
		}
	}
	return min_temp;
}