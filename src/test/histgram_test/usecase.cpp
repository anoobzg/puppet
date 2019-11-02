#include "usecase.h"

#define SETUP(usecase_data, size, usecase_result, result_size) \
		datas.resize(256);										\
		for (int i = 0; i < 256; ++i)							\
		{														\
			if (i < size) datas.at(i) = usecase_data[i];		\
			else datas.at(i) = 0;								\
		}														\
		expected_result.reserve(result_size);					\
		for (int i = 0; i < result_size; ++i)					\
			if(usecase_result[i] >= 0) expected_result.push_back(usecase_result[i]);

int usecase_data1[19] = {
	100, 99, 88, 88, 89, 92, 91, 91, 91, 92, 92, 92, 92, 93, 93, 93, 93, 90, 90
};

int usecase_result1[6] = {
	0, 5, 13, 14, 15, 16
};

void Case1::Setup(std::vector<int>& datas, std::vector<int>& expected_result)
{
	SETUP(usecase_data1, 19, usecase_result1, 6)
}

int usecase_data2[2] = {
	0, 0
};

int usecase_result2[1] = {
	-1
};

void Case2::Setup(std::vector<int>& datas, std::vector<int>& expected_result)
{
	SETUP(usecase_data2, 2, usecase_result2, 1)
}

int usecase_data3[12] = {
	100, 100, 100, 100, 100, 100,100, 100, 100,100, 100, 100
};

int usecase_result3[12] = {
	0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11
};

void Case3::Setup(std::vector<int>& datas, std::vector<int>& expected_result)
{
	SETUP(usecase_data3, 12, usecase_result3, 12)
}

int usecase_data4[12] = {
	10, 10, 10, 100, 100, 100,10, 10, 10,10, 10, 10
};

int usecase_result4[3] = {
	3, 4, 5
};

void Case4::Setup(std::vector<int>& datas, std::vector<int>& expected_result)
{
	SETUP(usecase_data4, 12, usecase_result4, 3)
}