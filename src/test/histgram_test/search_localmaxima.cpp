#include "search_localmaxima.h"

enum CompareResult
{
	ecr_equal,
	ecr_less,
	ecr_larger
};

enum ValueType
{
	evt_unsure,
	evt_maxima,
	evt_common
};


void search_localmaxima_in_histogram(const std::vector<int>& histogram_datas, std::vector<int>& indices)
{
	int size = (int)histogram_datas.size();
	if (size == 0) return;
	
#define SET(x, t) 

	std::vector<ValueType> indicators(size, evt_unsure);
	CompareResult left_result = ecr_equal;
	CompareResult right_result = ecr_equal;
	int left_index = -1;
	int right_index = -1;
	int left_data = -1;
	int right_data = -1;
	for (int i = 0; i < size; ++i)
	{
		if (indicators.at(i) == evt_unsure)
		{
			int data = histogram_datas.at(i);
			left_result = ecr_equal;
			right_result = ecr_equal;

			left_index = i - 1;
			right_index = i + 1;
			while (left_index >= 0)  //search left
			{
				left_data = histogram_datas.at(left_index);
				if (left_data > data)
				{
					left_result = ecr_larger;
					break;
				}
				else if (left_data < data)
				{
					left_result = ecr_less;
					break;
				}

				--left_index;
			}
			while (right_index < size)  //search right
			{
				right_data = histogram_datas.at(right_index);
				if (right_data > data)
				{
					right_result = ecr_larger;
					break;
				}
				else if (right_data < data)
				{
					right_result = ecr_less;
					break;
				}

				++right_index;
			}

			if (((left_result == ecr_less) && (right_result != ecr_larger))
				|| ((right_result == ecr_less) && (left_result != ecr_larger)))
			{// maxima
				for (int i = left_index + 1; i < right_index; ++i)
					indicators.at(i) = evt_maxima;
				if (left_index >= 0) indicators.at(left_index) = evt_common;
				if (right_index < size) indicators.at(right_index) = evt_common;
			}
			else
			{
				for (int i = left_index + 1; i < right_index; ++i)
					indicators.at(i) = evt_common;
			}
		}
	}

	indices.reserve(size);
	for (int i = 0; i < size; ++i)
	{
		if (indicators.at(i) == evt_maxima)
			indices.push_back(i);
	}
}