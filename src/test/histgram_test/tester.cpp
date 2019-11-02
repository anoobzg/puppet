#include "tester.h"
#include <iostream>

#include "search_localmaxima.h"
#include <algorithm>
Tester::Tester()
{

}

Tester::~Tester()
{

}

void Tester::SetCase(UseCase& use_case)
{
	m_datas.clear();
	m_expected_values.clear();
	use_case.Setup(m_datas, m_expected_values);
}

bool Tester::Test()
{
	if (m_datas.size() == 0)
	{
		std::cout << "Histogram datas size error." << std::endl;
		return false;
	}

	std::vector<int> results;
	search_localmaxima_in_histogram(m_datas, results);
	std::sort(results.begin(), results.end());

	size_t size = results.size();
	if (size != m_expected_values.size())
	{
		std::cout << "Unexpected result size." << std::endl;
		return false;
	}

	bool expected = true;
	for (size_t i = 0; i < size; ++i)
	{
		if (m_expected_values.at(i) != results.at(i))
		{
			expected = false;
			break;
		}
	}
	return expected;
}