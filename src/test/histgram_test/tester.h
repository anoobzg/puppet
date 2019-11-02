#pragma once
#include "usecase.h"

class Tester
{
public:
	Tester();
	~Tester();

	void SetCase(UseCase& use_case);
	bool Test();
private:
	std::vector<int> m_datas;
	std::vector<int> m_expected_values;
};