#include <iostream>
#include "tester.h"

#include <assert.h>

void TestOneCase(UseCase& use_case, Tester& tester)
{
	tester.SetCase(use_case);

	bool result = tester.Test();
	assert(result);
	if (!result) std::cout << "Test Error." << std::endl;
}

int main(int argc, const char* argv[])
{
	Tester tester;

	Case1 case1;
	Case2 case2;
	Case3 case3;

	TestOneCase(case1, tester);
	TestOneCase(case2, tester);
	TestOneCase(case3, tester);
	return EXIT_SUCCESS;
}