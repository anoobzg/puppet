#include <iostream>
#include <Python.h>

int main(int argc, char* argv[])
{
	std::cout << Py_GetBuildInfo() << std::endl;
	std::cout << _Py_gitversion() << std::endl;
	std::cout << _Py_gitidentifier() << std::endl;
	return EXIT_SUCCESS;
}