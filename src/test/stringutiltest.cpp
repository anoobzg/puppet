#include <iostream>
#include "stringutilblender.h"

int main(int argc, char* argv[])
{
	if (argc < 2)
		return EXIT_FAILURE;

	int filelen = -1;
	char* data = string_util::read_file_data(argv[1], filelen);
	if (filelen == -1 || !data)
	{
		fprintf(stderr, "Can't read file %s\n", argv[1]);
		return EXIT_FAILURE;
	}

	int newlen = -1;
	string_util::preprocess_include(data, filelen, newlen);
	std::string tmp(data);
	delete[]data;
	return EXIT_SUCCESS;
}