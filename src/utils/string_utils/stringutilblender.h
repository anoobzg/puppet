#pragma once
#include "stringutilexporter.h"

namespace string_util
{
	STRING_UTIL_API void preprocess_include(char* data, int len, int& newlen);
	STRING_UTIL_API char* read_file_data(const char* filename, int& len);
}