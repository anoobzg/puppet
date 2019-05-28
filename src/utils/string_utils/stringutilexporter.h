#pragma once

#if defined( STRING_UTIL_LIBRARY_STATIC )
#define STRING_UTIL_API
#elif defined( STRING_UTIL_DLL )
#define STRING_UTIL_API __declspec(dllexport)
#else
#define STRING_UTIL_API __declspec(dllimport)
#endif