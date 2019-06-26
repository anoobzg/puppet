#include "convert.h"
#include <Windows.h>

bool unicode_2_utf8(const std::wstring& unicode_string, std::string& utf8_string)
{
	utf8_string = "";
	if (_wcsicmp(unicode_string.c_str(), L"") == 0)
	{
		return false;
	}

	DWORD utf8_string_len = WideCharToMultiByte(CP_ACP, NULL, unicode_string.c_str(), -1, NULL, 0, NULL, FALSE);// WideCharToMultiByteµƒ‘À”√
	if (0 == utf8_string_len)
	{
		return false;
	}
	char *temp_utf8_string = new char[utf8_string_len + 1];
	memset(temp_utf8_string, 0, sizeof(char) * (utf8_string_len + 1));
	if (0 == WideCharToMultiByte(CP_ACP, NULL, unicode_string.c_str(), -1, temp_utf8_string, utf8_string_len, NULL, FALSE))
	{
		delete[] temp_utf8_string;
		temp_utf8_string = NULL;
		return false;
	}

	utf8_string = (std::string)temp_utf8_string;
	delete[] temp_utf8_string;
	temp_utf8_string = NULL;

	return true;
}