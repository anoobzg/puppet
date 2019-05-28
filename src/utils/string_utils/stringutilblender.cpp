#include "stringutilblender.h"
#include <string>
#include <iostream>

namespace string_util
{
	void preprocess_include(char* data, int len, int& newlen)
	{
		/* note: len + 1, last character is a dummy to prevent
		 * comparisons using uninitialized memory */
		char* temp = new char[len + 1];
		temp[len] = ' ';

		memcpy(temp, data, len);

		//for (int i = 0; i < len; ++i)
		//	std::cout << (*(temp + i));
		//std::cout << std::endl;

		/* remove all c++ comments */
		/* replace all enters/tabs/etc with spaces */
		char* cp = temp;
		int a = len;
		int comment = 0;
		while (a--) 
		{
			if (cp[0] == '/' && cp[1] == '/')
			{
				comment = 1;
			}
			else if (*cp < 32)
			{
				comment = 0;
			}
			if (comment || *cp < 32 || *cp > 128) *cp = 32;
			cp++;
		}

		//for (int i = 0; i < len; ++i)
		//	std::cout << (*(temp + i));
		//std::cout << std::endl;

		/* data from temp copy to maindata, remove comments and double spaces */
		cp = temp;
		char* md = data;
		newlen = 0;
		comment = 0;
		a = len;
		while (a--)
		{
			if (cp[0] == '/' && cp[1] == '*')
			{
				comment = 1;
				cp[0] = cp[1] = 32;
			}
			if (cp[0] == '*' && cp[1] == '/')
			{
				comment = 0;
				cp[0] = cp[1] = 32;
			}

			/* do not copy when: */
			if (comment)
			{
				/* pass */
			}
			else if (cp[0] == ' ' && cp[1] == ' ')
			{
				/* pass */
			}
			else if (cp[-1] == '*' && cp[0] == ' ') {
				/* pointers with a space */
			}	/* skip special keywords */
			else if (strncmp("DNA_DEPRECATED", cp, 14) == 0)
			{
				/* single values are skipped already, so decrement 1 less */
				a -= 13;
				cp += 13;
			}
			else
			{
				md[0] = cp[0];
				md++;
				newlen++;
			}
			cp++;
		}

		//for (int i = 0; i < newlen; ++i)
		//	std::cout << (*(data + i));
		//std::cout << std::endl;

		delete[]temp;
	}

	char* read_file_data(const char* filename, int& len)
	{
#ifdef WIN32
		FILE* fp = fopen(filename, "rb");
#else
		FILE* fp = fopen(filename, "r");
#endif
		if (!fp) {
			len = -1;
			return NULL;
		}

		fseek(fp, 0L, SEEK_END);
		len = ftell(fp);
		fseek(fp, 0L, SEEK_SET);

		if (len == -1) {
			fclose(fp);
			return NULL;
		}

		char* data = new char[len];
		if (!data) {
			len = -1;
			fclose(fp);
			return NULL;
		}

		if (fread(data, len, 1, fp) != 1) {
			len = -1;
			delete []data;
			fclose(fp);
			return NULL;
		}

		fclose(fp);
		return data;
	}
}