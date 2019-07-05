#include "stringtimeline.h"
#include <time.h>

namespace string_util
{
	std::string generate_time_line()
	{
		char buf[128] = { 0 };
		time_t t = time(NULL);
		tm* local = localtime(&t);
		strftime(buf, 64, "%Y-%m-%d-%H-%M-%S", local);
		return std::string(buf);
	}
}