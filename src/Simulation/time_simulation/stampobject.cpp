#include "stampobject.h"

StampObject::StampObject(const char* name)
	:m_name(name)
{

}

StampObject::~StampObject()
{

}

void StampObject::Load(std::fstream& in)
{
	m_times.reserve(1000);
	char line[256];

	int first_line = 0;
	while (!in.eof())
	{
		in.getline(line, 256);

		if (first_line >= 1)
		{
			std::string sline(line);
			size_t s = sline.find_first_of(',');
			size_t e = sline.find_last_of(',');
			if (s != std::string::npos && e != std::string::npos)
			{
				std::string start_time = std::string(sline.begin() + s + 1, sline.begin() + e);
				std::string end_time = std::string(sline.begin() + e + 1, sline.end());
				TimeSeg seg;
				seg.start = (float)atof(start_time.c_str());
				seg.end = (float)atof(end_time.c_str());
				m_times.push_back(seg);
			}
		}
		++first_line;
	}
}