#pragma once
#include <string>
#include <vector>
#include <fstream>

struct TimeSeg
{
	float start;
	float end;
};

class StampObject
{
public:
	StampObject(const char* name);
	~StampObject();

	void Load(std::fstream& in);

	std::string m_name;
	std::vector<TimeSeg> m_times;
};