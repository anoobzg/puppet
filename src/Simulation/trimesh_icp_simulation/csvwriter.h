#pragma once
#include <string>
#include <vector>
#include "timestamp.h"

class CSVWriter
{
public:
	CSVWriter();
	~CSVWriter();

	void Clear();
	void PushHead(const std::string& head);
	void PushData(double value);
	void TickStart();
	void TickEnd();
	void Output(const std::string& file);
protected:
	std::vector<std::string> m_headers;
	std::vector<double> m_values;

	trimesh::timestamp m_start_time;
};