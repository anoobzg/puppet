#pragma once
#include <string>
#include <vector>

class CSVWriter
{
public:
	CSVWriter();
	~CSVWriter();

	void Clear();
	void PushHead(const std::string& head);
	void PushData(double value);
	void Output(const std::string& file);
protected:
	std::vector<std::string> m_headers;
	std::vector<double> m_values;
};