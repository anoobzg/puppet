#include "csvwriter.h"
#include <iostream>
#include <fstream>

namespace string_util
{

	CSVWriter::CSVWriter()
	{

	}

	CSVWriter::~CSVWriter()
	{

	}

	void CSVWriter::Clear()
	{
		m_headers.clear();
		m_values.clear();
	}

	void CSVWriter::PushHead(const std::string& head)
	{
		m_headers.push_back(head);
	}

	void CSVWriter::PushData(double value)
	{
		m_values.push_back(value);
	}

	void CSVWriter::TickStart()
	{
		m_start_time = now();
	}

	void CSVWriter::TickEnd()
	{
		float t = now() - m_start_time;
		m_values.push_back((double)t);
	}

	void CSVWriter::Output(const std::string& file)
	{
		size_t head_size = m_headers.size();
		size_t value_size = m_values.size();
		if ((head_size == 0) || (value_size == 0) || (value_size % head_size != 0))
		{
			std::cout << "head and value size error." << std::endl;
			return;
		}

		std::fstream out(file.c_str(), std::ios::out);
		if (!out.is_open())
		{
			std::cout << "open file " << file.c_str() << " error." << std::endl;
			out.close();
			return;
		}

		auto add_seg = [&out, &head_size](size_t i) {
			if ((i + 1) % head_size == 0)
				out << std::endl;
			else out << ",";
		};

		for (size_t i = 0; i < head_size; ++i)
		{
			out << m_headers.at(i).c_str();
			add_seg(i);
		}
		for (size_t i = 0; i < value_size; ++i)
		{
			out << m_values.at(i);
			add_seg(i);
		}
		out.close();
	}
}