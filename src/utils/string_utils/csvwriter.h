#pragma once
#include <string>
#include <vector>
#include "stringtimestamp.h"
#include "stringutilexporter.h"

namespace string_util
{
	class STRING_UTIL_API CSVWriter
	{
	public:
		CSVWriter();
		~CSVWriter();

		void Clear();
		void PushHead(const std::string& head);
		void PushData(double value);
		void TickStart();
		void TickEnd();
		void Start(timestamp* time = 0);
		void Tick();
		void Output(const std::string& file);
	protected:
		std::vector<std::string> m_headers;
		std::vector<double> m_values;

		timestamp m_start_time;
		timestamp m_flag_time;

		int m_index;
	};

}