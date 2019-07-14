#pragma once
#include <string>

namespace esslam
{
	class DFrame;
	class DirectoryReader
	{
	public:
		DirectoryReader();
		~DirectoryReader();

		void SetParameters(const std::string& directory, const std::string& pattern);
		void Reset();

		bool Load(DFrame& frame);
	protected:
		int m_current_index;
		std::string m_directory;
		std::string m_pattern;
	};
}