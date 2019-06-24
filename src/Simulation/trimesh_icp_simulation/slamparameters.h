#pragma once
#include <string>

struct ReaderParameters
{
	std::string directory;
	std::string pattern;
	float time;
	int profile;
	std::string profile_file;
};

struct ICPParamters
{
	std::string calib_file;
	int profile;
	std::string profile_file;
};

namespace boost
{
	namespace program_options
	{
		class options_description;
	}
}
class SlamParameters
{
public:
	SlamParameters();
	~SlamParameters();

	void LoadDefault();
	void LoadFromFile(const std::string& file);

	ReaderParameters reader_param;
	ICPParamters icp_param;
protected:
	void LoadAllParameters(boost::program_options::options_description& options, bool inital);
	void LoadReaderParameters(boost::program_options::options_description& options, bool inital);
	void LoadICPParameters(boost::program_options::options_description& options, bool inital);
};