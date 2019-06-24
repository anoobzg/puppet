#include "slamparameters.h"
#include <boost\program_options.hpp>
#include <fstream>

using namespace boost::program_options;

SlamParameters::SlamParameters()
{

}

SlamParameters::~SlamParameters()
{

}

void SlamParameters::LoadDefault()
{
	options_description options;

	LoadAllParameters(options, true);
}

void SlamParameters::LoadFromFile(const std::string& file)
{
	LoadDefault();

	std::fstream in(file, std::ios::in);
	if (in.is_open())
	{
		variables_map maps;
		options_description options;

		LoadAllParameters(options, false);
		store(parse_config_file(in, options, true), maps);
		notify(maps);
	}

	in.close();
}

void SlamParameters::LoadAllParameters(boost::program_options::options_description& options, bool inital)
{
	LoadReaderParameters(options, inital);
	LoadICPParameters(options, inital);
}

void SlamParameters::LoadReaderParameters(boost::program_options::options_description& options, bool inital)
{
	if (inital)
	{
		reader_param.directory = "";
		reader_param.pattern = "%d";
		reader_param.time = 0.0f;
		reader_param.profile = 0;
		reader_param.profile_file = "";
	}
	else
	{
		options.add_options()
			("reader_param.directory", value<std::string>(&reader_param.directory))
			("reader_param.pattern", value<std::string>(&reader_param.pattern))
			("reader_param.time", value<float>(&reader_param.time))
			("reader_param.profile", value<int>(&reader_param.profile))
			("reader_param.profile_file", value<std::string>(&reader_param.profile_file))
			;
	}
}

void SlamParameters::LoadICPParameters(boost::program_options::options_description& options, bool inital)
{
	if (inital)
	{
		icp_param.calib_file = "";
		icp_param.profile = 0;
		icp_param.profile_file = "";
	}
	else
	{
		options.add_options()
			("icp_param.calib_file", value<std::string>(&icp_param.calib_file))
			("icp_param.profile", value<int>(&icp_param.profile))
			("icp_param.profile_file", value<std::string>(&icp_param.profile_file))
			;
	}
}