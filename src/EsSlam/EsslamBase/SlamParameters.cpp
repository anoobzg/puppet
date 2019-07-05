#include "SlamParameters.h"
#include <boost\program_options.hpp>
#include <fstream>

using namespace boost::program_options;

namespace esslam
{

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
		LoadOctreeParameters(options, inital);
		LoadDebugParameters(options, inital);
	}

	void SlamParameters::LoadReaderParameters(boost::program_options::options_description& options, bool inital)
	{
		if (inital)
		{
			reader_param.directory = "";
			reader_param.pattern = "%d";
			reader_param.time = 0.0f;
		}
		else
		{
			options.add_options()
				("reader_param.directory", value<std::string>(&reader_param.directory))
				("reader_param.pattern", value<std::string>(&reader_param.pattern))
				("reader_param.time", value<float>(&reader_param.time))
				;
		}
	}

	void SlamParameters::LoadICPParameters(boost::program_options::options_description& options, bool inital)
	{
		if (inital)
		{
			icp_param.calib_file = "";
			icp_param.use_fast = 0;
		}
		else
		{
			options.add_options()
				("icp_param.calib_file", value<std::string>(&icp_param.calib_file))
				("icp_param.use_fast", value<int>(&icp_param.use_fast))
				;
		}
	}

	void SlamParameters::LoadDebugParameters(boost::program_options::options_description& options, bool inital)
	{
		if (inital)
		{
			debug_param.debug = 0;
			debug_param.directory = "";
			debug_param.out_directory = "";
			debug_param.save_fm_failed = 0;
		}
		else
		{
			options.add_options()
				("debug_param.debug", value<int>(&debug_param.debug))
				("debug_param.directory", value<std::string>(&debug_param.directory))
				("debug_param.out_directory", value<std::string>(&debug_param.out_directory))
				("debug_param.save_fm_failed", value<int>(&debug_param.save_fm_failed))
				;
		}
	}

	void SlamParameters::LoadOctreeParameters(boost::program_options::options_description& options, bool inital)
	{
		if (inital)
		{
			octree_param.cell_depth = 5;
		}
		else
		{
			options.add_options()
				("octree_param.cell_depth", value<int>(&octree_param.cell_depth))
				;
		}
	}

}