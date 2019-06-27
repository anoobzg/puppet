#pragma once
#include <string>

struct ReaderParameters
{
	std::string directory;
	std::string pattern;
	float time;
};

struct ICPParamters
{
	std::string calib_file;
};

struct DebugParameters
{
	int debug;
	std::string directory;
	std::string out_directory;
};

struct OctreeParameters
{
	int cell_depth;
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
	DebugParameters debug_param;
	OctreeParameters octree_param;
protected:
	void LoadAllParameters(boost::program_options::options_description& options, bool inital);
	void LoadReaderParameters(boost::program_options::options_description& options, bool inital);
	void LoadICPParameters(boost::program_options::options_description& options, bool inital);
	void LoadOctreeParameters(boost::program_options::options_description& options, bool inital);
	void LoadDebugParameters(boost::program_options::options_description& options, bool inital);
};