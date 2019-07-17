#pragma once
#include "EsslamBaseExport.h"
#include <string>
#include <memory>

namespace boost
{
	namespace program_options
	{
		class options_description;
	}
}

namespace esslam
{
	struct ImageParameters
	{
		int width;
		int height;
	};

	struct ReaderParameters
	{
		bool load_from_file;
		std::string directory;
		std::string pattern;
		float time;
		int use_t_vo;
	};

	struct ICPParamters
	{
		std::string calib_file;
		int use_fast;
		int least_frame_count;
	};

	struct DebugParameters
	{
		int debug;
		std::string directory;
		std::string out_directory;
		int save_fm_failed;
		int profile_detail;
	};

	struct OctreeParameters
	{
		int cell_depth;
	};

	class ESSLAM_API SlamParameters
	{
	public:
		SlamParameters();
		~SlamParameters();

		void LoadDefault();
		void LoadFromFile(const std::string& file);

		ImageParameters image_param;
		ReaderParameters reader_param;
		ICPParamters icp_param;
		DebugParameters debug_param;
		OctreeParameters octree_param;
	protected:
		void LoadAllParameters(boost::program_options::options_description& options, bool inital);
		void LoadImageParameters(boost::program_options::options_description& options, bool inital);
		void LoadReaderParameters(boost::program_options::options_description& options, bool inital);
		void LoadICPParameters(boost::program_options::options_description& options, bool inital);
		void LoadOctreeParameters(boost::program_options::options_description& options, bool inital);
		void LoadDebugParameters(boost::program_options::options_description& options, bool inital);
	};

}