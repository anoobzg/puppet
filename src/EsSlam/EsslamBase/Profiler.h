#pragma once
#include "csvwriter.h"
#include "EsslamBaseExport.h"
#define PROFILE_TRACE

namespace esslam
{
	class ESSLAM_API Profiler
	{
		Profiler();
		~Profiler();

	public:
		static Profiler& Instance();

		string_util::CSVWriter& GetLocate() { return m_locate_writer; }
		string_util::CSVWriter& GetFusion() { return m_fusion_writer; }

		void Start();
		void Output();
	private:
		static Profiler m_profiler;

		string_util::CSVWriter m_locate_writer;
		string_util::CSVWriter m_fusion_writer;
	};
}

#ifdef PROFILE_TRACE
#define PROFILE_START esslam::Profiler::Instance().Start();

#define LOCATE_TICK esslam::Profiler::Instance().GetLocate().Tick();
#define FUSION_TICK esslam::Profiler::Instance().GetFusion().Tick();

#define PROFILE_OUTPUT esslam::Profiler::Instance().Output();

#else
#define PROFILE_START

#define LOCATE_TICK
#define FUSION_TICK

#define PROFILE_OUTPUT
#endif
