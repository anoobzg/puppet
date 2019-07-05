#pragma once
#include "EsslamBaseExport.h"
#include "SlamParameters.h"
#include "VoProfilerImpl.h"
#include "LocateTracerImpl.h"
#include "ProjectionicpTracerImpl.h"

namespace esslam
{

	class ESSLAM_API DebugCenter
	{
	public:
		DebugCenter();
		~DebugCenter();

		void SetParameters(const DebugParameters& parameters);
		void Save();
		VOProfiler* GetVOProfiler();
		LocateTracer* GetLocateTracer();
		trimesh::ProjectionICPTracer* GetProjectionICPTracer();
	private:
		VOProfilerImpl m_vo_profiler;
		LocateTracerImpl m_locate_tracer;
		ProjectionICPTracerImpl m_icp_tracer;

		std::string m_debug_directory;
	};
}