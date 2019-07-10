#pragma once
#include "../interface/slam_interface.h"
#include <base\threading\thread.h>
#include "InnerInterface.h"
#include "SlamParameters.h"
#include "VoImpl.h"

namespace esslam
{
	class SlamVO : public base::Thread, public VO
	{
	public:
		SlamVO();
		virtual ~SlamVO();

		void StartVO(const SlamParameters& parameters);
		void StopVO();

		void OnFrame(trimesh::TriMesh* mesh);
		void SetVisualProcessor(VisualProcessor* processor);
		void SetVOProfiler(VOProfiler* profiler);
		void SetLocateTracer(LocateTracer* tracer);
		void SetICPTracer(trimesh::ProjectionICPTracer* tracer);

		void Clear();

		void Build(IBuildTracer& tracer);
	protected:
		void ProcessFrame(trimesh::TriMesh* mesh);
	private:
		SlamParameters m_parameters;
		VOImpl m_vo_impl;
		VOProfiler* m_profiler;
	};
}