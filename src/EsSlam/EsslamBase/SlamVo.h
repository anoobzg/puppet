#pragma once
#include "../interface/slam_interface.h"
#include <base\threading\thread.h>
#include "InnerInterface.h"
#include "SlamParameters.h"
#include "VoImpl.h"
#include "VOLocator.h"
#include "VOFusion.h"

namespace esslam
{
	class SlamVO : public base::Thread, public VO, public KeyFrameAdder
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
		void SetFixMode();

		void AddKeyFrame(TriMeshPtr mesh);
	protected:
		void ProcessFrame(trimesh::TriMesh* mesh);
		void ProcessAddKeyFrame(TriMeshPtr mesh);
	private:
		SlamParameters m_parameters;
		VOImpl m_vo_impl;
		VOLocator m_vo_locator;
		VOFusion m_vo_fusion;
		VOProfiler* m_profiler;

		VisualProcessor* m_visual_processor;
	};
}