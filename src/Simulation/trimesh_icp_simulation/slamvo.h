#pragma once
#include "interface.h"
#include <base\threading\thread.h>
#include "voimpl.h"
#include "locatetracerimpl.h"

class SlamVO : public base::Thread , public VO
{
public:
	SlamVO();
	virtual ~SlamVO();

	void StartVO(const SlamParameters& parameters);
	void StopVO();

	void OnFrame(trimesh::TriMesh* mesh);
	void SetVOTracer(VOTracer* tracer);
	void SetVOProfiler(VOProfiler* profiler);
	void SetLocateTracer(LocateTracer* tracer);
	void SetICPTracer(trimesh::ProjectionICPTracer* tracer);
	void SetKeyFrameTracer(KeyFrameTracer* tracer);
protected:
	void ProcessFrame(trimesh::TriMesh* mesh);
private:
	SlamParameters m_parameters;
	VOProfiler* m_profiler;
	VOImpl m_vo_impl;

	VOTracer* m_tracer;
	KeyFrameTracer* m_keyframe_tracer;
};