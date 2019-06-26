#pragma once
#include "interface.h"
#include <base\threading\thread.h>
#include "voimpl.h"

class SlamVO : public base::Thread , public VO
{
public:
	SlamVO();
	virtual ~SlamVO();

	void StartVO(const ICPParamters& parameters);
	void StopVO();

	void OnFrame(trimesh::TriMesh* mesh);
	void SetVOTracer(VOTracer* tracer);
	void SetVOProfiler(VOProfiler* profiler);
protected:
	void ProcessFrame(trimesh::TriMesh* mesh);
private:
	ICPParamters m_parameters;
	VOProfiler* m_profiler;
	VOImpl m_vo_impl;

	VOTracer* m_tracer;
};