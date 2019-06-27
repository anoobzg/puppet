#include "slamvo.h"
#include <base\bind.h>
#include "timestamp.h"

SlamVO::SlamVO()
	:base::Thread("SlamVO"), m_tracer(NULL), m_profiler(NULL)
{

}

SlamVO::~SlamVO()
{

}

void SlamVO::StartVO(const SlamParameters& parameters)
{
	m_parameters = parameters;

	m_vo_impl.Setup(m_parameters);
	m_vo_impl.SetVOTracer(m_tracer);

	bool start = Start();
}

void SlamVO::StopVO()
{
	m_profiler = NULL;
	Stop();
	m_tracer = NULL;
}

void SlamVO::SetVOProfiler(VOProfiler* profiler)
{
	m_profiler = profiler;
}

void SlamVO::OnFrame(trimesh::TriMesh* mesh)
{
	if(mesh)
		task_runner()->PostTask(FROM_HERE, base::Bind(&SlamVO::ProcessFrame, base::Unretained(this), mesh));
}

void SlamVO::ProcessFrame(trimesh::TriMesh* mesh)
{
	if (m_profiler)
	{
		m_profiler->OnMesh(*mesh);
		m_profiler->OnBeforeLocate();
	}

	TriMeshPtr mesh_ptr(mesh);
	LocateData locate_data;
	locate_data.lost = true;
	locate_data.locate_type = 0;

	m_vo_impl.ProcessOneFrame(mesh_ptr, locate_data);

	if (m_profiler)
	{
		m_profiler->OnLocateResult(locate_data);
		m_profiler->OnAfterLocate();
	}
}

void SlamVO::SetVOTracer(VOTracer* tracer)
{
	m_tracer = tracer;
}