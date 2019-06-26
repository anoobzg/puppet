#include "voprofilerimpl.h"

VOProfilerImpl::VOProfilerImpl()
{
	m_writer.PushHead("count");
	m_writer.PushHead("type");
	m_writer.PushHead("time");
}

VOProfilerImpl::~VOProfilerImpl()
{
}

void VOProfilerImpl::OnBeforeLocate()
{
	m_writer.TickStart();
}

void VOProfilerImpl::OnAfterLocate()
{
	m_writer.TickEnd();
}

void VOProfilerImpl::OnMesh(const trimesh::TriMesh& mesh)
{
	m_writer.PushData((double)mesh.vertices.size());
}

void VOProfilerImpl::OnLocateResult(const LocateData& locate_data)
{
	m_writer.PushData((double)locate_data.locate_type);
}

void VOProfilerImpl::Write(const std::string& file)
{
	m_writer.Output(file);
}