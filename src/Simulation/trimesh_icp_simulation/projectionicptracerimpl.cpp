#include "projectionicptracerimpl.h"

ProjectionICPTracerImpl::ProjectionICPTracerImpl()
{
	m_writer.PushHead("error");
	m_writer.PushHead("correspondece");
}

ProjectionICPTracerImpl::~ProjectionICPTracerImpl()
{

}

void ProjectionICPTracerImpl::OnError(float error)
{
	m_writer.PushData((double)error);
}

void ProjectionICPTracerImpl::OnPreStepCorrespondences(const std::vector<trimesh::PtPair>& correspondences)
{
	
}

void ProjectionICPTracerImpl::OnStepCorrespondences(const std::vector<trimesh::PtPair>& correspondences)
{
	m_writer.PushData((double)correspondences.size());
}

void ProjectionICPTracerImpl::Save(const std::string& file)
{
	m_writer.Output(file);
}