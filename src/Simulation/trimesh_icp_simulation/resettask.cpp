#include "resettask.h"

ResetICPTask::ResetICPTask(ICPNode* source, ScreenGraph* screen_graph)
	:m_source(source), m_graph(screen_graph)
{

}

ResetICPTask::~ResetICPTask()
{

}

bool ResetICPTask::Execute()
{
	m_source->UpdateMatrix(osg::Matrixf::identity());
	m_graph->Clear();
	m_lines->RemoveAll();
	return false;
}

void ResetICPTask::SetAttributeNode(OSGWrapper::AttributeUtilNode* node)
{
	m_lines = node;
}