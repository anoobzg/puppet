#include "framenode.h"

FrameNode::FrameNode()
	:m_first(true)
{
	m_matrix = osg::Matrixf::identity();
	m_matrix_uniform = new osg::Uniform("align_matrix", m_matrix);
	AddUniform(m_matrix_uniform);

	SetRenderProgram("distancephong");

	m_used_geometry = new FrameGeometry();
	m_free_geometry = new FrameGeometry();
}

FrameNode::~FrameNode()
{

}

void FrameNode::UpdateGlobalMatrix(const osg::Matrixf& matrix)
{
	m_matrix = matrix;
}

FrameGeometry* FrameNode::GetUsedGeometry()
{
	return m_used_geometry;
}

FrameGeometry* FrameNode::GetFreeGeometry()
{
	return m_free_geometry;
}

bool FrameNode::First()
{
	return m_first;
}

void FrameNode::Exchange(bool add)
{
	m_first = false;
	RemoveAll();
	m_matrix_uniform->set(m_matrix);
	osg::ref_ptr<FrameGeometry> temp = m_free_geometry;
	m_free_geometry = m_used_geometry;
	m_used_geometry = temp;
	if(add) AddChild(m_used_geometry);
}