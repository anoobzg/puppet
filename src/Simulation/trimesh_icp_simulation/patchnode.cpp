#include "patchnode.h"

PatchNode::PatchNode()
{
	SetRenderProgram("trimeshicptest");
	m_color = new osg::Uniform("color", osg::Vec4f(1.0f, 1.0f, 1.0f, 1.0f));
	AddUniform(m_color);

	m_matrix_uniform = new osg::Uniform("icp_matrix", osg::Matrixf::identity());
	AddUniform(m_matrix_uniform);
}

PatchNode::~PatchNode()
{

}

void PatchNode::SetColor(const osg::Vec4f& color)
{
	m_color->set(color);
}

void PatchNode::UpdateMatrix(const osg::Matrixf& matrix)
{
	m_matrix_uniform->set(matrix);
}