#pragma once
#include <osgWrapper\AttributeUtilNode.h>

class PatchNode : public OSGWrapper::AttributeUtilNode
{
public:
	PatchNode();
	~PatchNode();

	void SetColor(const osg::Vec4f& color);
	void UpdateMatrix(const osg::Matrixf& matrix);
protected:
	osg::ref_ptr<osg::Uniform> m_color;
	osg::ref_ptr<osg::Uniform> m_matrix_uniform;
};