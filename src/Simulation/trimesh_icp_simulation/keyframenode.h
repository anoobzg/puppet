#pragma once
#include <osgWrapper\AttributeUtilNode.h>

class KeyFrameNode : public OSGWrapper::AttributeUtilNode
{
public:
	KeyFrameNode();
	~KeyFrameNode();

	void SetKeyMatrix(const osg::Matrixf& matrix);
	void SetDefaultID();
	void GenerateBoundingBox();
protected:
private:
	osg::ref_ptr<osg::Uniform> m_color_uniform;
	osg::ref_ptr<osg::Uniform> m_matrix_uniform;
};