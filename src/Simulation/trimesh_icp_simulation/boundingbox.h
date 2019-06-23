#pragma once
#include<osgWrapper\AttributeUtilNode.h>

class BoundingBox : public OSGWrapper::AttributeUtilNode
{
public:
	BoundingBox();
	~BoundingBox();

	void UpdateBoundingBox(const osg::BoundingBoxf& box, const osg::Matrixf& matrix);
	void UpdateBoundingBox(const osg::Vec3f& min, const osg::Vec3f& max);

protected:
	osg::ref_ptr<osg::Uniform> m_matrix;
};