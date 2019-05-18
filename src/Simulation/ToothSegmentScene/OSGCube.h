#pragma once
#include <osgWrapper\GeometryCreator.h>

class OSGCube : public osg::Geometry
{
public:
	OSGCube(float* center, float* fScale);
	void SetCenter(float* center);

	void SetColor(float* rgba);
	
private:
	osg::ref_ptr<osg::Uniform> m_matrix;
};