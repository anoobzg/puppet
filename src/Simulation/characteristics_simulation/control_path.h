#pragma once
#include <osgWrapper\GeometryCreator.h>

class ControlPath : public osg::Geometry
{
public:
	ControlPath(unsigned handle, unsigned control_point_handle[2], osg::DrawArrays* primitive_set, osg::Array* coord_array);
	~ControlPath();

	unsigned GetHandle();

private:
	unsigned m_handle;
	unsigned m_control_point_handle[2];
};