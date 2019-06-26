#pragma once
#include <osgWrapper/AttributeUtilNode.h>

class OctreeGrid : public OSGWrapper::AttributeUtilNode
{
public:
	OctreeGrid(const osg::Vec3f& center);
	~OctreeGrid();
};