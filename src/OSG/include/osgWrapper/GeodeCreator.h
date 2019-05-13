#pragma once
#ifndef GEODE_CREATOR_H
#define GEODE_CREATOR_H
#include <osg\Geode>
#include <osg\Geometry>

namespace OSGWrapper
{

class OSG_EXPORT GeodeCreator
{
public:
	static osg::Geode* CreateLineSegments(float* boundary_position, unsigned* boundary_segment_size, unsigned boundary_size);
	static osg::Geode* CreateGeode();
	static osg::Geode* CreateQuad();
	static osg::Geode* CreateGeode(unsigned vertex_number, float* vertex_position, float* vertex_normal, float* vertex_color, unsigned triangle_number, unsigned* triangle_index);
	static osg::Geode* CreatePoint(unsigned vertex_number, float* vertex_position, float* vertex_normal, float* vertex_color);
	static osg::Geode* CreateIndexAttributeGeode(osg::PrimitiveSet* primitive, osg::Array* attribute0 = 0, osg::Array* attribute1 = 0,
		osg::Array* attribute2 = 0, osg::Array* attribute3 = 0, osg::Array* attribute4 = 0);
};

}
#endif // ARRAY_CREATOR