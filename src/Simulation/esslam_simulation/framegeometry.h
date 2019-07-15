#pragma once
#include <osg/Geometry>
#include "Vec.h"

class FrameGeometry : public osg::Geometry
{
public:
	FrameGeometry();
	~FrameGeometry();
	
	void Update(int num, const std::vector<trimesh::vec3>& points, const std::vector<trimesh::vec3>& normals);
private:
	osg::ref_ptr<osg::Vec3Array> m_coord_array;
	osg::ref_ptr<osg::Vec3Array> m_normal_array;
	osg::ref_ptr<osg::DrawArrays> m_draw_array;
};