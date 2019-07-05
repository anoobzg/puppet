#pragma once
#include <osg\Geometry>
#include <osg\Array>
#define ChunkVertexSize 50000

class ChunkGeometry : public osg::Geometry
{
public:
	ChunkGeometry();
	~ChunkGeometry();

	void Update(int index, const osg::Vec3f& p, const osg::Vec3f& n, const osg::Vec4f& c,
		float t);
	void Check();
	int GetCount();
protected:
	osg::ref_ptr<osg::Vec3Array> m_coord_array;
	osg::ref_ptr<osg::Vec3Array> m_normal_array;
	osg::ref_ptr<osg::Vec4Array> m_color_array;
	osg::ref_ptr<osg::FloatArray> m_time_array;
	osg::ref_ptr<osg::DrawArrays> m_draw_arrays;

	int m_current_size;
	int m_state;
};