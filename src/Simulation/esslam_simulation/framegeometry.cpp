#include "framegeometry.h"

FrameGeometry::FrameGeometry()
{
	m_coord_array = new osg::Vec3Array();
	m_normal_array = new osg::Vec3Array();
	m_coord_array->reserve(1300000);
	m_normal_array->reserve(1300000);
	m_draw_array = new osg::DrawArrays(GL_POINTS);

	setVertexAttribArray(0, m_coord_array, osg::Array::BIND_PER_VERTEX);
	setVertexAttribArray(1, m_normal_array, osg::Array::BIND_PER_VERTEX);

	addPrimitiveSet(m_draw_array);
	setUseVertexBufferObjects(true);
	setUseDisplayList(false);
	setCullingActive(false);
}

FrameGeometry::~FrameGeometry()
{

}

void FrameGeometry::Update(int num, const std::vector<trimesh::vec3>& points, const std::vector<trimesh::vec3>& normals)
{
	if (num == 0) return;

	m_coord_array->resize(num);
	m_normal_array->resize(num);

	memcpy(&m_coord_array->at(0), &points.at(0), num * 3 * sizeof(float));
	memcpy(&m_normal_array->at(0), &normals.at(0), num * 3 * sizeof(float));
	m_coord_array->dirty();
	m_normal_array->dirty();
	m_draw_array->set(GL_POINTS, 0, num);
}