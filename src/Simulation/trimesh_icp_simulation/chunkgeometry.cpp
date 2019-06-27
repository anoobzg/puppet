#include "chunkgeometry.h"

ChunkGeometry::ChunkGeometry()
	:m_current_size(0), m_state(0)
{
	m_coord_array = new osg::Vec3Array();
	m_coord_array->reserve(ChunkVertexSize);
	m_normal_array = new osg::Vec3Array();
	m_normal_array->reserve(ChunkVertexSize);
	m_color_array = new osg::Vec4Array();
	m_color_array->reserve(ChunkVertexSize);
	m_time_array = new osg::FloatArray();
	m_time_array->reserve(ChunkVertexSize);
	m_draw_arrays = new osg::DrawArrays();

	setVertexAttribArray(0, m_coord_array, osg::Array::BIND_PER_VERTEX);
	setVertexAttribArray(1, m_normal_array, osg::Array::BIND_PER_VERTEX);
	setVertexAttribArray(2, m_color_array, osg::Array::BIND_PER_VERTEX);
	setVertexAttribArray(3, m_time_array, osg::Array::BIND_PER_VERTEX);

	addPrimitiveSet(m_draw_arrays);
	setUseVertexBufferObjects(true);
	setUseDisplayList(false);
	setCullingActive(false);
}

ChunkGeometry::~ChunkGeometry()
{

}

void ChunkGeometry::Update(int index, const osg::Vec3f& p, const osg::Vec3f& n, const osg::Vec4f& c,
	float t)
{
	while (m_current_size <= index)
	{
		m_coord_array->push_back(osg::Vec3f());
		m_normal_array->push_back(osg::Vec3f());
		m_color_array->push_back(osg::Vec4f());
		m_time_array->push_back(0.0f);
		++m_current_size;
		m_state = 2;
	}

	if(m_state == 0)
		m_state = 1;
	m_coord_array->at(index) = p;
	m_normal_array->at(index) = n;
	m_color_array->at(index) = c;
	m_time_array->at(index) = t;
}

void ChunkGeometry::Check()
{
	if (m_state != 0)
	{
		if (m_state == 2)
		{
			m_coord_array->dirty();
			m_color_array->dirty();

			m_draw_arrays->set(GL_POINTS, 0, m_current_size);
			m_draw_arrays->dirty();
		}

		m_normal_array->dirty();
		m_time_array->dirty();

		m_state = 0;
	}
}

int ChunkGeometry::GetCount()
{
	return m_current_size;
}