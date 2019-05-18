#include "cloud.h"
#include <osgWrapper\GeometryCreator.h>

using namespace OSGWrapper;
Cloud::Cloud()
{
	m_coord_array = new osg::Vec3Array();
	m_draw_array = new osg::DrawArrays(GL_LINES, 0, 0);
	osg::Geometry* geometry = GeometryCreator::CreateIndexAttributeGeometry(m_draw_array, m_coord_array);
	geometry->setCullingActive(false);
	addChild(geometry);
	setCullingActive(false);
}

Cloud::~Cloud()
{

}

void Cloud::AddPoint(float x, float y, float nx, float ny)
{
	Point p;
	p.x = x; p.y = y; p.nx = nx; p.ny = ny;
	m_clouds.push_back(p);

	osg::Vec3f p1(x, y, 0.0f);
	osg::Vec3f p2 = p1 + osg::Vec3f(nx, ny, 0.0f) * 1.0f;
	m_coord_array->push_back(p1);
	m_coord_array->push_back(p2);
	size_t size = m_draw_array->getCount() + 2;
	m_draw_array = new osg::DrawArrays(GL_LINES, 0, size);
	dynamic_cast<osg::Geometry*>(getDrawable(0))->setPrimitiveSet(0, m_draw_array);

	m_coord_array->dirty();
	m_draw_array->dirty();
}

void Cloud::Clear()
{
	m_clouds.clear();
	m_coord_array->clear();
	m_draw_array->setCount(0);

	m_coord_array->dirty();
	m_draw_array->dirty();
}

std::vector<Point>& Cloud::GetPoints()
{
	return m_clouds;
}
