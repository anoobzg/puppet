#include "line.h"
#include <osgWrapper\GeometryCreator.h>

using namespace OSGWrapper;
Line::Line()
{
	m_coord_array = new osg::Vec3Array();
	m_coord_array->push_back(osg::Vec3(-1000000.0f, -1000000.0f, 0.0f));
	m_coord_array->push_back(osg::Vec3(-1000000.0f, -1000000.0f, 0.0f));
	osg::DrawArrays* primitive_set = new osg::DrawArrays(GL_LINES, 0, 2);
	osg::Geometry* geometry = GeometryCreator::CreateIndexAttributeGeometry(primitive_set, m_coord_array);
	geometry->setCullingActive(false);
	addChild(geometry);
}

Line::~Line()
{

}

void Line::Set(const osg::Vec3f& p1, const osg::Vec3f& p2)
{
	m_coord_array->at(0) = p1;
	m_coord_array->at(1) = p2;
	m_coord_array->dirty();
}

void Line::SetP1(const osg::Vec3f& p1)
{
	m_coord_array->at(0) = p1;
	m_coord_array->dirty();
}

void Line::SetP2(const osg::Vec3f& p2)
{
	m_coord_array->at(1) = p2;
	m_coord_array->dirty();
}

void Line::Get(osg::Vec3f& p1, osg::Vec3f& p2)
{
	p1 = m_coord_array->at(0);
	p2 = m_coord_array->at(1);
}