#include "control_ball.h"
#include <osgWrapper\ArrayCreator.h>
#include <OSGBuilder\GeometryCache.h>

#include <iostream>

using namespace OSGBuilder;
ControlBall::ControlBall(unsigned vertex_handle, float x, float y, float z)
	:m_radius(0.1f)
{
	osg::Geometry* geometry = GeometryCache::Instance().Get(CGT_Ball);
	addDrawable(geometry);

	setCullingActive(false);

	m_matrix = new osg::Uniform("position_matrix", osg::Matrixf::identity());
	getOrCreateStateSet()->addUniform(m_matrix);

	m_color = new osg::Uniform("ball_color", osg::Vec4f(0.0f, 0.0f, 0.0f, 1.0f));
	getOrCreateStateSet()->addUniform(m_color);

	m_selected = new osg::Uniform("point_selected", 0.0f);

	m_vertex_handle = vertex_handle;
	m_x = x;
	m_y = y;
	m_z = z;

	m_matrix->set(osg::Matrixf::scale(m_radius, m_radius, m_radius) * osg::Matrixf::translate(x, y, z));
}

ControlBall::~ControlBall()
{
}

void ControlBall::Select()
{
	m_selected->set(1.0f);
}

void ControlBall::Unselect()
{
	m_selected->set(0.0f);
}

void ControlBall::SetColor(const osg::Vec4& color)
{
	m_color->set(color);
}