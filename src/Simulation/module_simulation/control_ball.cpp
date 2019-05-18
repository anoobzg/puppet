#include "control_ball.h"
#include <osgWrapper\ArrayCreator.h>
#include "GeometryCache.h"

#include <iostream>

using namespace OSGBuilder;
ControlBall::ControlBall(unsigned handle)
	:m_select(false), m_handle(handle), m_release(false), m_radius(0.1f)
{
	osg::Geometry* geometry = GeometryCache::Instance().Get(CGT_Ball);
	addDrawable(geometry);

	setCullingActive(false);

	m_matrix = new osg::Uniform("position_matrix", osg::Matrixf::identity());
	getOrCreateStateSet()->addUniform(m_matrix);

	m_color = new osg::Uniform("color", osg::Vec4f(0.0f, 0.5f, 0.0f, 1.0f));
	getOrCreateStateSet()->addUniform(m_color);
}

ControlBall::~ControlBall()
{
	std::cout << "Destroy" << std::endl;
}

void ControlBall::SetPosition(float x, float y, float z)
{
	m_matrix->set(osg::Matrixf::scale(m_radius, m_radius, m_radius) * osg::Matrixf::translate(x, y, z));
}

void ControlBall::GetModelMatrix(osg::Matrixf& out)
{
	m_matrix->getElement(0, out);
}

void ControlBall::Hover()
{
	if (m_select)
		return;

	m_color->set(osg::Vec4f(0.0f, 0.9f, 0.0f, 1.0f));
}

void ControlBall::Unhover()
{
	//std::cout << "UnHover " << m_handle << " " << this <<std::endl;

	if (m_select)
		return;
	m_color->set(osg::Vec4f(0.0f, 0.5f, 0.0f, 1.0f));
}

void ControlBall::Select()
{
	m_select = true;
	m_color->set(osg::Vec4f(0.0f, 0.0f, 0.0f, 1.0f));
}

void ControlBall::Unselect()
{
	m_select = false;
	m_color->set(osg::Vec4f(0.0f, 0.5f, 0.0f, 1.0f));
}

unsigned ControlBall::Handle()
{
	return m_handle;
}