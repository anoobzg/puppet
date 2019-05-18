#include "control_group_geode.h"
#include <osgWrapper\ArrayCreator.h>

#include <iostream>

ControlGroupGeode::ControlGroupGeode()
{
	m_group_selected = new osg::Uniform("scale_matrix", osg::Matrixf::scale(1.0f, 1.0f, 1.0f));
	getOrCreateStateSet()->addUniform(m_group_selected);

	static unsigned id = 0;
	m_group_id = id++;
}

ControlGroupGeode::~ControlGroupGeode()
{
}

void ControlGroupGeode::Select()
{
	m_group_selected->set(osg::Matrixf::scale(2.0f, 2.0f, 2.0f));
}

void ControlGroupGeode::Unselect()
{
	m_group_selected->set(osg::Matrixf::scale(1.0f, 1.0f, 1.0f));
}
