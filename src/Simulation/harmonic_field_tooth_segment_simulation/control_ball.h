#pragma once
#include <osgWrapper\GeometryCreator.h>
#include "control_group_geode.h"

class ControlBall : public osg::Geode
{
public:
	ControlBall(unsigned vertex_handle, float x, float y, float z);
	~ControlBall();

	void Select();
	void Unselect();

	void SetColor(const osg::Vec4& color);
public:
	unsigned m_vertex_handle;
	float m_x;
	float m_y;
	float m_z;
	float m_radius;

	osg::ref_ptr<ControlGroupGeode> m_parent;
private:
	osg::ref_ptr<osg::Uniform> m_matrix;
	osg::ref_ptr<osg::Uniform> m_color;
	osg::ref_ptr<osg::Uniform> m_selected;
};