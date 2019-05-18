#pragma once
#pragma once
#include <osgWrapper\GeometryCreator.h>

class ControlGroupGeode : public osg::Geode
{
public:
	ControlGroupGeode();
	~ControlGroupGeode();

	void Select();
	void Unselect();

	unsigned m_group_id;
private:
	osg::ref_ptr<osg::Uniform> m_group_selected;
};