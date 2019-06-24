#pragma once
#include<osgWrapper\AttributeUtilNode.h>
#include "screenboundingbox.h"
#include "screenlines.h"

class ScreenGraph : public OSGWrapper::AttributeUtilNode
{
public:
	ScreenGraph();
	~ScreenGraph();

	void Clear();
	void AddError(float value);
protected:
	std::vector<float> m_values;
	int m_step_total_count;
	float m_min_error;
	float m_max_error;

	osg::ref_ptr<ScreenBoundingBox> m_box;
	osg::ref_ptr<ScreenLines> m_lines;
};