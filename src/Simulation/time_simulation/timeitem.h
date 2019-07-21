#pragma once
#include <osgWrapper/UIItem.h>

class TimeItem : public OSGWrapper::UIItem
{
public:
	TimeItem(float delta);
	virtual ~TimeItem();

	void SetColor(const osg::Vec4& color);
protected:
	float m_delta;
	osg::ref_ptr<osg::Uniform> m_color;
};