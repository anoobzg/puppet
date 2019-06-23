#pragma once
#include <osg/Geometry>

class ScreenBoundingBox : public osg::Geometry
{
public:
	ScreenBoundingBox();
	virtual ~ScreenBoundingBox();

	void Update(const osg::Vec2f& min, const osg::Vec2f& max);
	const osg::Vec2f& GetMin() { return m_min; }
	const osg::Vec2f& GetMax() { return m_max; }
protected:
	osg::ref_ptr<osg::Vec2Array> m_box_array;
	osg::Vec2f m_min;
	osg::Vec2f m_max;
};