#pragma once
#include <osg/Geometry>

class ScreenLines : public osg::Geometry
{
public:
	ScreenLines();
	virtual ~ScreenLines();

	void Clear();
	void Add(const osg::Vec2& p);
protected:
	osg::ref_ptr<osg::Vec2Array> m_lines_array;
	osg::ref_ptr<osg::DrawArrays> m_draw_array;
};