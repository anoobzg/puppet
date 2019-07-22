#pragma once
#include "quad.h"

class ToolBar : public Quad
{
public:
	ToolBar();
	~ToolBar();

	OSGWrapper::QuadAttributeUtilNode* Generate();
protected:
	osg::Vec2f m_offset;
	osg::Vec2f m_item_size;

	osg::ref_ptr<Quad> m_button1;
	osg::ref_ptr<Quad> m_button2;
	osg::ref_ptr<Quad> m_button3;
	osg::ref_ptr<Quad> m_button4;
	osg::ref_ptr<Quad> m_button5;
};
