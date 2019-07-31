#include "toolbar.h"
#include <osgWrapper/WrapperColor.h>

ToolBar::ToolBar()
{
	m_item_size = osg::Vec2f(200.0f, 200.0f);
	m_offset = osg::Vec2f(500.0f, 50.0f);
	m_button1 = new EsButton(WRAPPER_RED);
	m_button1->SetRect(m_offset, m_item_size);

	AddChild(m_button1);
	m_button2 = new EsButton(WRAPPER_GREEN);
	m_button2->SetRect(m_offset + osg::Vec2f(200.0f, 0.0f), m_item_size);
	AddChild(m_button2);
	m_button3 = new EsButton(WRAPPER_BLUE);
	m_button3->SetRect(m_offset + osg::Vec2f(400.0f, 0.0f), m_item_size);
	AddChild(m_button3);
	m_button4 = new EsButton(WRAPPER_YELLOW);
	m_button4->SetRect(m_offset + osg::Vec2f(600.0f, 0.0f), m_item_size);
	AddChild(m_button4);
	m_button5 = new EsButton(WRAPPER_WHITE);
	m_button5->SetRect(m_offset + osg::Vec2f(800.0f, 0.0f), m_item_size);
	AddChild(m_button5);
}

ToolBar::~ToolBar()
{

}

OSGWrapper::QuadAttributeUtilNode* ToolBar::Generate()
{
	OSGWrapper::QuadAttributeUtilNode* node = new OSGWrapper::QuadAttributeUtilNode(1);
	for (size_t i = 0; i < m_children_quads.size(); ++i)
		node->AddChild(m_children_quads.at(i)->Generate());
	
	return node;
}