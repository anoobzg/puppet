#include "es_ui.h"

ESUI::ESUI()
{
	m_maintoolbar.Setup(osg::Vec2(40.0f, 600.0f), osg::Vec2f(100.0f, 100.0f));
	m_edittoolbar.Setup(osg::Vec2(400.0f, 100.0f), osg::Vec2f(100.0f, 100.0f));
	AddChild(m_maintoolbar.GetScanButton());
	AddChild(m_maintoolbar.GetClearButton());
	AddChild(m_edittoolbar.GetExitButton());
}

ESUI::~ESUI()
{

}

OSGWrapper::QuadAttributeUtilNode* ESUI::Generate()
{
	OSGWrapper::QuadAttributeUtilNode* node = new OSGWrapper::QuadAttributeUtilNode(1);
	for (size_t i = 0; i < m_children_quads.size(); ++i)
		node->AddChild(m_children_quads.at(i)->Generate());

	return node;
}