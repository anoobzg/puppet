#include "es_maintoolbar.h"

ESMainToolbar::ESMainToolbar()
{
	m_scan_button = new EsButton();
	m_clear_button = new EsButton();
}

ESMainToolbar::~ESMainToolbar()
{

}

void ESMainToolbar::Setup(const osg::Vec2f& offset, const osg::Vec2f& size)
{
	m_scan_button->SetRect(offset, size);
	m_clear_button->SetRect(offset + osg::Vec2f(0.0f, size.y()), size);
}

EsButton* ESMainToolbar::GetScanButton()
{
	return m_scan_button;
}

EsButton* ESMainToolbar::GetClearButton()
{
	return m_clear_button;
}