#include "es_edittoolbar.h"

ESEditToolbar::ESEditToolbar()
{
	m_exit_button = new EsButton();
}

ESEditToolbar::~ESEditToolbar()
{

}

EsButton* ESEditToolbar::GetExitButton()
{
	return m_exit_button;
}

void ESEditToolbar::Setup(const osg::Vec2f& offset, const osg::Vec2f& size)
{
	m_exit_button->SetRect(offset, size);
}