#include "esbutton.h"

EsButton::EsButton(const osg::Vec4f& color)
	:m_color(color), m_selected(false)
{
	SetColor(m_color);
	SetUseTexture(false);
}

EsButton::~EsButton()
{

}

void EsButton::OnDoubleClick()
{

}

void EsButton::OnUnHover()
{
	if (m_selected) return;
	SetColor(m_color);
}

void EsButton::OnHover()
{
	if (m_selected) return;
	SetColor(m_color * 0.6f);
}

void EsButton::OnPushDown()
{

}

void EsButton::OnPushUp()
{

}

void EsButton::OnClick()
{
	m_selected = !m_selected;
	if (m_selected) SetColor(m_color * 0.3f);
	else OnHover();
}