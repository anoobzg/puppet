#include "button.h"
#include <osgWrapper\ArrayCreator.h>
#include <osgWrapper\GeometryCreator.h>
#include <osgWrapper\UtilCreator.h>
#include <osgDB\ReadFile>

Button::Button()
{
	osg::Geometry* geometry = CREATE_UNIT_QUAD;
	AddChild(geometry);

	SetSize(osg::Vec2(100.0f, 100.0f));
}

Button::~Button()
{

}

void Button::SetHover(const std::string& file)
{
	if (!m_hover_image)
	{
		m_hover_image = new osg::Texture2D();
		m_hover_image->setImage(Load(file));
	}
}

void Button::SetNormal(const std::string& file)
{
	if (!m_normal_image)
	{
		m_normal_image = new osg::Texture2D();
		m_normal_image->setImage(Load(file));
	}
}

void Button::SetSelect(const std::string& file)
{
	if (!m_select_image)
	{
		m_select_image = new osg::Texture2D();
		m_select_image->setImage(Load(file));
	}
}

void Button::OnUnHover()
{

}

void Button::OnHover()
{

}

void Button::OnPushDown()
{

}

void Button::OnPushUp()
{

}

void Button::OnClick()
{

}

osg::Image* Button::Load(const std::string& file)
{
	return osgDB::readImageFile(file);
}
