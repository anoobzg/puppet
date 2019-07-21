#include "timeitem.h"
#include <osgWrapper/UtilCreator.h>

TimeItem::TimeItem(float delta)
	:m_delta(delta)
{
	m_color = new osg::Uniform("color", osg::Vec4(1.0f, 0.0f, 0.0f, 1.0f));
	AddUniform(m_color);

	AddChild(CREATE_UNIT_QUAD);
}

TimeItem::~TimeItem()
{

}

void TimeItem::SetColor(const osg::Vec4& color)
{
	m_color->set(color);
}