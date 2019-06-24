#include "screenboundingbox.h"

ScreenBoundingBox::ScreenBoundingBox()
{
	m_box_array = new osg::Vec2Array();
	m_box_array->resize(4, osg::Vec2(0.0f, 0.0f));
	osg::DrawElementsUInt* draw_array = new osg::DrawElementsUInt(GL_LINE_LOOP);
	draw_array->push_back(0); draw_array->push_back(1); draw_array->push_back(2); draw_array->push_back(3);

	setVertexAttribArray(0, m_box_array, osg::Array::BIND_PER_VERTEX);
	addPrimitiveSet(draw_array);
	setCullingActive(false);
	setUseDisplayList(false);
	setUseVertexBufferObjects(true);
	setComputeBoundingBoxCallback(new osg::Drawable::ComputeBoundingBoxCallback());

	m_min = osg::Vec2f(0.0f, 0.0f);
	m_max = osg::Vec2f(0.0f, 0.0f);;
}

ScreenBoundingBox::~ScreenBoundingBox()
{

}

void ScreenBoundingBox::Update(const osg::Vec2f& min, const osg::Vec2f& max)
{
	m_box_array->at(0) = min;
	m_box_array->at(1) = osg::Vec2(max.x(), min.y());
	m_box_array->at(2) = max;
	m_box_array->at(3) = osg::Vec2(min.x(), max.y());
	m_box_array->dirty();

	m_min = min;
	m_max = max;
}