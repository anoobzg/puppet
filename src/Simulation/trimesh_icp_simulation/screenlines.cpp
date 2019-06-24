#include "screenlines.h"

ScreenLines::ScreenLines()
{
	m_lines_array = new osg::Vec2Array();
	m_draw_array = new osg::DrawArrays(GL_LINE_STRIP);

	setVertexAttribArray(0, m_lines_array, osg::Array::BIND_PER_VERTEX);
	addPrimitiveSet(m_draw_array);
	setCullingActive(false);
	setUseDisplayList(false);
	setUseVertexBufferObjects(true);
	setComputeBoundingBoxCallback(new osg::Drawable::ComputeBoundingBoxCallback());
}

ScreenLines::~ScreenLines()
{

}

void ScreenLines::Clear()
{
	m_lines_array->clear();
	m_draw_array->setCount(0);
	m_lines_array->dirty();
	m_draw_array->dirty();
}

void ScreenLines::Add(const osg::Vec2& p)
{
	m_lines_array->push_back(p);
	m_draw_array->set(GL_LINE_STRIP, 0, m_lines_array->size());
	m_lines_array->dirty();
	m_draw_array->dirty();
}