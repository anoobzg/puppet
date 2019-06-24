#include "screengraph.h"

ScreenGraph::ScreenGraph()
	:m_step_total_count(40), m_min_error(0.0f), m_max_error(0.1f)
{
	setComputeBoundingSphereCallback(new osg::Node::ComputeBoundingSphereCallback());
	SetRenderProgram("viewport");

	m_box = new ScreenBoundingBox();
	m_lines = new ScreenLines();
	m_box->Update(osg::Vec2(0.0f, 0.0f), osg::Vec2(400.0f, 200.0f));
	AddChild(m_box);
	AddChild(m_lines);
}

ScreenGraph::~ScreenGraph()
{

}

void ScreenGraph::Clear()
{
	m_values.clear();
	m_lines->Clear();
}

void ScreenGraph::AddError(float value)
{
	m_values.push_back(value);

	osg::Vec2f bmin = m_box->GetMin();
	osg::Vec2f bmax = m_box->GetMax();
	osg::Vec2f size = bmax - bmin;

	if (size.x() == 0.0f || size.y() == 0.0f)
		return;

	float x = size.x() * (float)m_values.size() / (float)m_step_total_count;
	float y = size.y() * (value - m_min_error) / (m_max_error - m_min_error);
	m_lines->Add(osg::Vec2f(x, y) + bmin);
}