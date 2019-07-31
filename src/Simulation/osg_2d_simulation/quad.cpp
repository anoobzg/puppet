#include "quad.h"
#include <osgWrapper\UtilCreator.h>

Quad::Quad()
{
	m_node = new OSGWrapper::QuadAttributeUtilNode(1);
	m_geometry = OSGWrapper::UtilCreator::CreateUnitQuad();
	m_node->AddChild(m_geometry);
}

Quad::~Quad()
{

}

void Quad::SetOffset(const osg::Vec2f& offset)
{
	m_offset = offset;
	UpdateGeometry();
}

void Quad::SetSize(const osg::Vec2f& size)
{
	m_size = size;
	UpdateGeometry();
}

void Quad::SetRect(const osg::Vec2f& offset, const osg::Vec2f& size)
{
	m_offset = offset;
	m_size = size;
	UpdateGeometry();
}

void Quad::UpdateGeometry()
{
	osg::Vec2Array* coord_array = dynamic_cast<osg::Vec2Array*>(m_geometry->getVertexAttribArray(0));
	if (coord_array)
	{
		coord_array->at(0) = m_offset;
		coord_array->at(1) = m_offset + osg::Vec2f(m_size.x(), 0.0f);
		coord_array->at(2) = m_offset + osg::Vec2f(m_size.x(), m_size.y());
		coord_array->at(3) = m_offset + osg::Vec2f(0.0f, m_size.y());
		coord_array->dirty();
	}
}

OSGWrapper::QuadAttributeUtilNode* Quad::Generate()
{
	return m_node;
}

OSGWrapper::UIQuad* Quad::HitTest(float x, float y)
{
	OSGWrapper::UIQuad* q = OSGWrapper::UIQuad::HitTest(x, y);
	if (q) return q;

	if (x >= m_offset.x() && x <= m_offset.x() + m_size.x() &&
		y >= m_offset.y() && y <= m_offset.y() + m_size.y())
		return this;
	return 0;
}