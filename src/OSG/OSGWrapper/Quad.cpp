#include <osgWrapper/Quad.h>
#include <osgWrapper\UtilCreator.h>
#include <osgWrapper\TextureManager.h>

namespace OSGWrapper
{

	Quad::Quad()
		:m_use_texture(true)
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

	void Quad::SetColor(const osg::Vec4f& color)
	{
		osg::Vec4Array* color_array = dynamic_cast<osg::Vec4Array*>(m_geometry->getVertexAttribArray(2));
		if (color_array)
		{
			color_array->at(0) = color;
			color_array->at(1) = color;
			color_array->at(2) = color;
			color_array->at(3) = color;
			color_array->dirty();
		}
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

	void Quad::SetTexCoord(UnionCoord* coord)
	{
		if (!coord) return;

		osg::Vec2Array* coord_array = dynamic_cast<osg::Vec2Array*>(m_geometry->getVertexAttribArray(1));
		if (coord_array)
		{
			osg::Vec2 dmin = coord->dmin;
			osg::Vec2 dmax = coord->dmax;

			coord_array->at(0) = dmin;
			coord_array->at(1) = osg::Vec2f(dmax.x(), dmin.y());
			coord_array->at(2) = dmax;
			coord_array->at(3) = osg::Vec2f(dmin.x(), dmax.y());
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

	void Quad::SetUseTexture(bool use)
	{
		m_use_texture = use;
		m_node->SetRenderProgram("");
		if (!m_use_texture) m_node->SetRenderProgram("screenui");
	}

	void Quad::SetTexture(const char* name)
	{

	}

	void Quad::SetTexture(osg::Texture2D* tex)
	{
		m_node->SetTextureAttribute(0, tex);
	}
}