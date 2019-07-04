#include <osgWrapper/ScreenQuad.h>
#include <osgWrapper/GeometryCreator.h>

namespace OSGWrapper
{
	ScreenQuad::ScreenQuad()
	{
		SetRenderProgram("screenquad2d");

		osg::Vec2Array* coord_array = new osg::Vec2Array();
		coord_array->push_back(osg::Vec2(0.0f, 0.0f));
		coord_array->push_back(osg::Vec2(1.0f, 0.0f));
		coord_array->push_back(osg::Vec2(1.0f, 1.0f));
		coord_array->push_back(osg::Vec2(0.0f, 1.0f));
		osg::DrawArrays* draw_array = new osg::DrawArrays(GL_QUADS, 0, 4);
		m_geometry = GeometryCreator::CreateIndexAttributeGeometry(draw_array, coord_array);
		AddChild(m_geometry);

		m_origin_uniform = new osg::Uniform("origin", osg::Vec2f(0.0f, 0.0f));
		m_width_uniform = new osg::Uniform("width", 100.0f);
		m_height_uniform = new osg::Uniform("height", 100.0f);
		m_color_uniform = new osg::Uniform("color", osg::Vec4f(0.2f, 0.2f, 0.2f, 1.0f));

		AddUniform(m_color_uniform);
		AddUniform(m_origin_uniform);
		AddUniform(m_width_uniform);
		AddUniform(m_height_uniform);
	}

	ScreenQuad::~ScreenQuad()
	{

	}

	void ScreenQuad::SetOrigin(const osg::Vec2f& origin)
	{
		m_origin_uniform->set(origin);
	}

	void ScreenQuad::SetWidth(float width)
	{
		m_width_uniform->set(width);
	}

	void ScreenQuad::SetHeight(float height)
	{
		m_height_uniform->set(height);
	}

	void ScreenQuad::SetColor(const osg::Vec4f& color)
	{
		m_color_uniform->set(color);
	}
}