#include <osgWrapper/ScreenSingleText.h>
#include <osgWrapper/GeometryCreator.h>
#include <osgWrapper/FreetypeFontManager.h>

namespace OSGWrapper
{
	ScreenSingleText::ScreenSingleText()
		:m_font(NULL)
	{
		SetRenderProgram("screensingletext");

		osg::Vec2Array* coord_array = new osg::Vec2Array();
		m_texcoord_array = new osg::Vec2Array();
		coord_array->push_back(osg::Vec2(0.0f, 0.0f));
		coord_array->push_back(osg::Vec2(1.0f, 0.0f));
		coord_array->push_back(osg::Vec2(1.0f, 1.0f));
		coord_array->push_back(osg::Vec2(0.0f, 1.0f));
		m_texcoord_array->push_back(osg::Vec2(0.0f, 0.0f));
		m_texcoord_array->push_back(osg::Vec2(1.0f, 0.0f));
		m_texcoord_array->push_back(osg::Vec2(1.0f, 1.0f));
		m_texcoord_array->push_back(osg::Vec2(0.0f, 1.0f));

		osg::DrawArrays* draw_array = new osg::DrawArrays(GL_QUADS, 0, 4);
		m_geometry = GeometryCreator::CreateIndexAttributeGeometry(draw_array, coord_array, m_texcoord_array);
		AddChild(m_geometry);

		m_origin_uniform = new osg::Uniform("origin", osg::Vec2f(0.0f, 0.0f));
		m_width_uniform = new osg::Uniform("width", 100.0f);
		m_height_uniform = new osg::Uniform("height", 100.0f);
		m_color_uniform = new osg::Uniform("color", osg::Vec4f(1.0f, 1.0f, 1.0f, 1.0f));
		AddUniform(m_origin_uniform);
		AddUniform(m_width_uniform);
		AddUniform(m_height_uniform);
		AddUniform(m_color_uniform);
		AddUniform(new osg::Uniform("font", 0));


		SetMode(GL_BLEND, osg::StateAttribute::ON);
	}

	ScreenSingleText::~ScreenSingleText()
	{

	}

	void ScreenSingleText::SetOrigin(const osg::Vec2f& origin)
	{
		m_origin_uniform->set(origin);
	}

	void ScreenSingleText::SetWidth(float width)
	{
		m_width_uniform->set(width);
	}

	void ScreenSingleText::SetHeight(float height)
	{
		m_height_uniform->set(height);
	}

	void ScreenSingleText::SetColor(const osg::Vec4& color)
	{
		m_color_uniform->set(color);
	}

	void ScreenSingleText::SetFont(const std::string& file)
	{
		m_font = FreetypeFontManager::Instance().Get(file.c_str(), 32);
		
		if (m_font)
		{
			osg::Texture2D* texture = m_font->GetTexture();
			SetTextureAttribute(0, texture);
		}
	}

	void ScreenSingleText::SetText(wchar_t c)
	{
		if (m_font)
		{
			std::wstring text; text.push_back(c);
			std::vector<osg::Vec2f> texcoord;
			m_font->GenTextureCoord(text, texcoord);
			m_texcoord_array->clear();
			m_texcoord_array->push_back(texcoord.at(0));
			m_texcoord_array->push_back(texcoord.at(1));
			m_texcoord_array->push_back(texcoord.at(2));
			m_texcoord_array->push_back(texcoord.at(3));
		}
	}
	void ScreenSingleText::SetText(char c)
	{
		if (m_font)
		{
			std::string text; text.push_back(c);
			std::vector<osg::Vec2f> texcoord;
			m_font->GenTextureCoord(text, texcoord);
			m_texcoord_array->clear();
			m_texcoord_array->push_back(texcoord.at(0));
			m_texcoord_array->push_back(texcoord.at(1));
			m_texcoord_array->push_back(texcoord.at(2));
			m_texcoord_array->push_back(texcoord.at(3));
		}
	}
}