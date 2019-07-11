#include <osgWrapper/ScreenLineText.h>
#include <osgWrapper/GeometryCreator.h>
#include <osgWrapper/FreetypeFontManager.h>

namespace OSGWrapper
{
	ScreenLineText::ScreenLineText(const std::string& font, int line_count)
		:m_font(NULL), m_count(line_count)
	{
		SetRenderProgram("screenlinetext");

		m_font = FreetypeFontManager::Instance().Get(font.c_str(), 8);

		if (m_font)
		{
			osg::Texture2D* texture = m_font->GetTexture();
			SetTextureAttribute(0, texture);
		}

		osg::Vec2Array* coord_array = new osg::Vec2Array();
		m_texcoord_array = new osg::Vec2Array();
		m_color_array = new osg::Vec4Array();

		for (int i = 0; i < m_count; ++i)
		{
			coord_array->push_back(osg::Vec2(1.0f * (i), 0.0f));
			coord_array->push_back(osg::Vec2(1.0f * (i + 1), 0.0f));
			coord_array->push_back(osg::Vec2(1.0f * (i + 1), 1.0f));
			coord_array->push_back(osg::Vec2(1.0f * (i), 1.0f));
			m_texcoord_array->push_back(osg::Vec2(0.0f, 0.0f));
			m_texcoord_array->push_back(osg::Vec2(1.0f, 0.0f));
			m_texcoord_array->push_back(osg::Vec2(1.0f, 1.0f));
			m_texcoord_array->push_back(osg::Vec2(0.0f, 1.0f));
			m_color_array->push_back(osg::Vec4(1.0f, 1.0f, 0.0f, 1.0f));
			m_color_array->push_back(osg::Vec4(1.0f, 1.0f, 0.0f, 1.0f));
			m_color_array->push_back(osg::Vec4(1.0f, 1.0f, 0.0f, 1.0f));
			m_color_array->push_back(osg::Vec4(1.0f, 1.0f, 0.0f, 1.0f));
		}


		osg::DrawArrays* draw_array = new osg::DrawArrays(GL_QUADS, 0, 4 * m_count);
		m_geometry = GeometryCreator::CreateIndexAttributeGeometry(draw_array, coord_array, m_texcoord_array, m_color_array);
		m_geometry->setComputeBoundingBoxCallback(new osg::Drawable::ComputeBoundingBoxCallback());
		AddChild(m_geometry);

		m_origin_uniform = new osg::Uniform("origin", osg::Vec2f(0.0f, 0.0f));
		m_width_uniform = new osg::Uniform("width", 100.0f);
		m_height_uniform = new osg::Uniform("height", 100.0f);
		AddUniform(m_origin_uniform);
		AddUniform(m_width_uniform);
		AddUniform(m_height_uniform);
		AddUniform(new osg::Uniform("font", 0));

		SetMode(GL_BLEND, osg::StateAttribute::ON);
	}

	ScreenLineText::~ScreenLineText()
	{

	}

	void ScreenLineText::SetOrigin(const osg::Vec2f& origin)
	{
		m_origin_uniform->set(origin);
	}

	void ScreenLineText::SetWidth(float width)
	{
		m_width_uniform->set(width);
	}

	void ScreenLineText::SetHeight(float height)
	{
		m_height_uniform->set(height);
	}

	void ScreenLineText::SetColor(const osg::Vec4& color)
	{
		for (size_t i = 0; i < m_color_array->size(); ++i)
			m_color_array->at(i) = color;
	}

	void ScreenLineText::SetText(const std::wstring& text)
	{
		if (!m_font) return;
		std::wstring valid_text;
		std::wstring::const_iterator it = text.begin();
		while (valid_text.size() < m_count)
		{
			if (it != text.end())
			{
				valid_text.push_back(*it);
				++it;
			}
			else
				valid_text.push_back(L' ');
		}

		std::vector<osg::Vec2f> coord_array;
		m_font->GenTextureCoord(valid_text, coord_array);
		for (int i = 0; i < 4 * m_count; ++i)
			m_texcoord_array->at(i) = coord_array.at(i);

		m_texcoord_array->dirty();
	}
}