#pragma once
#include <osgWrapper/AttributeUtilNode.h>
#include <osgWrapper/FreetypeFont.h>

namespace OSGWrapper
{
	class OSG_EXPORT ScreenSingleText : public AttributeUtilNode
	{
	public:
		ScreenSingleText();
		virtual ~ScreenSingleText();

		void SetOrigin(const osg::Vec2f& origin);
		void SetWidth(float width);
		void SetHeight(float height);

		void SetFont(const std::string& file);
		void SetText(char c);
		void SetText(wchar_t c);
		void SetColor(const osg::Vec4& color);
	protected:
		osg::ref_ptr<osg::Geometry> m_geometry;
		osg::ref_ptr<osg::Vec2Array> m_texcoord_array;
		osg::ref_ptr<osg::Uniform> m_origin_uniform;
		osg::ref_ptr<osg::Uniform> m_width_uniform;
		osg::ref_ptr<osg::Uniform> m_height_uniform;
		osg::ref_ptr<osg::Uniform> m_color_uniform;

		FreetypeFont* m_font;
	};
}
