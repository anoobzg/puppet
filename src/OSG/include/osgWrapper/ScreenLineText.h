#pragma once
#include <osgWrapper/AttributeUtilNode.h>
#include <osgWrapper/FreetypeFont.h>

namespace OSGWrapper
{
	class OSG_EXPORT ScreenLineText : public AttributeUtilNode
	{
	public:
		ScreenLineText(const std::string& font, int line_count);
		virtual ~ScreenLineText();

		void SetOrigin(const osg::Vec2f& origin);
		void SetWidth(float width);
		void SetHeight(float height);

		void SetText(const std::wstring& text);
	protected:
		osg::ref_ptr<osg::Geometry> m_geometry;
		osg::ref_ptr<osg::Vec2Array> m_texcoord_array;
		osg::ref_ptr<osg::Vec4Array> m_color_array;
		osg::ref_ptr<osg::Uniform> m_origin_uniform;
		osg::ref_ptr<osg::Uniform> m_width_uniform;
		osg::ref_ptr<osg::Uniform> m_height_uniform;

		FreetypeFont* m_font;
		int m_count;
	};
}
