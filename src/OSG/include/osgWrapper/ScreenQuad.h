#pragma once
#include <osgWrapper/AttributeUtilNode.h>
#include <osg/Geometry>

namespace OSGWrapper
{
	class OSG_EXPORT ScreenQuad : public AttributeUtilNode
	{
	public:
		ScreenQuad();
		virtual ~ScreenQuad();

		void SetOrigin(const osg::Vec2f& origin);
		void SetWidth(float width);
		void SetHeight(float height);
		void SetColor(const osg::Vec4f& color);
	protected:
		osg::ref_ptr<osg::Geometry> m_geometry;
		osg::ref_ptr<osg::Uniform> m_origin_uniform;
		osg::ref_ptr<osg::Uniform> m_width_uniform;
		osg::ref_ptr<osg::Uniform> m_height_uniform;
		osg::ref_ptr<osg::Uniform> m_color_uniform;
	};
}