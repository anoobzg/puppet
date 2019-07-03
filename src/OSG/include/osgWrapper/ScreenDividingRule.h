#pragma once
#include <osgWrapper/AttributeUtilNode.h>

namespace OSGWrapper
{
	class OSG_EXPORT ScreenDividingRule : public AttributeUtilNode
	{
	public:
		ScreenDividingRule();
		virtual ~ScreenDividingRule();

		void SetOrigin(const osg::Vec2f& origin);
		void SetWidth(float width);
		void SetHeight(float height);

		void SetDivide(int divide1, int divide2);
		void SetDivideRatio(float d);
	private:
		void Create();
		osg::Geometry* CreateInner();
	protected:
		osg::ref_ptr<osg::Uniform> m_origin_uniform;
		osg::ref_ptr<osg::Uniform> m_width_uniform;
		osg::ref_ptr<osg::Uniform> m_height_uniform;

		int m_divide1;
		int m_divide2;

		osg::ref_ptr<osg::Uniform> m_upper_color_uniform;
		osg::ref_ptr<osg::Uniform> m_lower_color_uniform;
		osg::ref_ptr<osg::Uniform> m_divide_uniform;
	};
}