#pragma once
#include <osgWrapper/AttributeUtilNode.h>
#include <osgWrapper/ScreenQuad.h>
#include <osgWrapper/ScreenDividingRule.h>

namespace OSGWrapper
{
	class OSG_EXPORT ScreenDistanceIndicator : public AttributeUtilNode
	{
	public:
		ScreenDistanceIndicator();
		virtual ~ScreenDistanceIndicator();

		void SetOrigin(const osg::Vec2f& origin);
		void SetQuadWidth(float width);
		void SetDividerWidth(float width);
		void SetQDGap(float gap);
		void SetHeight(float height);
		void SetCurrentValue(float value);

		void SetActiveColor(const osg::Vec4f& color);
		void SetDefaultColor(const osg::Vec4f& color);
	private:
		void Update();
	protected:
		osg::ref_ptr<ScreenQuad> m_quad;
		osg::ref_ptr<ScreenDividingRule> m_rule;

		float m_height;
		float m_quad_width;
		float m_divider_width;
		float m_quad_divider_gap;

		float m_min_value;
		float m_max_value;
		float m_current_value;

		osg::Vec2f m_offset;
	};
}