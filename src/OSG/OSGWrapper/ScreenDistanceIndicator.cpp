#include "osgWrapper/ScreenDistanceIndicator.h"

namespace OSGWrapper
{
	ScreenDistanceIndicator::ScreenDistanceIndicator()
		:m_divider_width(30.0f), m_quad_width(80.0f), m_quad_divider_gap(5.0f)
		,m_height(200.0f), m_min_value(30.0f), m_max_value(100.0f)
		, m_offset(400.0f, 100.0f), m_current_value(m_min_value)
	{
		m_quad = new ScreenQuad();
		m_rule = new ScreenDividingRule();

		AddChild(m_quad);
		AddChild(m_rule);
		Update();
	}

	ScreenDistanceIndicator::~ScreenDistanceIndicator()
	{

	}

	void ScreenDistanceIndicator::SetOrigin(const osg::Vec2f& origin)
	{
		m_offset = origin;
		Update();
	}

	void ScreenDistanceIndicator::SetQuadWidth(float width)
	{
		m_quad_width = width;
		Update();
	}

	void ScreenDistanceIndicator::SetDividerWidth(float width)
	{
		m_divider_width = width;
		Update();
	}

	void ScreenDistanceIndicator::SetQDGap(float gap)
	{
		m_quad_divider_gap = gap;
		Update();
	}

	void ScreenDistanceIndicator::SetHeight(float height)
	{
		m_height = height;
		Update();
	}

	void ScreenDistanceIndicator::SetCurrentValue(float value)
	{
		m_current_value = value;
		Update();
	}

	void ScreenDistanceIndicator::Update()
	{
		float ratio = (m_current_value - m_min_value) / (m_max_value - m_min_value);
		float current_height = m_height * ratio;
		osg::Vec2f rule_offset = m_offset;
		m_rule->SetHeight(m_height);
		m_rule->SetWidth(m_divider_width);
		m_rule->SetDivideRatio(ratio);
		m_rule->SetOrigin(rule_offset);

		osg::Vec2f quad_offset = rule_offset + osg::Vec2f(m_divider_width + m_quad_divider_gap, 0.0f);
		m_quad->SetHeight(current_height);
		m_quad->SetWidth(m_quad_width);
		m_quad->SetOrigin(quad_offset);
	}

	void ScreenDistanceIndicator::SetActiveColor(const osg::Vec4f& color)
	{
		m_quad->SetColor(color);
		m_rule->SetLowerColor(color);
	}

	void ScreenDistanceIndicator::SetDefaultColor(const osg::Vec4f& color)
	{
		m_rule->SetUpperColor(color);
	}
}