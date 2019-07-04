#include <osgWrapper/ScreenDividingRule.h>
#include <osgWrapper/GeometryCreator.h>

namespace OSGWrapper
{
	ScreenDividingRule::ScreenDividingRule()
		:m_divide1(5), m_divide2(5)
	{
		SetRenderProgram("screendividingrule2d");

		m_origin_uniform = new osg::Uniform("origin", osg::Vec2f(0.0f, 0.0f));
		m_width_uniform = new osg::Uniform("width", 100.0f);
		m_height_uniform = new osg::Uniform("height", 100.0f);

		m_divide_uniform = new osg::Uniform("divide", 0.0f);
		m_upper_color_uniform = new osg::Uniform("upper_color", osg::Vec4f(1.0f, 0.0f, 0.0f, 1.0f));
		m_lower_color_uniform = new osg::Uniform("lower_color", osg::Vec4f(0.0f, 1.0f, 0.0f, 1.0f));

		AddUniform(m_origin_uniform);
		AddUniform(m_width_uniform);
		AddUniform(m_height_uniform);
		AddUniform(m_divide_uniform);
		AddUniform(m_upper_color_uniform);
		AddUniform(m_lower_color_uniform);

		Create();
	}

	ScreenDividingRule::~ScreenDividingRule()
	{

	}

	void ScreenDividingRule::SetOrigin(const osg::Vec2f& origin)
	{
		m_origin_uniform->set(origin);
	}

	void ScreenDividingRule::SetWidth(float width)
	{
		m_width_uniform->set(width);
	}

	void ScreenDividingRule::SetHeight(float height)
	{
		m_height_uniform->set(height);
	}

	void ScreenDividingRule::SetDivideRatio(float d)
	{
		m_divide_uniform->set(d);
	}

	void ScreenDividingRule::SetUpperColor(const osg::Vec4f& color)
	{
		m_upper_color_uniform->set(color);
	}

	void ScreenDividingRule::SetLowerColor(const osg::Vec4f& color)
	{
		m_lower_color_uniform->set(color);
	}

	void ScreenDividingRule::Create()
	{
		RemoveAll();

		osg::Geometry* geometry = CreateInner();
		AddChild(geometry);
	}

	void ScreenDividingRule::SetDivide(int divide1, int divide2)
	{
		if (divide1 <= 0 || divide2 <= 0)
			return;
		bool update = (divide1 != m_divide1) || (divide2 != m_divide2);
		m_divide1 = divide1;
		m_divide2 = divide2;

		if (update) Create();
	}

	osg::Geometry* ScreenDividingRule::CreateInner()
	{
		osg::Vec2Array* coord_array = new osg::Vec2Array();
		coord_array->reserve(2 + 2 * (m_divide1 + 1) + 2 * (m_divide1 * m_divide2 + 1));
		//left
		coord_array->push_back(osg::Vec2(0.0f, 0.0f));
		coord_array->push_back(osg::Vec2(0.0f, 1.0f));
		//long
		float ldelta = 1.0f / (float)m_divide1;
		for (int i = 0; i <= m_divide1; ++i)
		{
			float h = (float)i * ldelta;
			coord_array->push_back(osg::Vec2(0.0f, h));
			coord_array->push_back(osg::Vec2(1.0f, h));
		}
		//short
		int num = m_divide1 * m_divide2;
		float sdelta = 1.0f / (float)num;
		for (int i = 0; i <= num; ++i)
		{
			float h = (float)i * sdelta;
			coord_array->push_back(osg::Vec2(0.0f, h));
			coord_array->push_back(osg::Vec2(0.5f, h));
		}
		osg::DrawArrays* draw_array = new osg::DrawArrays(GL_LINES, 0, coord_array->size());
		osg::Geometry* geometry = GeometryCreator::CreateIndexAttributeGeometry(draw_array, coord_array);
		return geometry;
	}
}