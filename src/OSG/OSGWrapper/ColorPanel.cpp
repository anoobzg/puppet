#include <osgWrapper/ColorPanel.h>
#include <osgWrapper/GeometryCreator.h>
#include <osgWrapper/ArrayCreator.h>
#include <osg/PolygonMode>

namespace OSGWrapper
{
	ColorPanel::ColorPanel(float h, float s, float v)
		:m_width(400.0f), m_mwidth(350.0f), m_height(350.0f), m_origin(osg::Vec2f(0.0f, 0.0f))
		, m_H(h), m_S(s), m_V(v), m_alpha(0.1f), m_capture(false), m_capture_v(false)
	{
		SetRenderProgram("colorpanel");

		m_origin_uniform = new osg::Uniform("origin", m_origin);

		m_geometry = Create();
		CreateColorBar();
		UpdateColorBar(m_H, m_S);

		AddChild(m_geometry);
		AddUniform(m_origin_uniform);

		SetMode(GL_DEPTH_TEST, osg::StateAttribute::OFF);
		//SetAttribute(new osg::PolygonMode(osg::PolygonMode::FRONT_AND_BACK, osg::PolygonMode::LINE));

		CreateIndicator();
		UpdateIndicator(m_H, m_S, m_V);
	}

	ColorPanel::~ColorPanel()
	{
	}

	void ColorPanel::CreateColorBar()
	{
		osg::Vec2Array* coord_array = new osg::Vec2Array();
		m_color_array = new osg::Vec4Array();
		int n = 100;
		int tn = 2 * (n + 1);
		coord_array->resize(tn);
		m_color_array->resize(tn);
		float dy = m_height / (float)n;
		for (int i = 0; i <= n; ++i)
		{
			int index1 = i;
			int index2 = i + n + 1;
			float x1 = m_mwidth; float x2 = m_width;
			float y = (float)i * dy;
			coord_array->at(index1) = osg::Vec2f(x1, y); coord_array->at(index2) = osg::Vec2f(x2, y);
			m_color_array->at(index1) = osg::Vec4f(1.0f, 1.0f, 1.0f, 1.0f);
			m_color_array->at(index2) = osg::Vec4f(1.0f, 1.0f, 1.0f, 1.0f);
		}

		int pn = n;
		osg::DrawElementsUInt* draw_array = new osg::DrawElementsUInt(GL_QUADS);
		draw_array->resize(4 * pn);

		int index = 0;
		for (int i = 0; i < n; ++i)
		{
			int tindex = i;
			draw_array->at(index++) = tindex;
			draw_array->at(index++) = tindex + n + 1;
			draw_array->at(index++) = tindex + n + 2;
			draw_array->at(index++) = tindex + 1;
		}
		m_color_bar = GeometryCreator::CreateIndexAttributeGeometry(draw_array, coord_array, m_color_array);
		AddChild(m_color_bar);
	}

	void ColorPanel::UpdateColorBar(float H, float S)
	{
		int n = 100;
		float dy = 1.0f / (float)n;
		for (int i = 0; i <= n; ++i)
		{
			int index1 = i;
			int index2 = i + n + 1;
			float y = (float)i * dy;
			osg::Vec4f color = HSVColor(H, S, y);
			m_color_array->at(index1) = color;
			m_color_array->at(index2) = color;
		}

		m_color_array->dirty();
	}

	osg::Geometry* ColorPanel::Create()
	{
		osg::Vec2Array* coord_array = new osg::Vec2Array();
		osg::Vec4Array* color_array = new osg::Vec4Array();
		int n = 100;
		int tn = (n + 1) * (n + 1);
		coord_array->resize(tn);
		color_array->resize(tn);
		float dx = m_mwidth / (float)n;
		float dy = m_height / (float)n;
		for (int i = 0; i <= n; ++i)
		{
			for (int j = 0; j <= n; ++j)
			{
				int index = i * (n + 1) + j;
				float x = (float)i * dx;
				float y = (float)j * dy;
				coord_array->at(index) = osg::Vec2f(x, y);
				color_array->at(index) = GetHSColor(x, y);
			}
		}

		int pn = n * n;
		osg::DrawElementsUInt* draw_array = new osg::DrawElementsUInt(GL_QUADS);
		draw_array->resize(4 * pn);

		int index = 0;
		for (int i = 0; i < n; ++i)
		{
			for (int j = 0; j < n; ++j)
			{
				int tindex = i * (n + 1) + j;
				draw_array->at(index++) = tindex;
				draw_array->at(index++) = tindex + n + 1;
				draw_array->at(index++) = tindex + n + 2;
				draw_array->at(index++) = tindex + 1;
			}
		}
		return GeometryCreator::CreateIndexAttributeGeometry(draw_array, coord_array, color_array);
	}

	void ColorPanel::SetOrigin(const osg::Vec2f& origin)
	{
		m_origin_uniform->set(origin);
		m_origin = origin;
	}

	void ColorPanel::SetColorUniform(osg::Uniform* uniform)
	{
		m_color_uniform = uniform;
		Notify();
	}

	osg::Vec4f ColorPanel::GetHSColor(float x, float y)
	{
		float dx = (x) / m_mwidth * 360.0f;
		float dy = (y) / m_height;
		return HSVColor(dx, dy, 1.0f);
	}

	osg::Vec4f ColorPanel::GetHSVColor(float x, float y, float z)
	{
		float dx = (x) / m_mwidth * 360.0f;
		float dy = (y) / m_height;
		float dz = (z) / m_height;
		return HSVColor(dx, dy, dz);
	}

	osg::Vec4f ColorPanel::HSVColor(float H, float S, float V)
	{
		if (H < 0.0f) H = 0.0f;
		if (H >= 360.0f) H = 360.0f;
		if (S < 0.0f) S = 0.0f;
		if (S > 1.0f) S = 1.0f;
		if (V < 0.0f) V = 0.0f;
		if (V > 1.0f) V = 1.0f;

		if (S == 0.0f) return osg::Vec4f(V, V, V, 1.0f);
		float h = H / 60.0f;
		float i = (int)floorf(h);
		float f = h - (float)i;
		float p = V * (1.0f - S);
		float q = V * (1.0f - S * f);
		float t = V * (1.0f - S * (1.0f - f));
		if (i == 0)
		{
			return osg::Vec4f(V, t, p, 1.0f);
		}
		else if (i == 1)
		{
			return osg::Vec4f(q, V, p, 1.0f);
		}
		else if (i == 2)
		{
			return osg::Vec4f(p, V, t, 1.0f);
		}
		else if (i == 3)
		{
			return osg::Vec4f(p, q, V, 1.0f);
		}
		else if (i == 4)
		{
			return osg::Vec4f(t, p, V, 1.0f);
		}
		else
		{
			return osg::Vec4f(V, p, q, 1.0f);
		}
	}

	void ColorPanel::UpdateNormalized(float h, float s, float v)
	{
		m_H = h; m_S = s; m_V = v;
		UpdateColorBar(m_H, m_S);
		UpdateIndicator(m_H, m_S, m_V);
		Notify();
	}

	void ColorPanel::UpdateHS(float x, float y)
	{
		float h = (x - m_origin.x()) * 360.0f / m_mwidth;
		float s = (y - m_origin.y()) / m_height;

		if (h < 0.0f) h = 0.0f;
		if (h >= 360.0f) h = 360.0f;
		if (s < 0.0f) s = 0.0f;
		if (s > 1.0f) s = 1.0f;

		UpdateNormalized(h, s, m_V);
	}

	void ColorPanel::UpdateV(float z)
	{
		float v = (z - m_origin.y()) / m_height;

		if (v < 0.0f) v = 0.0f;
		if (v > 1.0f) v = 1.0f;

		UpdateNormalized(m_H, m_S, v);
	}

	void ColorPanel::CreateIndicator()
	{
		m_indicator_coord = new osg::Vec2Array();
		m_indicator_coord->resize(6, osg::Vec2f(0.0f, 0.0f));
		osg::Vec4Array* color_array = new osg::Vec4Array();
		color_array->resize(6, osg::Vec4f(0.0f, 0.0f, 0.0f, 1.0f));
		osg::DrawArrays* draw_array = new osg::DrawArrays(GL_LINES, 0, m_indicator_coord->size());
		m_indicator =  GeometryCreator::CreateIndexAttributeGeometry(draw_array, m_indicator_coord, color_array);
		AddChild(m_indicator);
	}

	void ColorPanel::UpdateIndicator(float H, float S, float V)
	{
		float x = H * m_mwidth / 360.0f;
		float y = S * m_height;
		float z = V * m_height;

		float d = 10.0f;
		m_indicator_coord->at(0) = osg::Vec2f(x - d, y);
		m_indicator_coord->at(1) = osg::Vec2f(x + d, y);
		m_indicator_coord->at(2) = osg::Vec2f(x, y - d);
		m_indicator_coord->at(3) = osg::Vec2f(x, y + d);
		m_indicator_coord->at(4) = osg::Vec2f(m_mwidth, z);
		m_indicator_coord->at(5) = osg::Vec2f(m_width, z);
		m_indicator_coord->dirty();
	}

	void ColorPanel::Notify()
	{
		if (m_color_uniform)
		{
			osg::Vec4f color = HSVColor(m_H, m_S, m_V);
			color.a() = m_alpha;
			m_color_uniform->set(color);
		}
	}

	void ColorPanel::SetAlpha(float alpha)
	{
		m_alpha = alpha;
	}

	bool ColorPanel::OnMouse(const osgGA::GUIEventAdapter& ea)
	{
		osgGA::GUIEventAdapter::EventType event_type = ea.getEventType();

		if (event_type == osgGA::GUIEventAdapter::RELEASE && ea.getButtonMask() == 0)
		{
			if (m_capture)
			{
				m_capture = false;
				m_capture_v = false;
				return true;
			}
			return false;
		}
		else if (ea.getEventType() == osgGA::GUIEventAdapter::PUSH && ea.getButton() == osgGA::GUIEventAdapter::LEFT_MOUSE_BUTTON)
		{
			if (ea.getX() >= m_origin.x() && ea.getX() <= m_origin.x() + m_width
				&& ea.getY() >= m_origin.y() && ea.getY() <= m_origin.y() + m_height)
			{
				m_capture = true;
				if (ea.getX() <= m_mwidth)
				{
					m_capture_v = false;
					UpdateHS(ea.getX(), ea.getY());
				}
				else
				{
					m_capture_v = true;
					UpdateV(ea.getY());
				}
				return true;
			}
			return false;
		}
		else if (event_type == osgGA::GUIEventAdapter::DRAG && ea.getButtonMask() & osgGA::GUIEventAdapter::LEFT_MOUSE_BUTTON)
		{
			if (m_capture)
			{
				if (!m_capture_v)
				{
					UpdateHS(ea.getX(), ea.getY());
				}
				else
				{
					UpdateV(ea.getY());
				}
				return true;
			}
			return false;
		}

		return false;
	}
}