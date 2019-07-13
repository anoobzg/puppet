#pragma once
#include <osgWrapper/AttributeUtilNode.h>
#include <osg/Geometry>
#include <osgGA/GUIEventAdapter>

namespace OSGWrapper
{
	class OSG_EXPORT ColorPanel : public AttributeUtilNode
	{
	public:
		ColorPanel(float h, float s, float v);
		virtual ~ColorPanel();

		void SetOrigin(const osg::Vec2f& origin);
		void SetColorUniform(osg::Uniform* uniform);
		void SetAlpha(float alpha);

		bool OnMouse(const osgGA::GUIEventAdapter& ea);
	protected:
		osg::Geometry* Create();
		void CreateColorBar();
		void UpdateColorBar(float H, float S);
		void CreateIndicator();
		void UpdateIndicator(float H, float S, float V);

		osg::Vec4f GetHSColor(float x, float y);
		osg::Vec4f GetHSVColor(float x, float y, float z);

		osg::Vec4f HSVColor(float x, float y, float z);

		void UpdateNormalized(float h, float s, float v);
		void UpdateHS(float x, float y);
		void UpdateV(float z);
		void Notify();
	protected:
		const float m_width;
		const float m_height;
		const float m_mwidth;

		osg::Vec2f m_origin;
		osg::ref_ptr<osg::Uniform> m_origin_uniform;
		osg::ref_ptr<osg::Uniform> m_color_uniform;

		osg::ref_ptr<osg::Geometry> m_geometry;
		osg::ref_ptr<osg::Geometry> m_color_bar;
		osg::ref_ptr<osg::Vec4Array> m_color_array;

		float m_H;
		float m_S;
		float m_V;

		osg::ref_ptr<osg::Geometry> m_indicator;
		osg::ref_ptr<osg::Vec2Array> m_indicator_coord;

		float m_alpha;

		bool m_capture;
		bool m_capture_v;
	};
}