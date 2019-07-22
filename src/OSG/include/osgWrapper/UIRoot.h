#pragma once
#include <osgWrapper\AttributeUtilNode.h>
#include <osg\Camera>
#include <osgGA\GUIEventAdapter>
#include <osgGA\GUIActionAdapter>
#include <osgWrapper\UIQuad.h>

namespace OSGWrapper
{
	class OSG_EXPORT UIRoot : public osg::Camera
	{
	public:
		UIRoot();
		~UIRoot();

		void Resize(float width, float height);

		bool handleFrameEvent(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa);
		bool handleKeyEvent(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa);
		bool handleMouseEvent(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa);
		bool handleDoubleClickEvent(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa);

		void AddQuad(UIQuad* quad, bool update = false);
		void RemoveQuad(UIQuad* quad, bool update = false);
		void Rebuild();
	protected:
		std::vector<osg::ref_ptr<UIQuad>> m_quads;

		osg::ref_ptr<osg::Uniform> m_width;
		osg::ref_ptr<osg::Uniform> m_height;
	};
}