#pragma once
#include <osgWrapper/AttributeUtilNode.h>
#include <osgWrapper/UIItem.h>
#include <osg\Viewport>
#include <osg\Camera>
#include <osgGA\GUIEventAdapter>
#include <osgGA\GUIActionAdapter>

namespace OSGWrapper
{
	class OSG_EXPORT UIPanel : public  osg::Camera
	{
	public:
		UIPanel();
		~UIPanel();

		void SetGlobalUI(UIItem* item);
		void AddUI(UIItem* item);
		void RemoveUI(UIItem* item);

		void Resize(float width, float height);

		bool handleFrameEvent(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa);
		bool handleKeyEvent(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa);
		bool handleMouseEvent(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa);
		bool handleDoubleClickEvent(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa);
	protected:
		void ProcessHover(float x, float y);
	private:
		osg::ref_ptr<UIItem> m_global_item;
		std::vector<osg::ref_ptr<UIItem>> m_local_items;

		osg::ref_ptr<osg::Uniform> m_width;
		osg::ref_ptr<osg::Uniform> m_height;

		osg::ref_ptr<osg::Viewport> m_viewport;

		osg::ref_ptr<UIItem> m_hover_item;
	};
}