#pragma once
#include <osgWrapper/AttributeUtilNode.h>
#include <osgGA\GUIActionAdapter>
#include <osgGA\GUIEventAdapter>
#include <osg\Texture2D>
#include <osgWrapper\ProcessMode.h>

namespace OSGWrapper
{
	class OSG_EXPORT UIItem : public AttributeUtilNode
	{
		friend class UIPanel;
	public:
		UIItem();
		virtual ~UIItem();

		void SetOffset(const osg::Vec2& offset);
		void SetSize(const osg::Vec2& size);

		void AddItem(UIItem* item);
		void RemoveItem(UIItem* item);
		void Update(const osg::Vec2& offset);

		bool handleFrameEvent(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa);
		bool handleKeyEvent(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa);
		bool handleMouseEvent(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa);
		bool handleDoubleClickEvent(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa);
		UIItem* Hover(float x, float y);
		void SetTexture(osg::Texture2D* texture);
		void SetAlpha(float alpha);
	private:
		bool CheckIn(const osgGA::GUIEventAdapter& ea);
		bool CheckIn(float x, float y);

		virtual void OnDoubleClick(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa);
		virtual void OnUnHover();
		virtual void OnHover();
		virtual void OnPushDown();
		virtual void OnPushUp();
		virtual void OnClick();

		void UpdateAllChildrenItems();
		void UpdateSize();
	private:
		osg::Vec2 m_offset;
		osg::Vec2 m_size;
		osg::ref_ptr<osg::Uniform> m_uniform_offset;
		osg::ref_ptr<osg::Uniform> m_uniform_size;
		osg::ref_ptr<osg::Uniform> m_uniform_alpha;

		bool m_capture;
		ProcessMode m_process_mode;

		osg::Vec3f m_left_push_state;
	};
}