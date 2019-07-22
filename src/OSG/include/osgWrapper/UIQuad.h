#pragma once
#include <osgWrapper/AttributeUtilNode.h>
#include <osgGA\GUIActionAdapter>
#include <osgGA\GUIEventAdapter>
#include <osg\Texture2D>
#include <osgWrapper\ProcessMode.h>

namespace OSGWrapper
{
	class OSG_EXPORT QuadAttributeUtilNode : public AttributeUtilNode
	{
	public:
		QuadAttributeUtilNode(int order);
		virtual ~QuadAttributeUtilNode();

		int Order();
	private:
		int m_order;
	};

	class OSG_EXPORT UIQuad : public osg::Referenced
	{
	public:
		UIQuad();
		virtual ~UIQuad();

		void AddChild(UIQuad* quad);
		void SetEventSpread(bool spread);

		void Resize(float parent_width, float parent_height);
		bool handleFrameEvent(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa);
		bool handleKeyEvent(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa);
		bool handleMouseEvent(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa);
		bool handleDoubleClickEvent(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa);

		virtual QuadAttributeUtilNode* Generate();
		virtual bool HitTest(float x, float y);
	protected:
		virtual void OnDoubleClick();
		virtual void OnUnHover();
		virtual void OnHover();
		virtual void OnPushDown();
		virtual void OnPushUp();
		virtual void OnClick();
	private:
		std::vector<osg::ref_ptr<UIQuad>> m_children_quads;
		bool m_event_spread;

		bool m_capture;
		ProcessMode m_process_mode;

		osg::Vec3f m_left_push_state;
	};
}