#include <osgWrapper\UIQuad.h>
#include <iostream>

namespace OSGWrapper
{
	QuadAttributeUtilNode::QuadAttributeUtilNode(int order)
		:m_order(order)
	{

	}

	QuadAttributeUtilNode::~QuadAttributeUtilNode()
	{

	}

	int QuadAttributeUtilNode::Order()
	{
		return m_order;
	}

	UIQuad::UIQuad()
		:m_event_spread(false)
	{

	}

	UIQuad::~UIQuad()
	{

	}

	void UIQuad::Resize(float parent_width, float parent_height)
	{

	}

	void UIQuad::AddChild(UIQuad* quad)
	{
		m_children_quads.push_back(quad);
	}

	void UIQuad::SetEventSpread(bool spread)
	{
		m_event_spread = spread;
	}

	bool UIQuad::handleFrameEvent(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
	{
		return false;
	}

	bool UIQuad::handleKeyEvent(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
	{
		return false;
	}

	bool UIQuad::handleMouseEvent(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
	{
		bool child_handled = false;
		unsigned size = m_children_quads.size();
		for (unsigned i = 0; i < size; ++i)
		{
			UIQuad* item = m_children_quads.at(i);
			child_handled = item->handleMouseEvent(ea, aa);
			if (child_handled) break;
		}

		if (child_handled && !m_event_spread) return true;

		float x = ea.getX();
		float y = ea.getY();
		osgGA::GUIEventAdapter::EventType event_type = ea.getEventType();
		if (ea.getEventType() == osgGA::GUIEventAdapter::PUSH)
		{
			if (HitTest(x, y))
			{
				m_capture = true;
				if (ea.getButton() == osgGA::GUIEventAdapter::LEFT_MOUSE_BUTTON)
				{
					OnPushDown();
					m_left_push_state.x() = ea.getX();
					m_left_push_state.y() = ea.getY();
					m_left_push_state.z() = (float)ea.getTime();
					m_process_mode = pm_left;
				}
				return true;
			}
			return false;
		}

		if (event_type == osgGA::GUIEventAdapter::RELEASE)
		{
			bool rt = m_capture;

			if (ea.getButton() == osgGA::GUIEventAdapter::LEFT_MOUSE_BUTTON)
			{
				if (m_capture)
				{
					OnPushUp();

					{
						//std::cout << "Click "<<abs(m_left_push_state.z() - (float)ea.getTime()) << std::endl;
						if (HitTest(x, y) &&
							abs(m_left_push_state.z() - (float)ea.getTime()) < 0.2f)
							OnClick();
					}
					m_process_mode = pm_none;
				}
			}

			if (ea.getButtonMask() == 0)
			{
				m_capture = false;
			}
			return rt;
		}

		if (event_type == osgGA::GUIEventAdapter::DRAG)
		{
			if (m_capture) return true;
			else return false;
		}

		return false;
	}

	bool UIQuad::handleDoubleClickEvent(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
	{
		bool child_handled = false;
		unsigned size = m_children_quads.size();
		for (unsigned i = 0; i < size; ++i)
		{
			UIQuad* item = m_children_quads.at(i);
			child_handled = item->handleDoubleClickEvent(ea, aa);
			if (child_handled) break;
		}

		if (child_handled && !m_event_spread) return true;

		if (HitTest(ea.getX(), ea.getY()))
		{
			OnDoubleClick();
			return true;
		}
		return false;
	}


	QuadAttributeUtilNode* UIQuad::Generate()
	{
		return 0;
	}

	OSGWrapper::UIQuad* UIQuad::HitTest(float x, float y)
	{
		unsigned size = m_children_quads.size();
		for (unsigned i = 0; i < size; ++i)
		{
			UIQuad* item = m_children_quads.at(i);
			OSGWrapper::UIQuad* hit = item->HitTest(x, y);
			if (hit) return hit;
		}
		return 0;
	}

	void UIQuad::OnDoubleClick()
	{
		std::cout << "OnDoubleClick." << std::endl;
	}

	void UIQuad::OnUnHover()
	{
		std::cout << "OnUnHover." << std::endl;
	}

	void UIQuad::OnHover()
	{
		std::cout << "OnHover." << std::endl;
	}

	void UIQuad::OnPushDown()
	{
		std::cout << "OnPushDown." << std::endl;
	}

	void UIQuad::OnPushUp()
	{
		std::cout << "OnPushUp." << std::endl;
	}

	void UIQuad::OnClick()
	{
		std::cout << "OnClick." << std::endl;
	}
}