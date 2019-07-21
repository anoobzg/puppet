#include <osgWrapper/UIItem.h>
#include <iostream>

namespace OSGWrapper
{
	UIItem::UIItem()
		:m_process_mode(pm_none)
	{
		m_size = osg::Vec2(0.0f, 0.0f);
		m_offset = osg::Vec2(0.0f, 0.0f);
		m_uniform_size = new osg::Uniform("size", m_size);
		m_uniform_offset = new osg::Uniform("offset", m_offset);
		m_uniform_alpha = new osg::Uniform("alpha", 1.0f);
		AddUniform(m_uniform_offset);
		AddUniform(m_uniform_size);
		AddUniform(m_uniform_alpha);

		AddUniform(new osg::Uniform("texture", 0));
	}

	UIItem::~UIItem()
	{

	}

	void UIItem::SetOffset(const osg::Vec2& offset)
	{
		m_offset = offset;
		m_uniform_offset->set(m_offset);

		UpdateAllChildrenItems();
	}

	void UIItem::SetSize(const osg::Vec2& size)
	{
		m_size = size;
		m_uniform_size->set(m_size);
	}

	void UIItem::AddItem(UIItem* item)
	{
		if (item)
		{
			AddChild(item);
			item->Update(m_offset);
		}
	}

	void UIItem::RemoveItem(UIItem* item)
	{
		RemoveChild(item);
	}

	bool UIItem::CheckIn(const osgGA::GUIEventAdapter& ea)
	{
		float x = ea.getX(); float y = ea.getY();
		return CheckIn(x, y);
	}

	bool UIItem::CheckIn(float x, float y)
	{
		if (x >= m_offset.x() && x <= m_offset.x() + m_size.x() &&
			y >= m_offset.y() && y <= m_offset.y() + m_size.y())
			return true;
		return false;
	}

	bool UIItem::handleFrameEvent(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
	{
		return false;
	}

	bool UIItem::handleKeyEvent(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
	{
		return false;
	}

	bool UIItem::handleMouseEvent(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
	{
		bool child_handled = false;
		unsigned size = getNumChildren();
		for (unsigned i = 0; i < size; ++i)
		{
			UIItem* item = dynamic_cast<UIItem*>(getChild(i));
			if (item)
			{
				child_handled = item->handleMouseEvent(ea, aa);
				if (child_handled) break;
			}
		}

		if (child_handled) return true;

		osgGA::GUIEventAdapter::EventType event_type = ea.getEventType();
		if (ea.getEventType() == osgGA::GUIEventAdapter::PUSH)
		{
			if (CheckIn(ea))
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
						if (CheckIn(ea) &&
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

	bool UIItem::handleDoubleClickEvent(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
	{
		bool child_handled = false;
		unsigned size = getNumChildren();
		for (unsigned i = 0; i < size; ++i)
		{
			UIItem* item = dynamic_cast<UIItem*>(getChild(i));
			if (item)
			{
				child_handled = item->handleDoubleClickEvent(ea, aa);
				if (child_handled) break;
			}
		}

		if (child_handled) return true;

		if (CheckIn(ea))
		{
			OnDoubleClick(ea, aa);
			return true;
		}
		return false;
	}

	void UIItem::OnDoubleClick(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
	{
		std::cout << "UIItem Double Click." << std::endl;
	}

	void UIItem::OnUnHover()
	{
		std::cout << "UIItem UnHover." << std::endl;
	}

	void UIItem::OnHover()
	{
		std::cout << "UIItem Hover." << std::endl;
	}

	void UIItem::OnPushDown()
	{
		std::cout << "UIItem OnPushDown." << std::endl;
	}

	void UIItem::OnPushUp()
	{
		std::cout << "UIItem OnPushUp." << std::endl;
	}

	void UIItem::OnClick()
	{
		std::cout << "UIItem OnClick." << std::endl;
	}

	void UIItem::UpdateAllChildrenItems()
	{
		unsigned size = getNumChildren();
		for (unsigned i = 0; i < size; ++i)
		{
			UIItem* item = dynamic_cast<UIItem*>(getChild(i));
			if (item) item->Update(m_offset);
		}

	}

	void UIItem::Update(const osg::Vec2& offset)
	{
		m_offset += offset;
		m_uniform_offset->set(m_offset);
	}

	void UIItem::UpdateSize()
	{

	}

	UIItem* UIItem::Hover(float x, float y)
	{
		UIItem* rt = 0;
		unsigned size = getNumChildren();
		for (unsigned i = 0; i < size; ++i)
		{
			UIItem* item = dynamic_cast<UIItem*>(getChild(i));
			if (item)
			{
				rt = item->Hover(x, y);
				if (rt) return rt;
			}
		}

		if (CheckIn(x, y))
			return this;
		
		return NULL;
	}

	void UIItem::SetTexture(osg::Texture2D* texture)
	{
		SetTextureAttribute(0, texture);
	}

	void UIItem::SetAlpha(float alpha)
	{
		m_uniform_alpha->set(alpha);
	}
}