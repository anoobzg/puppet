#include <osgWrapper/UIPanel.h>
#include <osgWrapper\ProgramManager.h>

namespace OSGWrapper
{
	UIPanel::UIPanel()
	{
		getOrCreateStateSet()->setAttribute(ProgramManager::Instance().Get("ui"), osg::StateAttribute::ON);
		//SetRenderProgram("ui");

		m_width = new osg::Uniform("viewport_width", 1920.0f);
		m_height = new osg::Uniform("viewport_height", 1080.0f);
		//AddUniform(m_width);
		//AddUniform(m_height);
		//
		//SetMode(GL_DEPTH_TEST, osg::StateAttribute::OFF);
		getOrCreateStateSet()->addUniform(m_width, osg::StateAttribute::ON);
		getOrCreateStateSet()->addUniform(m_height, osg::StateAttribute::ON);
		getOrCreateStateSet()->setMode(GL_DEPTH_TEST, osg::StateAttribute::OFF);
		getOrCreateStateSet()->setMode(GL_BLEND, osg::StateAttribute::ON);

		//m_viewport = new osg::Viewport();
		//m_viewport->x() = 0.0;
		//m_viewport->y() = 0.0;
		//m_viewport->width() = 1920.0;
		//m_viewport->height() = 1080.0;
		//getOrCreateStateSet()->setAttribute(m_viewport, osg::StateAttribute::ON);

		setReferenceFrame(osg::Transform::RELATIVE_RF);
		setRenderOrder(osg::Camera::POST_RENDER, 0);
		setClearMask(0);
		//setClearColor(osg::Vec4(0.0f, 0.0f, 0.0f, 0.0f));
	}

	UIPanel::~UIPanel()
	{

	}

	void UIPanel::SetGlobalUI(UIItem* item)
	{
		if (m_global_item) removeChild(m_global_item);
		m_global_item = item;
		if (m_global_item) addChild(m_global_item);
	}

	void UIPanel::AddUI(UIItem* item)
	{
		if (item)
		{
			m_local_items.push_back(item);
			addChild(item);
		}
	}

	void UIPanel::RemoveUI(UIItem* item)
	{
		std::vector<osg::ref_ptr<UIItem>>::iterator it = std::find(m_local_items.begin(), m_local_items.end(), item);
		if (it != m_local_items.end())
		{
			m_local_items.erase(it);
			removeChild(item);
		}
	}

	void UIPanel::Resize(float width, float height)
	{
		m_width->set(width);
		m_height->set(height);

		//m_viewport->width() = width;
		//m_viewport->height() = height;
	}

	bool UIPanel::handleFrameEvent(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
	{
		m_global_item->handleFrameEvent(ea, aa);
		std::vector<osg::ref_ptr<UIItem>>::iterator it = m_local_items.begin();
		for (; it != m_local_items.end(); ++it)
			(*it)->handleFrameEvent(ea, aa);
		return false;
	}

	bool UIPanel::handleKeyEvent(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
	{
		bool handled = m_global_item->handleKeyEvent(ea, aa);
		if (handled) return true;

		std::vector<osg::ref_ptr<UIItem>>::iterator it = m_local_items.begin();
		for (; it != m_local_items.end(); ++it)
		{
			handled = (*it)->handleKeyEvent(ea, aa);
			if (handled) break;
		}
		return handled;
	}

	void UIPanel::ProcessHover(float x, float y)
	{
		UIItem* item = m_global_item->Hover(x, y);
		if (!item)
		{
			std::vector<osg::ref_ptr<UIItem>>::iterator it = m_local_items.begin();
			for (; it != m_local_items.end(); ++it)
			{
				item = (*it)->Hover(x, y);
				if (item) break;
			}
		}

		if (m_hover_item && item != m_hover_item)
			m_hover_item->OnUnHover();
		if (item != m_hover_item)
		{
			m_hover_item = item;
			if (m_hover_item) m_hover_item->OnHover();
		}
	}

	bool UIPanel::handleMouseEvent(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
	{
		ProcessHover(ea.getX(), ea.getY());

		osgGA::GUIEventAdapter::EventType type = ea.getEventType();
		if (type == osgGA::GUIEventAdapter::RELEASE ||
			type == osgGA::GUIEventAdapter::PUSH ||
			type == osgGA::GUIEventAdapter::DRAG)
		{
			bool handled = m_global_item->handleMouseEvent(ea, aa);
			if (handled) return true;

			std::vector<osg::ref_ptr<UIItem>>::iterator it = m_local_items.begin();
			for (; it != m_local_items.end(); ++it)
			{
				handled = (*it)->handleMouseEvent(ea, aa);
				if (handled) break;
			}
			return handled;
		}
		return false;
	}

	bool UIPanel::handleDoubleClickEvent(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
	{
		bool handled = m_global_item->handleDoubleClickEvent(ea, aa);
		if (handled) return true;

		std::vector<osg::ref_ptr<UIItem>>::iterator it = m_local_items.begin();
		for (; it != m_local_items.end(); ++it)
		{
			handled = (*it)->handleDoubleClickEvent(ea, aa);
			if (handled) break;
		}
		return handled;
	}
}