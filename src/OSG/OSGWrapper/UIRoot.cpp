#include <osgWrapper\UIRoot.h>
#include <iostream>
#include <osgWrapper\ProgramManager.h>
#include <algorithm>

namespace OSGWrapper
{
	UIRoot::UIRoot()
	{
		getOrCreateStateSet()->setAttribute(ProgramManager::Instance().Get("screenui"), osg::StateAttribute::ON);
		m_width = new osg::Uniform("viewport_width", 1920.0f);
		m_height = new osg::Uniform("viewport_height", 1080.0f);

		getOrCreateStateSet()->addUniform(m_width, osg::StateAttribute::ON);
		getOrCreateStateSet()->addUniform(m_height, osg::StateAttribute::ON);
		getOrCreateStateSet()->setMode(GL_DEPTH_TEST, osg::StateAttribute::OFF);
		getOrCreateStateSet()->setMode(GL_BLEND, osg::StateAttribute::ON);

		setReferenceFrame(osg::Transform::RELATIVE_RF);
		setRenderOrder(osg::Camera::POST_RENDER, 0);
		setClearMask(0);
	}

	UIRoot::~UIRoot()
	{

	}

	void UIRoot::Resize(float width, float height)
	{
		m_width->set(width);
		m_height->set(height);

		std::vector<osg::ref_ptr<UIQuad>>::iterator it = m_quads.begin();
		for (; it != m_quads.end(); ++it)
			(*it)->Resize(width, height);
	}

	bool UIRoot::handleFrameEvent(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
	{
		return true;
	}

	bool UIRoot::handleKeyEvent(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
	{
		return true;
	}

	bool UIRoot::handleMouseEvent(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
	{
		return true;
	}

	bool UIRoot::handleDoubleClickEvent(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
	{
		return true;
	}

	void UIRoot::AddQuad(UIQuad* quad, bool update)
	{
		if (!quad) return;

		std::vector<osg::ref_ptr<UIQuad>>::iterator it = std::find(m_quads.begin(), m_quads.end(), quad);
		if (it != m_quads.end())
		{
			std::cout << "Error: quad already in." << std::endl;
			return;
		}
		m_quads.push_back(quad);
		if (update) Rebuild();
	}

	void UIRoot::RemoveQuad(UIQuad* quad, bool update)
	{
		if (!quad) return;

		std::vector<osg::ref_ptr<UIQuad>>::iterator it = std::find(m_quads.begin(), m_quads.end(), quad);
		if (it != m_quads.end())
		{
			m_quads.erase(it);
			if (update) Rebuild();
		}
	}

	void UIRoot::Rebuild()
	{
		removeChildren(0, getNumChildren());

		std::vector<OSGWrapper::QuadAttributeUtilNode*> nodes;

		std::vector<osg::ref_ptr<UIQuad>>::iterator it = m_quads.begin();
		for (; it != m_quads.end(); ++it)
		{
			QuadAttributeUtilNode* node = (*it)->Generate();
			if (node)
				nodes.push_back(node);
		}

		std::sort(nodes.begin(), nodes.end(), [&nodes](QuadAttributeUtilNode* n1, QuadAttributeUtilNode* n2)->bool {
			return n1->Order() < n2->Order();
		});

		for (size_t i = 0; i < nodes.size(); ++i)
			addChild(nodes.at(i));
	}
}