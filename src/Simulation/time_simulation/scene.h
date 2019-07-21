#pragma once
#include <osgWrapper\RenderScene.h>
#include <osgWrapper\ManipulableNode.h>
#include <osgWrapper\Manipulator.h>
#include <osgWrapper\AttributeUtilNode.h>
#include "timepanel.h"

class Scene : public OSGWrapper::RenderScene
{
public:
	Scene();
	~Scene();

	void LoadFrom(int argc, char *argv[]);
protected:
	void UpdateCamera();
	bool OnMouse(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa);
	bool OnKey(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa);

	void OnEnterUI(OSGWrapper::UIPanel* panel);
	void OnEnterOutUI(OSGWrapper::UIPanel* panel);
private:
	osg::ref_ptr<OSGWrapper::ManipulableNode> m_manipulable_node;
	osg::ref_ptr<OSGWrapper::Manipulator> m_manipulator;

	osg::ref_ptr<TimePanel> m_time_panel;
};