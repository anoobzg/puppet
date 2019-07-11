#pragma once
#include <osgWrapper\RenderScene.h>
#include <osgWrapper\ManipulableNode.h>
#include <osgWrapper\Manipulator.h>
#include <osgWrapper\AttributeUtilNode.h>
#include "parameterstext.h"

class Scene : public OSGWrapper::RenderScene
{
public:
	Scene(osg::Geometry* geometry);
	~Scene();

protected:
	void UpdateCamera();
	bool OnMouse(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa);
private:
	osg::ref_ptr<OSGWrapper::ManipulableNode> m_manipulable_node;
	osg::ref_ptr<OSGWrapper::Manipulator> m_manipulator;
	osg::ref_ptr<OSGWrapper::AttributeUtilNode> m_layer_node;
	osg::ref_ptr<OSGWrapper::AttributeUtilNode> m_render_node;

	osg::ref_ptr<ParametersText> m_hue;

	std::vector<osg::ref_ptr<ParametersText>> m_parameters;

	osg::ref_ptr<osg::Texture2D> m_texture;
	osg::ref_ptr<osg::Image> m_image;
};