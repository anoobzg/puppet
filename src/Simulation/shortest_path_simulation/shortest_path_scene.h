#pragma once
#include <osgWrapper\RenderScene.h>
#include <osgWrapper\AttributeUtilNode.h>

#include <osgWrapper\ManipulableNode.h>
#include <osgWrapper\AttributeUtilNode.h>
#include <osgWrapper\Manipulator.h>
#include <osgWrapper\ColorIndexPicker.h>

#include <osg\PolygonMode>
#include <memory>

using namespace OSGWrapper;
class Surface;
class Scene : public RenderScene
{
public:
	Scene(int argc, char* argv[], RenderView* view);
	~Scene();

	void OnEnter();
	void OnEnterOut();

	bool OnMouse(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa);
	bool OnKey(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa);
	bool OnResize(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa);
	bool OnDoubleClick(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa);
private:
	void ResetCamera();
private:
	std::auto_ptr<Surface> m_surface;

	osg::ref_ptr<ManipulableNode> m_manipulable_node;
	osg::ref_ptr<Manipulator> m_manipulator;
	osg::ref_ptr<AttributeUtilNode> m_render_node;
	osg::ref_ptr<AttributeUtilNode> m_path_node;
	osg::ref_ptr<ColorIndexPicker> m_picker;

	osg::ref_ptr<osg::PolygonMode> m_polygon_mode;
};