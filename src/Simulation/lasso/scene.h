#pragma once
#include <osgWrapper/RenderScene.h>
#include <osgWrapper/ArrayCreator.h>
#include <osgWrapper/ManipulableNode.h>
#include <osgWrapper/Manipulator.h>
#include <osgWrapper/AttributeUtilNode.h>

#include "Mesh.h"

class Scene : public OSGWrapper::RenderScene
{
public:
	Scene(const LauncaGeometry::Mesh& mesh);
	~Scene();

protected:
	bool OnMouse(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa);
	bool OnKey(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa);
private:
	void SetupGeometry(const LauncaGeometry::Mesh& mesh);
	void ResetCamera();
protected:
	osg::ref_ptr<osg::Vec3Array> m_coord_array;
	osg::ref_ptr<osg::Vec3Array> m_normal_array;
	osg::ref_ptr<osg::DrawArrays> m_draw_array;
	osg::ref_ptr<osg::Geometry> m_geometry;
	osg::ref_ptr<osg::Geode> m_geode;

	osg::ref_ptr<OSGWrapper::ManipulableNode> m_manipulable_node;
	osg::ref_ptr<OSGWrapper::Manipulator> m_manipulator;
	osg::ref_ptr<OSGWrapper::AttributeUtilNode> m_render_node;
};